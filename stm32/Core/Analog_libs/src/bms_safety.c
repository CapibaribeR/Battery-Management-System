/*
 * bms_safety.c
 *
 * Descrição: Implementação do módulo de segurança.
 *
 * Funções implementadas:
 *   [OK] bms_safety_check_voltage        – verificação SW de OV/UV
 *   [OK] bms_safety_check_die_temperature – sobretemperatura do die do IC
 *   [OK] bms_safety_check_open_wire      – algoritmo canais pares/ímpares
 *   [OK] bms_safety_check_hw_flags       – leitura RDSTATC/RDSTATD
 *   [OK] bms_safety_trigger_shutdown     – abre shutdown circuit via GPIO
 *        (placeholder de pino -- ver aviso abaixo)
 *
 * Conformidade FSAE 2026: ver bms_safety.h para citações corretas
 * (EV.7.3.4 / EV.7.3.5). EV.5.7 e T.9.1 NÃO se aplicam aqui -- ver nota
 * em bms_safety.h.
 *
 * Referência: ADBMS6830B Datasheet Rev.0, seções 7.3 (Status Registers),
 *             8.4 (Open Wire Detection), Thermal Shutdown.
 *
 * Criado em: Mar 2026
 * Modificado em: jun 2026
 * Autor: vtaua
 */

#include "../inc/bms_safety.h"
#include "../inc/bms_balancing.h"
#include "../inc/bms_generic.h"
#include "../inc/bms_data.h"
#include "../inc/common.h"
#include "../inc/communication.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* -----------------------------------------------------------------------
 * @brief Verificação de Tensão SW (OV / UV)
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_check_voltage(float volt[TOTAL_IC][TOTAL_CELL],
                                           bms_safety_state_t *state)
{
    BMS_StatusTypeDef result = BMS_OK;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < TOTAL_CELL; c++) {
            float v = volt[ic][c];

            if (v > BMS_OV_THRESHOLD_V) {
                state->ov_cell[ic][c] = true;
                state->fault_mask |= BMS_FAULT_OV;
                result = BMS_ERR_VOLTAGE;
            } else {
                state->ov_cell[ic][c] = false;
            }

            if (v < BMS_UV_THRESHOLD_V) {
                state->uv_cell[ic][c] = true;
                state->fault_mask |= BMS_FAULT_UV;
                result = BMS_ERR_VOLTAGE;
            } else {
                state->uv_cell[ic][c] = false;
            }
        }
    }
    return result;
}

/* -----------------------------------------------------------------------
 * @brief Verificação de sobretemperatura do DIE (auto-diagnóstico do IC).
 * Temperatura de célula é responsabilidade do TMS dedicado, fora deste
 * módulo.
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_check_die_temperature(float temp_die[TOTAL_IC],
                                                   bms_safety_state_t *state)
{
    BMS_StatusTypeDef result = BMS_OK;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        float t = temp_die[ic];
        state->temp_die[ic] = t;

        if (t > BMS_DIE_TEMP_MAX_C || t < BMS_DIE_TEMP_MIN_C) {
            state->die_ot[ic] = true;
            state->fault_mask |= BMS_FAULT_DIE_OVERTEMP;
            result = BMS_ERR_TEMP;
        } else {
            state->die_ot[ic] = false;
        }
    }
    return result;
}

/* -----------------------------------------------------------------------
 * @brief Detecção de Open Wire
 *
 * Procedimento (datasheet seção "Cell Open Wire Detection"):
 *   1. MUTE    – desabilita descarga passiva para não interferir na medição.
 *   2. ADCV com OW[1:0]=01 (canais pares)   → lê tensões volt_even.
 *   3. ADCV com OW[1:0]=10 (canais ímpares) → lê tensões volt_odd.
 *   4. UNMUTE  – reabilita descarga.
 *   5. Para cada célula: se |Vpares − Vímpares| > BMS_OW_DELTA_V → fio aberto.
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_check_open_wire(float volt_norm[TOTAL_IC][TOTAL_CELL],
                                             float volt_even[TOTAL_IC][TOTAL_CELL],
                                             float volt_odd[TOTAL_IC][TOTAL_CELL],
                                             bms_safety_state_t *state)
{
    BMS_StatusTypeDef result = BMS_OK;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < TOTAL_CELL; c++) {
            /* Com a chave ligada, o canal íntegro lê ~10/12 da tensão normal
             * (datasheet, Cell Open Wire Detection); com fio aberto o
             * capacitor de entrada descarrega e a leitura cai para ~0.
             * Canal (c+1) par -> chave ativa na passada "pares". */
            float v_on   = ((c + 1) % 2 == 0) ? volt_even[ic][c] : volt_odd[ic][c];
            float v_norm = volt_norm[ic][c];
            float drop   = v_norm - v_on;
            if (drop > BMS_OW_DROP_MAX_FRAC * v_norm) {
                state->ow_cell[ic][c] = true;
                state->fault_mask |= BMS_FAULT_OPEN_WIRE;
                result = BMS_ERR_OPEN_WIRE;
            } else {
                state->ow_cell[ic][c] = false;
            }
        }
    }
    return result;
}

/* -----------------------------------------------------------------------
 * @brief Leitura de flags de hardware
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_check_hw_flags(bms_safety_state_t *state)
{
    /* Falhas internas em STATC byte 4 (STCR4): VA_OV, VA_UV, VD_OV, VD_UV,
     * CMED, SMED. CED/SED são erros de trim de bit único, corrigidos pelo chip. */
    const uint8_t STCR4_FAULTS = 0xF5;
    /* STATC byte 5 (STCR5): VDE, SPIFLT, THSD, TMODCHK, OSCCHK.
     * VDEL e COMP não são falhas; SLEEP é limpo no init. */
    const uint8_t STCR5_FAULTS = 0x57;

    uint8_t rx_data[TOTAL_IC * RX_DATA];
    uint8_t pec_err[TOTAL_IC];
    uint8_t cmd_cntr[TOTAL_IC];
    BMS_StatusTypeDef result = BMS_OK;

    /* CSxFLT (STATC bytes 0-1) só vale com a comparação C-ADC x S-ADC ativa
     * (ADCV com RD=1), que este firmware não usa; por isso não é avaliado. */
    spiReadData(TOTAL_IC, RDSTATC, rx_data, pec_err, cmd_cntr, RX_DATA);
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        if (pec_err[ic]) {
            state->fault_mask |= BMS_FAULT_COMMS;
            result = BMS_ERR_COMMS;
        } else if ((rx_data[ic * RX_DATA + 4] & STCR4_FAULTS) ||
                   (rx_data[ic * RX_DATA + 5] & STCR5_FAULTS)) {
            state->fault_mask |= BMS_FAULT_INTERNAL;
            if (result == BMS_OK) result = BMS_ERR_INTERNAL;
        }
    }

    /* STATD: flags sticky de OV/UV por célula (Tabela 92). Em cada byte,
     * bits ímpares = OV (0xAA), pares = UV (0x55). */
    spiReadData(TOTAL_IC, RDSTATD, rx_data, pec_err, cmd_cntr, RX_DATA);
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        if (pec_err[ic]) {
            state->fault_mask |= BMS_FAULT_COMMS;
            result = BMS_ERR_COMMS;
            continue;
        }
        for (int b = 0; b < 4; b++) {
            uint8_t v = rx_data[ic * RX_DATA + b];
            if (v & 0xAA) {
                state->fault_mask |= BMS_FAULT_OV;
                if (result == BMS_OK) result = BMS_ERR_VOLTAGE;
            }
            if (v & 0x55) {
                state->fault_mask |= BMS_FAULT_UV;
                if (result == BMS_OK) result = BMS_ERR_VOLTAGE;
            }
        }
    }

    return result;
}

/* -----------------------------------------------------------------------
 * @brief Task completa de segurança
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_task(float volt[TOTAL_IC][TOTAL_CELL],
                                  float temp_die[TOTAL_IC],
                                  bool  check_ow,
                                  bms_safety_state_t *state)
{
    BMS_StatusTypeDef status = BMS_OK;

    /* 1. Verificação de tensão SW */
    BMS_StatusTypeDef v_status = bms_safety_check_voltage(volt, state);
    if (v_status != BMS_OK) status = v_status;

    /* 2. Verificação de temperatura do die */
    BMS_StatusTypeDef t_status = bms_safety_check_die_temperature(temp_die, state);
    if (t_status != BMS_OK) status = t_status;

    /* 3. Verificação de flags HW (RDSTATC/RDSTATD) */
    BMS_StatusTypeDef h_status = bms_safety_check_hw_flags(state);
    if (h_status != BMS_OK) status = h_status;

    /* 4. Open wire (opcional – demorado ~10-20 ms) */
    if (check_ow) {
        /* readCellVoltage(VOLTAGE_C) grava em bms.volt_cell[VOLTAGE_C], não
         * no array `volt` (S-ADC) recebido por parâmetro. */
        float (*vc)[TOTAL_CELL] = bms_getVoltPtr(VOLTAGE_C);

        /* [0]=referência (OW=00), [1]=pares ligados (OW=01), [2]=ímpares (OW=10) */
        float volt_ow[3][TOTAL_IC][TOTAL_CELL];
        bool  ow_comm_ok = true;

        spiSendCmd(MUTE);
        for (uint8_t m = 0; m < 3; m++) {
            ADCV.ow   = m;
            ADCV.cont = 0;
            spiSendCmd((uint8_t *)&ADCV);
            if (bmsPollAdc(PLCADC) == UINT32_MAX ||
                readCellVoltage(VOLTAGE_C) != BMS_OK) {
                ow_comm_ok = false;
                break;
            }
            memcpy(volt_ow[m], vc, sizeof(volt_ow[m]));
        }
        spiSendCmd(UNMUTE);

        /* Restaura C-ADC contínuo */
        ADCV.ow   = 0;
        ADCV.cont = 1;
        spiSendCmd((uint8_t *)&ADCV);

        if (!ow_comm_ok) {
            state->fault_mask |= BMS_FAULT_COMMS;
            status = BMS_ERR_COMMS;
        } else {
            BMS_StatusTypeDef ow_status =
                bms_safety_check_open_wire(volt_ow[0], volt_ow[1], volt_ow[2], state);
            if (ow_status != BMS_OK) status = ow_status;
        }
    }

    /* 5. Aciona desligamento se necessário */
    if (status != BMS_OK) {
        bms_safety_trigger_shutdown((bms_fault_t)state->fault_mask);
    }

    return status;
}

/* -----------------------------------------------------------------------
 * @brief Trigger de shutdown – sinaliza GPIO do shutdown circuit.
 *
 * Pinos (main.h / .ioc): SDC_CTRL (PC8, saída), SDC_SENSE (PC9, entrada com
 * pull-down) e BMS_LED (PC6, saída).
 *
 * Lógica: SDC_CTRL alto = SDC fechado, baixo = aberto (BMS normalmente
 * aberto, EV.7.1.3). Use pull-down externo em SDC_CTRL para o SDC ficar
 * aberto durante reset/travamento do MCU. SDC_SENSE alto = SDC fechado.
 * Confirme a polaridade do driver do AIR e do LED antes de ligar em hardware.
 * ----------------------------------------------------------------------- */
/* Latch de shutdown: só zera com reset do MCU (EV.7.2.3, rearme manual). */
static bool s_shutdown_latched = false;

void bms_safety_trigger_shutdown(bms_fault_t fault)
{
    s_shutdown_latched = true;

    /* Abre o Shutdown Circuit (baixo = aberto) e acende a luz BMS (EV.7.3.5) */
    HAL_GPIO_WritePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BMS_LED_GPIO_Port,  BMS_LED_Pin,  GPIO_PIN_SET);

    /* TODO: Transmitir código de falha via CAN para telemetria/dashboard. */
    (void)fault;
}

bool bms_safety_close_sdc(void)
{
    if (s_shutdown_latched) return false;
    HAL_GPIO_WritePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_PIN_SET);
    return true;
}

bool bms_safety_sdc_is_closed(void)
{
    /* Pull-down interno: pino desconectado lê "aberto" (seguro) */
    return HAL_GPIO_ReadPin(SDC_SENSE_GPIO_Port, SDC_SENSE_Pin) == GPIO_PIN_SET;
}

void bms_safety_clear_hw_flags(void)
{
    uint8_t tx[TOTAL_IC][TX_DATA];

    /* Bit = 1 limpa o flag correspondente (Tabelas 25 e 26) */
    memset(tx, 0xFF, sizeof(tx));
    spiWriteData(TOTAL_IC, CLRFLAG, (uint8_t *)tx);
    spiWriteData(TOTAL_IC, CLOVUV,  (uint8_t *)tx);
}

/* -----------------------------------------------------------------------
 * @brief Limpar falhas
 * ----------------------------------------------------------------------- */
void bms_safety_clear_faults(bms_safety_state_t *state) {
    state->fault_mask = BMS_FAULT_NONE;
    memset(state->ov_cell,  0, sizeof(state->ov_cell));
    memset(state->uv_cell,  0, sizeof(state->uv_cell));
    memset(state->die_ot,   0, sizeof(state->die_ot));
    memset(state->ow_cell,  0, sizeof(state->ow_cell));
}
