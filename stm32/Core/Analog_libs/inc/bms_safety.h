/*
 * bms_safety.h
 *
 * Descrição: Módulo de segurança para o ADBMS6830B.
 *            Implementa verificações de:
 *              - Sobretensão / Subtensão (SW check + HW flags)
 *              - Sobretemperatura do DIE do IC (auto-diagnóstico interno;
 *                temperatura de CÉLULA é responsabilidade do TMS dedicado
 *                do acumulador, fora deste módulo)
 *              - Fio aberto / Open Wire (método canais pares/ímpares)
 *
 * Conformidade FSAE 2026 (verificado contra FSAE_Rules_2026_V1.pdf):
 *   - EV.7.3.4 : BMS deve monitorar OV/UV/temp/comms/falha interna.
 *   - EV.7.3.5 : Ao detectar falha, BMS deve abrir o Shutdown Circuit
 *                (EV.7.2.2) e acender o BMS Indicator Light.
 *   - EV.7.4.2 : Limites de tensão de célula vêm do datasheet da célula
 *                (BMS_OV_THRESHOLD_V / BMS_UV_THRESHOLD_V devem refletir
 *                isso, não um valor genérico).
 *   NOTA: EV.5.7 é o item "Voltage Indicator" (lâmpada HV hardwired) e
 *   T.9.1 é a seção de Definições -- nenhum dos dois define um tempo de
 *   resposta numérico para OV/UV. Não há requisito explícito de "500 ms"
 *   no texto da regra 2026 V1.0; confirme contra o ESF do time se esse
 *   número vem de outra fonte (ano anterior, regra interna, etc.).
 *
 * Referência: ADBMS6830B Datasheet Rev.0, seções Status Registers, Open Wire,
 *             Thermal Shutdown, GPIO and Device Parameter Measurements (ITMP).
 *
 * Criado em: Mar 2026
 * Autor: vtaua
 */

#ifndef ANALOG_LIBS_INC_BMS_SAFETY_H_
#define ANALOG_LIBS_INC_BMS_SAFETY_H_

#include "common.h"
#include "bms_data.h"

/* Sem NTC neste IC -- temperatura de célula é responsabilidade do TMS
 * dedicado do acumulador. Bloco de parâmetros NTC removido. */

/* -----------------------------------------------------------------------
 * Bitmask de falhas
 * ----------------------------------------------------------------------- */
typedef enum {
    BMS_FAULT_NONE         = 0x00,
    BMS_FAULT_OV           = 0x01,   /* Sobretensão                        */
    BMS_FAULT_UV           = 0x02,   /* Subtensão                          */
    BMS_FAULT_DIE_OVERTEMP = 0x04,   /* Sobretemperatura do DIE do IC      */
    BMS_FAULT_OPEN_WIRE    = 0x08,   /* Fio aberto                         */
    BMS_FAULT_COMMS        = 0x10,   /* Falha de comunicação               */
    BMS_FAULT_INTERNAL     = 0x20,   /* Falha interna do ADBMS6830B        */
} bms_fault_t;

/* -----------------------------------------------------------------------
 * Estado de falha global
 * ----------------------------------------------------------------------- */
typedef struct {
    uint32_t fault_mask;                          /* Bitmask de falhas ativas (bms_fault_t) */
    bool     ov_cell[TOTAL_IC][TOTAL_CELL];        /* Célula em OV             */
    bool     uv_cell[TOTAL_IC][TOTAL_CELL];        /* Célula em UV             */
    bool     die_ot[TOTAL_IC];                     /* IC com die em sobretemp. */
    bool     ow_cell[TOTAL_IC][TOTAL_CELL];        /* Open wire detectado      */
    float    temp_die[TOTAL_IC];                   /* Temperatura do die (°C)  */
} bms_safety_state_t;

/* -----------------------------------------------------------------------
 * API pública
 * ----------------------------------------------------------------------- */

/**
 * @brief Verifica OV e UV por software comparando com thresholds.
 *        Atualiza state->fault_mask com BMS_FAULT_OV / BMS_FAULT_UV.
 * @param volt   Matriz de tensões [TOTAL_IC][TOTAL_CELL]
 * @param state  Estado de segurança a atualizar
 * @return BMS_OK se nenhuma falha, BMS_ERR_VOLTAGE caso contrário.
 */
BMS_StatusTypeDef bms_safety_check_voltage(float volt[TOTAL_IC][TOTAL_CELL],
                                           bms_safety_state_t *state);

/**
 * @brief Verifica sobretemperatura do DIE do IC (não da célula -- isso é
 *        responsabilidade do TMS externo).
 * @param temp_die  Temperatura do die por IC [TOTAL_IC], já em °C
 *                  (calculada via ITMP_TO_CELSIUS em comm.c)
 * @param state  Estado de segurança
 * @return BMS_OK ou BMS_ERR_TEMP
 */
BMS_StatusTypeDef bms_safety_check_die_temperature(float temp_die[TOTAL_IC],
                                                   bms_safety_state_t *state);

/**
 * @brief Detecção de fio aberto via método canais pares/ímpares (OW[1:0]).
 *        Sequência: MUTE → ADCV(OW=pares) → lê → ADCV(OW=ímpares) → lê → UNMUTE.
 *        Células com |V_pares − V_ímpares| > BMS_OW_DELTA_V marcadas como OW
 *        (comparação feita por par de canais vizinhos, conforme datasheet
 *        seção "Cell Open Wire Detection").
 *
 * @param volt_norm  Tensões de referência (OW desligado) [TOTAL_IC][TOTAL_CELL]
 * @param volt_even  Tensões medidas com canais pares habilitados [TOTAL_IC][TOTAL_CELL]
 * @param volt_odd   Tensões medidas com canais ímpares habilitados [TOTAL_IC][TOTAL_CELL]
 * @param state      Estado de segurança
 * @return BMS_OK ou BMS_ERR_OPEN_WIRE
 * Critério: queda > BMS_OW_DROP_MAX_FRAC da tensão de referência.
 */
BMS_StatusTypeDef bms_safety_check_open_wire(float volt_norm[TOTAL_IC][TOTAL_CELL],
                                             float volt_even[TOTAL_IC][TOTAL_CELL],
                                             float volt_odd[TOTAL_IC][TOTAL_CELL],
                                             bms_safety_state_t *state);

/**
 * @brief Lê flags de hardware do RDSTATC/RDSTATD e atualiza state->fault_mask.
 * @param state  Estado de segurança
 * @return BMS_OK (status de comunicação deve ser checado via checkPecFault
 *         internamente; falhas de PEC aqui não abortam a leitura do outro
 *         registrador, mas idealmente deveriam propagar BMS_ERR_COMMS --
 *         ver nota em bms_safety.c)
 */
BMS_StatusTypeDef bms_safety_check_hw_flags(bms_safety_state_t *state);

/**
 * @brief Task de segurança completa: tensão + temp. de die + OW (se habilitado).
 * @param volt       Tensões de célula
 * @param temp_die   Temperatura do die por IC [TOTAL_IC]
 * @param check_ow   Se true, executa verificação de open wire
 * @param state      Estado de segurança
 * @return BMS_OK ou código de erro
 */
BMS_StatusTypeDef bms_safety_task(float volt[TOTAL_IC][TOTAL_CELL],
                                  float temp_die[TOTAL_IC],
                                  bool  check_ow,
                                  bms_safety_state_t *state);

/**
 * @brief Aciona desligamento de segurança: abre o Shutdown Circuit (EV.7.2.2)
 *        via GPIO e (TODO) sinaliza falha por CAN.
 *
 * Também acende a luz BMS (BMS_LED) e trava o rearme até reset do MCU.
 */
void bms_safety_trigger_shutdown(bms_fault_t fault);

/**
 * @brief Fecha o Shutdown Circuit (SDC_CTRL = alto). BMS é normalmente aberto
 *        (EV.7.1.3): só deve ser chamada depois de init e verificações OK.
 *        Recusada (retorna false) se já houve shutdown desde o último reset
 *        do MCU (EV.7.2.3: só rearma por reset manual).
 */
bool bms_safety_close_sdc(void);

/** @brief true se o SDC_SENSE indica SDC fechado (qualquer componente). */
bool bms_safety_sdc_is_closed(void);

/** @brief Limpa as flags sticky do ADBMS6830B (CLRFLAG e CLOVUV). */
void bms_safety_clear_hw_flags(void);

/**
 * @brief Limpa todos os flags de falha. Chamado por bms_init()/bms_reset()
 *        (reset manual do BMS, conforme EV.7.2.3).
 */
void bms_safety_clear_faults(bms_safety_state_t *state);

#endif /* ANALOG_LIBS_INC_BMS_SAFETY_H_ */
