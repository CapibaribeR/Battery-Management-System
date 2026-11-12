/*
 * bms_balancing.c
 *
 * Descrição: Implementação do módulo de balanceamento de células.
 *
 * Algoritmo "Bottom Balancing":
 *   1. Encontra Vmin global (menor tensão de toda a cadeia).
 *   2. Ativa bit DCC para células com V > Vmin + delta_v.
 *   3. Duty cycle PWM = maxDischarge / cellDischargeCount (proporcional).
 *   4. Watchdog de hardware: DCTO = BAL_DCTO_MIN minutos por segurança.
 *   5. Verifica estado de balanceamento: done quando delta < delta_v.
 *
 * Conformidade:
 *   - FSAE 2026 EV.5.7: proteção OV/UV ativa durante balanceamento.
 *   - Datasheet ADBMS6830B Rev.0 Table 11: campos DCC, DCTO, DTRNG, DTMEN.
 *
 * Criado em: Mar 2026
 * Autor: vtaua
 */

#include "../inc/bms_balancing.h"
#include "../inc/bms_generic.h"
#include "../inc/bms_data.h"
#include "../inc/common.h"
#include <string.h>
#include <math.h>

/* -----------------------------------------------------------------------
 * Helpers internos
 * ----------------------------------------------------------------------- */

/**
 * @brief Aplica valor PWM de 4 bits em um canal específico de um IC.
 */
void bms_balancing_set_pwm_cell(ad68_pwma_t *pwma, ad68_pwmb_t *pwmb,
                                uint8_t cell_0idx, uint8_t duty)
{
    duty &= 0x0F;   /* garante 4 bits */
    switch (cell_0idx) {
        case  0: pwma->pwm1  = duty; break;
        case  1: pwma->pwm2  = duty; break;
        case  2: pwma->pwm3  = duty; break;
        case  3: pwma->pwm4  = duty; break;
        case  4: pwma->pwm5  = duty; break;
        case  5: pwma->pwm6  = duty; break;
        case  6: pwma->pwm7  = duty; break;
        case  7: pwma->pwm8  = duty; break;
        case  8: pwma->pwm9  = duty; break;
        case  9: pwma->pwm10 = duty; break;
        case 10: pwma->pwm11 = duty; break;
        case 11: pwma->pwm12 = duty; break;
        case 12: pwmb->pwm13 = duty; break;
        case 13: pwmb->pwm14 = duty; break;
        case 14: pwmb->pwm15 = duty; break;
        case 15: pwmb->pwm16 = duty; break;
        default: break;
    }
}

void bms_balancing_set_pwm_ic(ad68_pwma_t *pwma, ad68_pwmb_t *pwmb, uint8_t duty)
{
    for (uint8_t c = 0; c < TOTAL_CELL; c++) {
        bms_balancing_set_pwm_cell(pwma, pwmb, c, duty);
    }
}

/* -----------------------------------------------------------------------
 * @brief Codificação de limites VUV / VOV
 * Fórmula (datasheet Table 103):
 *   threshold_V = code12 × 16 × 150e-6 + 1.5
 *   code12      = (threshold_V − 1.5) / (16 × 150e-6)
 * Resultado: inteiro 12-bit [0x000 – 0xFFF]
 *
 * CORRIGIDO (Mar 2026): fórmula anterior (V / (16×150e-6) − 1) estava
 * invertida e não subtraía o offset de 1.5V do datasheet, gerando
 * thresholds muito acima do valor configurado (ex.: 2.5V codificava
 * para ~5.5V na prática) -- a proteção de UV em hardware nunca disparava.
 * ----------------------------------------------------------------------- */
uint16_t bms_encode_vuv(float v_volts) {
    int32_t code = (int32_t)((v_volts - 1.5f) / (16.0f * 150e-6f));
    if (code < 0)      code = 0;
    if (code > 0xFFF)  code = 0xFFF;
    return (uint16_t)code;
}

uint16_t bms_encode_vov(float v_volts) {
    int32_t code = (int32_t)((v_volts - 1.5f) / (16.0f * 150e-6f));
    if (code < 0)      code = 0;
    if (code > 0xFFF)  code = 0xFFF;
    return (uint16_t)code;
}

/* -----------------------------------------------------------------------
 * @brief Cálculo de estatísticas
 * ----------------------------------------------------------------------- */
void bms_balancing_compute_stats(float volt[TOTAL_IC][TOTAL_CELL],
                                 bms_bal_state_t *state)
{
    float g_min =  9999.0f;
    float g_max = -9999.0f;
    float g_sum =  0.0f;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        float ic_min =  9999.0f;
        float ic_max = -9999.0f;

        for (int c = 0; c < TOTAL_CELL; c++) {
            float v = volt[ic][c];
            if (v < ic_min) ic_min = v;
            if (v > ic_max) ic_max = v;
            if (v < g_min)  g_min  = v;
            if (v > g_max)  g_max  = v;
            g_sum += v;
        }

        state->v_min_per_ic[ic] = ic_min;
        state->v_max_per_ic[ic] = ic_max;
    }

    state->v_min   = g_min;
    state->v_max   = g_max;
    state->v_avg   = g_sum / (float)(TOTAL_IC * TOTAL_CELL);
    state->v_delta = g_max - g_min;
}

/* -----------------------------------------------------------------------
 * @brief Determinação de células a descarregar
 * ----------------------------------------------------------------------- */
void bms_balancing_update(float volt[TOTAL_IC][TOTAL_CELL],
                          bms_bal_state_t *state,
                          float delta_v)
{
    bms_balancing_compute_stats(volt, state);

    if (state->v_delta <= delta_v) {
        /* Cadeia já balanceada */
        memset(state->cell_active, 0, sizeof(state->cell_active));
        state->balancing_done = true;
        return;
    }
    state->balancing_done = false;

    float threshold = state->v_min + delta_v;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < TOTAL_CELL; c++) {
            state->cell_active[ic][c] = (volt[ic][c] > threshold);
        }
    }
}

/* -----------------------------------------------------------------------
 * @brief Escrita no hardware (CFGB + PWMA + PWMB)
 * ----------------------------------------------------------------------- */
void bms_balancing_apply(ad68_cfb_t cfb_Tx[TOTAL_IC],
                         ad68_pwma_t pwma[TOTAL_IC],
                         ad68_pwmb_t pwmb[TOTAL_IC],
                         const bms_bal_state_t *state)
{
    /* Máximo de descarga = metade do duty total compartilhado */
    const uint8_t MAX_DUTY = 0x07;   /* 4-bit: ~50 % do ciclo */

    uint8_t txCfgB[TOTAL_IC][TX_DATA];
    uint8_t txPwmA[TOTAL_IC][TX_DATA];
    uint8_t txPwmB[TOTAL_IC][TX_DATA];

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        /* Conta células em descarga neste IC */
        uint32_t count = 0;
        for (int c = 0; c < TOTAL_CELL; c++) {
            if (state->cell_active[ic][c]) count++;
        }
        if (count == 0) count = 1;   /* evita divisão por zero */

        /* Duty cycle proporcional (não ultrapassa MAX_DUTY) */
        uint8_t duty = (uint8_t)((MAX_DUTY * 16) / count);
        if (duty > 0x0F) duty = 0x0F;

        /* Monta bits DCC e PWM */
        uint16_t dcc = 0;
        for (int c = 0; c < TOTAL_CELL; c++) {
            if (state->cell_active[ic][c]) {
                BIT_SET(dcc, c);
                bms_balancing_set_pwm_cell(&pwma[ic], &pwmb[ic], c, duty);
            } else {
                bms_balancing_set_pwm_cell(&pwma[ic], &pwmb[ic], c, 0);
            }
        }

        /* Atualiza CFGB: DCC bits e timer de descarga */
        cfb_Tx[ic].dcc1_8  = (uint8_t)(dcc & 0xFF);
        cfb_Tx[ic].dcc9_16 = (uint8_t)((dcc >> 8) & 0xFF);
        cfb_Tx[ic].dcto    = BAL_DCTO_MIN;   /* watchdog HW */
        cfb_Tx[ic].dtrng   = 0;              /* unidade = minutos */
        cfb_Tx[ic].dtmen   = 0;              /* monitor desabilitado */

        memcpy(txCfgB[ic], &cfb_Tx[ic], TX_DATA);
        memcpy(txPwmA[ic], &pwma[ic],   TX_DATA);
        memcpy(txPwmB[ic], &pwmb[ic],   TX_DATA);
    }

    /* Envia para o hardware */
    spiWriteData(TOTAL_IC, WRCFGB, (uint8_t *)txCfgB);
    spiWriteData(TOTAL_IC, WRPWMA, (uint8_t *)txPwmA);
    spiWriteData(TOTAL_IC, WRPWMB, (uint8_t *)txPwmB);
}

/* -----------------------------------------------------------------------
 * @brief Para toda descarga
 * ----------------------------------------------------------------------- */
void bms_balancing_stop_all(ad68_cfb_t cfb_Tx[TOTAL_IC],
                            ad68_pwma_t pwma[TOTAL_IC],
                            ad68_pwmb_t pwmb[TOTAL_IC])
{
    uint8_t txCfgB[TOTAL_IC][TX_DATA];
    uint8_t txPwmA[TOTAL_IC][TX_DATA];
    uint8_t txPwmB[TOTAL_IC][TX_DATA];

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        cfb_Tx[ic].dcc1_8  = 0;
        cfb_Tx[ic].dcc9_16 = 0;
        cfb_Tx[ic].dcto    = 0;
        cfb_Tx[ic].dtmen   = 0;
        bms_balancing_set_pwm_ic(&pwma[ic], &pwmb[ic], 0);

        memcpy(txCfgB[ic], &cfb_Tx[ic], TX_DATA);
        memcpy(txPwmA[ic], &pwma[ic],   TX_DATA);
        memcpy(txPwmB[ic], &pwmb[ic],   TX_DATA);
    }

    spiWriteData(TOTAL_IC, WRCFGB, (uint8_t *)txCfgB);
    spiWriteData(TOTAL_IC, WRPWMA, (uint8_t *)txPwmA);
    spiWriteData(TOTAL_IC, WRPWMB, (uint8_t *)txPwmB);
}

/* -----------------------------------------------------------------------
 * @brief Task de alto nível
 * ----------------------------------------------------------------------- */
bool bms_balancing_task(float volt[TOTAL_IC][TOTAL_CELL],
                        ad68_cfb_t cfb_Tx[TOTAL_IC],
                        ad68_pwma_t pwma[TOTAL_IC],
                        ad68_pwmb_t pwmb[TOTAL_IC],
                        bms_bal_state_t *state,
                        float delta_v)
{
    bms_balancing_update(volt, state, delta_v);

    if (state->balancing_done) {
        bms_balancing_stop_all(cfb_Tx, pwma, pwmb);
        return false;
    }

    bms_balancing_apply(cfb_Tx, pwma, pwmb, state);
    return true;
}
