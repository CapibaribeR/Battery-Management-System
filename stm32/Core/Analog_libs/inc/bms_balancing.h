/*
 * bms_balancing.h
 *
 * Descrição: Módulo de balanceamento de células para o ADBMS6830B.
 *
 * Algoritmo: Balanceamento "bottom" (referenciado ao mínimo da pilha):
 *   - Calcula Vmin de toda a cadeia.
 *   - Ativa descarga (bit DCC) para células com V > Vmin + delta.
 *   - Duty cycle PWM proporcional ao número de células em descarga por IC.
 *   - Watchdog de hardware configurado via DCTO (máx. 5 min por segurança).
 *
 * Conformidade FSAE 2026:
 *   - EV.7.3.3: balanceamento de célula não é permitido com o Shutdown
 *     Circuit aberto (verificar isso no nível do chamador/programLoop).
 *   - EV.7.4.1: BMS deve medir a tensão de cada célula individualmente.
 *
 * Referência: ADBMS6830B Datasheet Rev.0, Tabela 11 (CFGB), Seção "Cell Balancing"
 *
 * Criado em: Mar 2026
 * Autor: vtaua
 */

#ifndef ANALOG_LIBS_INC_BMS_BALANCING_H_
#define ANALOG_LIBS_INC_BMS_BALANCING_H_

#include "common.h"
#include "bms_data.h"

/* -----------------------------------------------------------------------
 * Parâmetros padrão (ajustáveis)
 * ----------------------------------------------------------------------- */
#define BAL_DELTA_DEFAULT_V   0.010f  /* Diferença mínima para iniciar descarga (V) */
#define BAL_PWM_DUTY_DEFAULT  0x0F    /* Duty 100 % (4-bit: 0x0=0%, 0xF=100%)      */
#define BAL_DCTO_MIN          1       /* Timer de descarga HW: 1 minuto (DTRNG=0)   */
#define BAL_DCTO_MAX          5       /* Timer máximo seguro: 5 minutos              */
#define BAL_MAX_CONCURRENT    8       /* Máximo de células em descarga simultânea    */

/* -----------------------------------------------------------------------
 * Estado do balanceador (por chamada a bms_balancing_compute_stats)
 * ----------------------------------------------------------------------- */
typedef struct {
	float v_min; /* Tensão mínima global (toda a cadeia)  */
	float v_max; /* Tensão máxima global                  */
	float v_avg; /* Tensão média global                   */
	float v_delta; /* v_max − v_min                         */
	float v_min_per_ic[TOTAL_IC]; /* Tensão mínima por IC                  */
	float v_max_per_ic[TOTAL_IC]; /* Tensão máxima por IC                  */
	bool cell_active[TOTAL_IC][TOTAL_CELL]; /* true = célula em descarga   */
	bool balancing_done; /* true quando delta < threshold         */
} bms_bal_state_t;

/* -----------------------------------------------------------------------
 * API pública
 * ----------------------------------------------------------------------- */

/**
 * @brief Calcula estatísticas de tensão (min/max/avg/delta) da cadeia.
 * @param volt   Matriz de tensões float [TOTAL_IC][TOTAL_CELL]
 * @param state  Estado de saída
 */
void bms_balancing_compute_stats(float volt[TOTAL_IC][TOTAL_CELL],
		bms_bal_state_t *state);

/**
 * @brief Determina quais células devem ser descarregadas.
 *        Atualiza state->cell_active[][] e state->balancing_done.
 * @param volt      Matriz de tensões [TOTAL_IC][TOTAL_CELL]
 * @param state     Estado do balanceador
 * @param delta_v   Limiar de diferença para descarga
 */
void bms_balancing_update(float volt[TOTAL_IC][TOTAL_CELL],
		bms_bal_state_t *state, float delta_v);

/**
 * @brief Aplica configuração de balanceamento no hardware via SPI.
 *        Escreve CFGB (bits DCC), WRPWMA e WRPWMB para todos os ICs.
 * @param cfb_Tx   Array de registradores CFGB a escrever [TOTAL_IC]
 * @param pwma     Array de registradores PWMA [TOTAL_IC]
 * @param pwmb     Array de registradores PWMB [TOTAL_IC]
 * @param state    Estado atual do balanceador
 */
void bms_balancing_apply(ad68_cfb_t cfb_Tx[TOTAL_IC],
		ad68_pwma_t pwma[TOTAL_IC], ad68_pwmb_t pwmb[TOTAL_IC],
		const bms_bal_state_t *state);

/**
 * @brief Para toda descarga: limpa DCC, zera PWM, zera DCTO.
 */
void bms_balancing_stop_all(ad68_cfb_t cfb_Tx[TOTAL_IC],
		ad68_pwma_t pwma[TOTAL_IC], ad68_pwmb_t pwmb[TOTAL_IC]);

/**
 * @brief Define duty cycle PWM para todas as células de um IC.
 * @param pwma     Registrador PWMA do IC
 * @param pwmb     Registrador PWMB do IC
 * @param duty     Valor 4-bit (0x0–0xF)
 */
void bms_balancing_set_pwm_ic(ad68_pwma_t *pwma, ad68_pwmb_t *pwmb,
		uint8_t duty);

/**
 * @brief Define duty cycle PWM de uma célula específica.
 */
void bms_balancing_set_pwm_cell(ad68_pwma_t *pwma, ad68_pwmb_t *pwmb,
		uint8_t cell_0idx, uint8_t duty);

/**
 * @brief Codifica tensão float em código de 12 bits para VUV/VOV do CFGB.
 *        Fórmula (datasheet Table 103):
 *          code = (V − 1.5) / (16 × 150e-6)
 *        Resultado clampado para [0, 0xFFF].
 */
uint16_t bms_encode_vuv(float v_volts);
uint16_t bms_encode_vov(float v_volts);

/**
 * @brief Task de alto nível: stats → update → apply.
 *        Deve ser chamada periodicamente no loop principal.
 * @return true se ainda há células acima do limiar, false se balanceado.
 */
bool bms_balancing_task(float volt[TOTAL_IC][TOTAL_CELL],
		ad68_cfb_t cfb_Tx[TOTAL_IC], ad68_pwma_t pwma[TOTAL_IC],
		ad68_pwmb_t pwmb[TOTAL_IC], bms_bal_state_t *state, float delta_v);

#endif /* ANALOG_LIBS_INC_BMS_BALANCING_H_ */
