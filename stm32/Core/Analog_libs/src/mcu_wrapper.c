/*
 * mcu_wrapper.c
 *
 * Descrição: Implementação da camada de abstração de hardware.
 *
 * FIXs em relação ao arquivo anterior:
 *   1. '#define WAKEUP_DELAY 1;' → '#define WAKEUP_DELAY 1'  (semicolon no macro
 *      causava expansão inválida: 'HAL_Delay(1;)').
 *   2. 'adBmsWakeupIc' renomeado para 'bmsWakeupIc' (consistência com header).
 *   3. Adicionadas funções delayUs() e delayMsActive() (usadas por bms_safety.c).
 *   4. Macro GPIO_PORT/CS_PIN alinhados com pinout do main.h do projeto.
 *   5. CORRIGIDO (Mar 2026): apontava para hspi1, que nunca era inicializado
 *      (main.c só chama MX_SPI2_Init() -> hspi2; MX_SPI1_Init() de spi.c
 *      nunca era chamada). Agora usa hspi2.
 *   6. CORRIGIDO (Mar 2026): funções spiXxxBytes agora retornam bool/status
 *      do HAL em vez de descartar o retorno -- permite detectar falha de
 *      comunicação em vez de seguir com dados não confiáveis.
 *
 * Modificado em: Mar 2026
 * Autor: vtaua
 */

#include "../inc/common.h"
#include "../inc/mcu_wrapper.h"
#include "main.h"

/* -----------------------------------------------------------------------
 * Handles HAL (definidos pelo CubeMX em main.c -- MX_SPI2_Init())
 * ----------------------------------------------------------------------- */
static SPI_HandleTypeDef  *hspi  = &hspi2;
static TIM_HandleTypeDef  *htim  = &htim2;

/* -----------------------------------------------------------------------
 * Pino de Chip Select – alinhe com as definições do seu main.h
 * Pinout real do projeto: BMS_CS_GPIO_Port / BMS_CS_Pin (PC0)
 * ----------------------------------------------------------------------- */
#define GPIO_PORT   BMS_CS_GPIO_Port
#define CS_PIN      BMS_CS_Pin

#define WAKEUP_DELAY  1   /* ms – sem ponto-e-vírgula! */

/* -----------------------------------------------------------------------
 * Chip Select
 * ----------------------------------------------------------------------- */
void bmsCsLow(void) {
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_RESET);
}

void bmsCsHigh(void) {
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_SET);
}

/* -----------------------------------------------------------------------
 * Timer de microsegundos (TIM2 deve ser configurado a 1 MHz no CubeMX)
 * ----------------------------------------------------------------------- */
void startTimer(void) {
    HAL_TIM_Base_Start(htim);
}

void stopTimer(void) {
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_SetCounter(htim, 0);
}

uint32_t getTimCount(void) {
    return (uint32_t)__HAL_TIM_GetCounter(htim);
}

/* -----------------------------------------------------------------------
 * Wake-up da cadeia daisy chain
 * Cada IC acorda ao detectar uma borda CS dentro de 4 ms de inatividade.
 * ----------------------------------------------------------------------- */
void bmsWakeupIc(uint8_t total_ic) {
    for (uint8_t ic = 0; ic < total_ic; ic++) {
        bmsCsLow();
        HAL_Delay(WAKEUP_DELAY);
        bmsCsHigh();
        HAL_Delay(WAKEUP_DELAY);
    }
}

/* -----------------------------------------------------------------------
 * Delays
 * ----------------------------------------------------------------------- */
void Delay_ms(uint32_t delay) {
    HAL_Delay(delay);
}

void delayUs(uint32_t us) {
    startTimer();
    while (getTimCount() < us);
    stopTimer();
}

/**
 * @brief Delay em ms mantendo o IsoSPI ativo (pulsa CS a cada 500 µs).
 *        Necessário para evitar que o IC entre em modo de dormência durante
 *        esperas longas (ex.: aguardar conversão AUX).
 */
void delayMsActive(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        bmsCsLow();
        delayUs(500);
        bmsCsHigh();
        delayUs(500);
    }
}

/* -----------------------------------------------------------------------
 * Funções SPI de baixo nível
 * ----------------------------------------------------------------------- */
bool spiWriteBytes(uint16_t size, uint8_t *tx_data) {
    return HAL_SPI_Transmit(hspi, tx_data, size, SPI_TIME_OUT) == HAL_OK;
}

bool spiWriteReadBytes(uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    if (HAL_SPI_Transmit(hspi, tx_data, 4, SPI_TIME_OUT) != HAL_OK) return false;
    return HAL_SPI_Receive(hspi, rx_data, size, SPI_TIME_OUT) == HAL_OK;
}

bool spiReadBytes(uint16_t size, uint8_t *rx_data) {
    return HAL_SPI_Receive(hspi, rx_data, size, SPI_TIME_OUT) == HAL_OK;
}
