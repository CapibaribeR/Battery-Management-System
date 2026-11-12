/*
 * spi.c
 *
 * Descrição: Inicialização do SPI2 para comunicação com o ADBMS6830B.
 *
 * CORREÇÕES em relação ao arquivo anterior:
 *   1. 'hspi1.Instance = hspi1;' → 'hspi1.Instance = SPI2;'
 *      (deve ser o periférico SPI2, não o handle).
 *   2. 'hsp1.Init.NSSPMode' → 'hspi1.Init.NSSPMode' (typo: hsp1 vs hspi1).
 *   3. 'CRCPolynimial' → 'CRCPolynomial' (typo ortográfico).
 *   4. 'GPIO_AFS_SPI1' → 'GPIO_AF5_SPI1' (AF correto para SPI1 no STM32G4).
 *   5. 'HAL_SPI_MSPInit' → 'HAL_SPI_MspInit' (case-sensitive, nome HAL correto).
 *   6. 'SPI_HandleTypedef' → 'SPI_HandleTypeDef' (typo).
 *   7. 'HAL_SPI_MspDeInIT' → 'HAL_SPI_MspDeInit' (case-sensitive).
 *   8. Adicionado pin CS na inicialização GPIO (output push-pull).
 *
 * NOTA: Os pinos BMS_SCK/MISO/MOSI devem estar definidos em main.h.
 *       O repositório Manchester usa: PA5=SCK, PA6=MISO, PA7=MOSI, PB6=CS.
 *
 * Modificado em: Mar 2026
 * Autor: vtaua
 */

#include "spi.h"
#include "main.h"

SPI_HandleTypeDef hspi1;

/**
 * @brief Inicialização do SPI1 – Master, 8-bit, CPOL=0, CPHA=0 (Modo 0).
 *        Prescaler 64 → ~2.6 MHz com PCLK = 170 MHz (ajuste se necessário).
 */
void MX_SPI1_Init(void) {
    hspi1.Instance               = SPI2;               /* FIX: era 'hspi1' */
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;   /* CPOL = 0         */
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;    /* CPHA = 0 (Modo 0)*/
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 7;                  /* FIX: era CRCPolynimial */
    hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE; /* FIX: era hsp1.Init */

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief HAL MSP callback – configura pinos GPIO para SPI2.
 *        Chamado internamente por HAL_SPI_Init().
 */
/*void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle) {   // FIX: era MSPInit + Typedef
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (spiHandle->Instance == SPI2) {
        // Clock do SPI2
        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        //SCK=PA5, MISO=PA6, MOSI=PA7 – Alternate Function 5
        GPIO_InitStruct.Pin       = BMS_SCK_Pin | BMS_MISO_Pin | BMS_MOSI_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;     // FIX: era GPIO_AFS_SPI2
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // CS=PB6 – GPIO output (gerenciado por bmsCsLow/High em mcu_wrapper.c)
        GPIO_InitStruct.Pin   = BMS_CS_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(BMS_CS_GPIO_Port, &GPIO_InitStruct);

        // Garante CS em nível alto no início
        HAL_GPIO_WritePin(BMS_CS_GPIO_Port, BMS_CS_Pin, GPIO_PIN_SET);
    }
}*/

/**
 * @brief HAL MSP DeInit – desabilita SPI2 e libera GPIOs.
 */
/*void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle) { // FIX: era MspDeInIT
    if (spiHandle->Instance == SPI2) {
        __HAL_RCC_SPI2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, BMS_SCK_Pin | BMS_MISO_Pin | BMS_MOSI_Pin);
        HAL_GPIO_DeInit(BMS_CS_GPIO_Port, BMS_CS_Pin);
    }
}*/
