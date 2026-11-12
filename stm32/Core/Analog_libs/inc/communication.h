/*
 * communication.h
 *
 * Descrição: Interface de alto nível para o BMS (análogo a bms_libWrapper.h
 *            do repositório Manchester Stinger).
 *
 * CORREÇÕES em relação ao arquivo anterior:
 *   1. 'unit32_t' → 'uint32_t' (typo).
 *   2. 'CanTxMsg' movido para cá (dependência de bms_can.h removida do módulo base).
 *   3. Funções completas adicionadas conforme Manchester repo.
 *   4. enableBalancing (variável) renomeada para evitar conflito com função de
 *      mesmo nome – função renomeada para BMS_EnableBalancing.
 *
 * Modificado em: Mar 2026
 * Autor: vtaua
 */

#ifndef ANALOG_LIBS_INC_COMMUNICATION_H_
#define ANALOG_LIBS_INC_COMMUNICATION_H_

#pragma once

#include "bms_cmd.h"
#include "bms_data.h"
#include "mcu_wrapper.h"
#include "bms_generic.h"
#include "bms_balancing.h"
#include "bms_safety.h"
#include "common.h"
#include "main.h"
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * Tipos de tensão lida (C-ADC, S-ADC, filtrado, médio, temperatura)
 * ----------------------------------------------------------------------- */
typedef enum {
    VOLTAGE_C,
    VOLTAGE_C_AVG,
    VOLTAGE_C_FIL,
    VOLTAGE_S,
    VOLTAGE_TEMP,
    TOTAL_VOLTAGE_TYPES
} VoltageTypes;

/* -----------------------------------------------------------------------
 * Tipos de registrador para leitura/escrita
 * ----------------------------------------------------------------------- */
typedef enum {
    REG_CONFIG_A,
    REG_CONFIG_B,
    REG_PWM_A,
    REG_PWM_B,
    REG_SID,
    TOTAL_REG_TYPES
} RegisterTypes;

/* -----------------------------------------------------------------------
 * Container de mensagem CAN (definido aqui para não depender de bms_can.h
 * nos módulos base; bms_can.h pode redefini-lo via #include próprio)
 * ----------------------------------------------------------------------- */
#ifndef BMS_CAN_MSG_DEFINED
#define BMS_CAN_MSG_DEFINED
#include "stm32g4xx_hal.h"
typedef struct {
    uint8_t               data[8];
    FDCAN_TxHeaderTypeDef header;
} CanTxMsg;
#endif

/* -----------------------------------------------------------------------
 * API principal
 * ----------------------------------------------------------------------- */

/** @brief Inicializa CFGA, CFGB (VOV, VUV, DCTO, REFON), inicia ADCV contínuo. */
BMS_StatusTypeDef bms_init(void);

/** @brief Configura GPO4/GPO5 para controle do mux de temperatura. */
void bms_setGpo45(uint8_t twoBitIndex);

/** @brief Inicia medição contínua de célula (C-ADC). */
void bms_startAdcvCont(bool enableRedundant);

/** @brief Lê tensões de célula de todos os ICs e popula volt_cell. */
BMS_StatusTypeDef readCellVoltage(VoltageTypes voltTypes);

/** @brief Ponteiro para a matriz [IC][célula] de tensões do tipo indicado. */
float (*bms_getVoltPtr(VoltageTypes t))[TOTAL_CELL];

/** @brief Lê / imprime um registrador para depuração. */
BMS_StatusTypeDef readRegister(RegisterTypes regTypes);

/** @brief Executa medição AUX (temperatura via GPIO/NTC). */
BMS_StatusTypeDef auxMeasurement(void);

/** @brief Inicia descarga PWM para células acima de threshold. */
void startDischarge(float threshold);

/** @brief Para toda descarga e zera DCTO. */
void stopDischarge(void);

/** @brief Reset suave do IC (comando SRST + re-init). */
void bms_reset(void);

/** @brief Lê tensão de pack (AD29 ou soma das células). */
BMS_StatusTypeDef readV(void);

/** @brief Lê corrente do shunt (placeholder – AD29 não presente no projeto). */
BMS_StatusTypeDef readCurrent(void);

/** @brief Dispara S-ADC para medição durante balanceamento (para PWM brevemente). */
BMS_StatusTypeDef balancingMeasurementVoltage(void);

/** @brief Calcula limiar e inicia balanceamento. */
void startBalancing(float delta_v);

/** @brief Prepara buffer CAN com dados de célula, temperatura, status. */
void getCanData(CanTxMsg **buff, uint32_t *len);

/** @brief Loop principal do BMS (chamado a cada iteração do while(1)). */
BMS_StatusTypeDef programLoop(void);

/* -----------------------------------------------------------------------
 * Verificações de falha
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef checkVoltage(void);
BMS_StatusTypeDef checkTemp(void);
BMS_StatusTypeDef checkCur(void);
BMS_StatusTypeDef checkCommsFault(void);

/* -----------------------------------------------------------------------
 * Controle de carga / balanceamento
 * ----------------------------------------------------------------------- */
void BMS_EnableCharging(bool enabled);
void BMS_ChargingButtonLogic(void);
bool BMS_IsCharging(void);

void BMS_EnableBalancing(bool enabled);
void BMS_ToggleBalancing(void);

/* -----------------------------------------------------------------------
 * Sinalização de falha
 * ----------------------------------------------------------------------- */
void BMS_SetCommsFault(bool state);
void BMS_WriteFaultSignal(bool state);

/* -----------------------------------------------------------------------
 * Acesso aos dados de telemetria (para CAN / UART)
 * ----------------------------------------------------------------------- */
void BMS_GetCanData(CanTxMsg **buff, uint32_t *len);

#endif /* ANALOG_LIBS_INC_COMMUNICATION_H_ */
