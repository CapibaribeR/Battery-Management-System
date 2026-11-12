/*
 * bms_generic.h
 *
 * Descrição: Funções genéricas de SPI/PEC para comunicação com o ADBMS6830B.
 *
 * CORREÇÕES em relação ao arquivo anterior:
 *   1. Removida referência a 'cell_asic' e 'TYPE'/'GRP' (estruturas ADI não
 *      utilizadas neste projeto – substituídas pela struct adbms68_t em comm.c).
 *   2. bmsReadData / bmsWriteData removidos da interface pública (encapsulados
 *      dentro de comm.c).
 *
 * Modificado em: Mar 2026
 * Autor: vtaua
 */

#ifndef ANALOG_LIBS_INC_BMS_GENERIC_H_
#define ANALOG_LIBS_INC_BMS_GENERIC_H_

#include "common.h"
#include "bms_cmd.h"
#include "mcu_wrapper.h"
#include "bms_data.h"

/* -----------------------------------------------------------------------
 * CRC / PEC
 * ----------------------------------------------------------------------- */
uint16_t Pec15_Calc(uint8_t len, uint8_t *data);
uint16_t pec10_calc(bool rx_cmd, int len, uint8_t *data);

/* -----------------------------------------------------------------------
 * Funções de transporte SPI
 * ----------------------------------------------------------------------- */

/**
 * @brief Envia um comando de 2 bytes + PEC15 (total 4 bytes).
 */
void spiSendCmd(uint8_t tx_cmd[2]);

/**
 * @brief Envia comando CLRFLAG com 6 bytes de dados de registrador.
 */
void spiSendCLRFLAG(uint8_t tx_cmd[2], uint8_t st_reg[6]);

/**
 * @brief Lê dados de todos os ICs na cadeia após enviar um comando.
 * @param tIC          Número de ICs
 * @param tx_cmd       Comando (2 bytes)
 * @param rx_data      Buffer de saída [tIC × regData_size]
 * @param pec_error    Array de flags de erro PEC [tIC]
 * @param cmd_cntr     Array de contadores de comando [tIC]
 * @param regData_size Tamanho do pacote por IC (6 dados + 2 PEC = 8)
 */
void spiReadData(uint8_t tIC, uint8_t tx_cmd[2],
                 uint8_t *rx_data, uint8_t *pec_error,
                 uint8_t *cmd_cntr, uint8_t regData_size);

/**
 * @brief Escreve dados em todos os ICs na cadeia.
 * @param tIC    Número de ICs
 * @param tx_cmd Comando (2 bytes)
 * @param data   Buffer de dados [tIC × TX_DATA]
 */
void spiWriteData(uint8_t tIC, uint8_t tx_cmd[2], uint8_t *data);

/**
 * @brief Envia comando de Poll ADC e aguarda conversão.
 * @return Contagem do timer (microsegundos) até conclusão, ou UINT32_MAX
 *         se estourou BMS_POLL_TIMEOUT_MS sem SDO ir para nível alto
 *         (indica IC travado, desconectado, ou falha de comunicação).
 */
uint32_t bmsPollAdc(uint8_t tx_cmd[2]);

#endif /* ANALOG_LIBS_INC_BMS_GENERIC_H_ */
