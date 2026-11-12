/*
 * common.h
 *
 * Descrição: Inclusões padrão, macros globais e tipo de status BMS.
 *
 *
 * Modificado em: Mar 2026
 * Autor: vtaua
 */

#ifndef ANALOG_LIBS_INC_COMMON_H_
#define ANALOG_LIBS_INC_COMMON_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

/* -----------------------------------------------------------------------
 * Configuração da cadeia (daisy chain)
 * TOTAL_IC  : número total de ICs ADBMS6830 na cadeia
 * TOTAL_CELL: células por IC (fixo em 16 para ADBMS6830)
 * ----------------------------------------------------------------------- */
#ifndef TOTAL_IC
#define TOTAL_IC        1       /* Número de ADBMS6830B na cadeia           */
#endif

#ifndef TOTAL_CELL
#define TOTAL_CELL      16      /* Células por IC – NÃO alterar             */
#endif

/* -----------------------------------------------------------------------
 * Macros de manipulação de bits
 * ----------------------------------------------------------------------- */
#define BIT_SET(reg, n)    ((reg) |=  (UINT32_C(1) << (n)))
#define BIT_CLEAR(reg, n)  ((reg) &= ~(UINT32_C(1) << (n)))
#define BIT_CHECK(reg, n)  ((reg) &   (UINT32_C(1) << (n)))
#define BIT_FLIP(reg, n)   ((reg) ^=  (UINT32_C(1) << (n)))

/* -----------------------------------------------------------------------
 * Tipo de retorno de status BMS
 * ----------------------------------------------------------------------- */
typedef enum {
	BMS_OK = 0x00, /* Sem erros                             */
	BMS_ERR_COMMS = 0x01, /* Erro de comunicação SPI / PEC         */
	BMS_ERR_VOLTAGE = 0x02, /* Falha de tensão (OV ou UV)            */
	BMS_ERR_TEMP = 0x04, /* Falha de temperatura                  */
	BMS_ERR_CURRENT = 0x08, /* Falha de corrente                     */
	BMS_ERR_OPEN_WIRE = 0x10, /* Fio aberto detectado                  */
	BMS_ERR_INTERNAL = 0x20, /* Falha interna do ADBMS6830B (STATC)   */
} BMS_StatusTypeDef;

/* Alias para compatibilidade com código legado do projeto */
typedef BMS_StatusTypeDef BMS_status_t;

#endif /* ANALOG_LIBS_INC_COMMON_H_ */
