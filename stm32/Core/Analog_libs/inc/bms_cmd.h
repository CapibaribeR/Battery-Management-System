/*
 * bms_cmd.h
 *
 * Descrição: Tabela de comandos SPI do ADBMS6830B.
 *
 * CORREÇÕES/ADIÇÕES em relação ao arquivo anterior:
 *   1. Removidas declarações de structs ADCV/ADSV/ADAX (agora em bms_data.h).
 *   2. Renomeados WRPWM1/WRPWM2 para WRPWMA/WRPWMB (padrão Manchester repo
 *      e documentação Analog Devices).
 *   3. Adicionado guarda de inclusão múltipla correto.
 *   4. Comando DIAGN REMOVIDO (Mar 2026): o ADBMS6830B (Rev.0) não possui
 *      comando DIAGN -- era um comando de família LTC68xx mais antiga,
 *      ausente da Table 50 deste datasheet. O valor {0x07,0x15} usado
 *      antes colidia byte-a-byte com CLOVUV, fazendo os dois comandos
 *      mapearem para a mesma sequência SPI.
 *   5. PLAUT/PLAUX1 (duplicatas/nomes confusos do mesmo valor 0x07,0x1E)
 *      consolidados em um único PLAUX, conforme nome do comando no
 *      datasheet (Table 50: "Poll AUX ADC").
 *
 * Referência: ADBMS6830B Datasheet Rev.0, Table 50 (Command Summary)
 *
 * Modificado em: Mar 2026
 * Autor: vtaua
 */

#ifndef ANALOG_LIBS_INC_BMS_CMD_H_
#define ANALOG_LIBS_INC_BMS_CMD_H_

#include "common.h"

/* -----------------------------------------------------------------------
 * Comandos de Configuração
 * ----------------------------------------------------------------------- */
extern uint8_t WRCFGA[2];    /* Write Configuration Register Group A       */
extern uint8_t RDCFGA[2];    /* Read  Configuration Register Group A       */
extern uint8_t WRCFGB[2];    /* Write Configuration Register Group B       */
extern uint8_t RDCFGB[2];    /* Read  Configuration Register Group B       */

/* -----------------------------------------------------------------------
 * Leitura de tensão de célula (C-ADC)
 * ----------------------------------------------------------------------- */
extern uint8_t RDCVA[2];     /* Read Cell Voltage Group A  (células 1-3)   */
extern uint8_t RDCVB[2];     /* Read Cell Voltage Group B  (células 4-6)   */
extern uint8_t RDCVC[2];     /* Read Cell Voltage Group C  (células 7-9)   */
extern uint8_t RDCVD[2];     /* Read Cell Voltage Group D  (células 10-12) */
extern uint8_t RDCVE[2];     /* Read Cell Voltage Group E  (células 13-15) */
extern uint8_t RDCVF[2];     /* Read Cell Voltage Group F  (célula 16)     */
extern uint8_t RDCVALL[2];   /* Read All Cell Voltage Registers            */

/* -----------------------------------------------------------------------
 * Leitura de tensão média (A-ADC)
 * ----------------------------------------------------------------------- */
extern uint8_t RDACA[2];
extern uint8_t RDACB[2];
extern uint8_t RDACC[2];
extern uint8_t RDACD[2];
extern uint8_t RDACE[2];
extern uint8_t RDACF[2];
extern uint8_t RDACALL[2];

/* -----------------------------------------------------------------------
 * Leitura de tensão S-ADC (redundante)
 * ----------------------------------------------------------------------- */
extern uint8_t RDSVA[2];
extern uint8_t RDSVB[2];
extern uint8_t RDSVC[2];
extern uint8_t RDSVD[2];
extern uint8_t RDSVE[2];
extern uint8_t RDSVF[2];
extern uint8_t RDSALL[2];

extern uint8_t RDCSALL[2];
extern uint8_t RDACSALL[2];

/* -----------------------------------------------------------------------
 * Leitura de tensão filtrada (F-ADC)
 * ----------------------------------------------------------------------- */
extern uint8_t RDFCA[2];
extern uint8_t RDFCB[2];
extern uint8_t RDFCC[2];
extern uint8_t RDFCD[2];
extern uint8_t RDFCE[2];
extern uint8_t RDFCF[2];
extern uint8_t RDFCALL[2];

/* -----------------------------------------------------------------------
 * Leitura de AUX / GPIO
 * ----------------------------------------------------------------------- */
extern uint8_t RDAUXA[2];
extern uint8_t RDAUXB[2];
extern uint8_t RDAUXC[2];
extern uint8_t RDAUXD[2];
extern uint8_t RDRAXA[2];
extern uint8_t RDRAXB[2];
extern uint8_t RDRAXC[2];
extern uint8_t RDRAXD[2];
extern uint8_t RDASALL[2];

/* -----------------------------------------------------------------------
 * Leitura de registradores de Status
 * ----------------------------------------------------------------------- */
extern uint8_t RDSTATA[2];
extern uint8_t RDSTATB[2];
extern uint8_t RDSTATC[2];
extern uint8_t RDSTATCERR[2];   /* ERR variant                            */
extern uint8_t RDSTATD[2];
extern uint8_t RDSTATE[2];

/* -----------------------------------------------------------------------
 * Registradores PWM (balanceamento)
 * WRPWMA: células 1-12 | WRPWMB: células 13-16
 * ----------------------------------------------------------------------- */
extern uint8_t WRPWMA[2];
extern uint8_t RDPWMA[2];
extern uint8_t WRPWMB[2];
extern uint8_t RDPWMB[2];

/* -----------------------------------------------------------------------
 * Comandos de limpeza
 * ----------------------------------------------------------------------- */
extern uint8_t CLRCELL[2];
extern uint8_t CLRAUX[2];
extern uint8_t CLRSPIN[2];
extern uint8_t CLRFLAG[2];
extern uint8_t CLRFC[2];
extern uint8_t CLOVUV[2];

/* -----------------------------------------------------------------------
 * Comandos de Poll ADC
 * ----------------------------------------------------------------------- */
extern uint8_t PLADC[2];
extern uint8_t PLCADC[2];
extern uint8_t PLSADC[2];
extern uint8_t PLAUX[2];
extern uint8_t PLAUX2[2];

/* -----------------------------------------------------------------------
 * Comunicação I2C/SPI master via GPIO
 * ----------------------------------------------------------------------- */
extern uint8_t WRCOMM[2];
extern uint8_t RDCOMM[2];
extern uint8_t STCOMM[13];

/* -----------------------------------------------------------------------
 * Controle / Mute
 * ----------------------------------------------------------------------- */
extern uint8_t MUTE[2];
extern uint8_t UNMUTE[2];
extern uint8_t RSTCC[2];
extern uint8_t SNAP[2];
extern uint8_t UNSNAP[2];
extern uint8_t SRST[2];
extern uint8_t RDSID[2];

#endif /* ANALOG_LIBS_INC_BMS_CMD_H_ */
