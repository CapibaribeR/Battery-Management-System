/*
 * bms_data.h
 *
 * Descrição: Estruturas de dados dos registradores do ADBMS6830B.
 *
 * CORREÇÕES em relação ao arquivo anterior:
 *   1. Typedef duplicado 'pwma_t' → renomeado segundo typedef para 'ad68_pwmb_t'
 *      e primeiro para 'ad68_pwma_t'.
 *   2. Membros de struct sem ';' em sta_, stb_, stc_ → corrigidos.
 *   3. Tipedefs anônimos de cfga/cfgb → adicionados nomes: ad68_cfa_t / ad68_cfb_t.
 *   4. Macros de conversão de tensão / temperatura adicionadas.
 *   5. Struct 'adcv_t' reescrita para alinhar com datasheet (ADCV_t).
 *   6. Defines de limites de segurança (FSAE 2026 + datasheet).
 *
 * Referências:
 *   - ADBMS6830B Datasheet Rev.0, Table 56 (CFGB), Table 18 (ADCV)
 *   - FSAE Rules 2026 EV.5.7, T.9.1
 *
 * Modificado em: Mar 2026
 * Autor: vtaua
 */

#ifndef ANALOG_LIBS_INC_BMS_DATA_H_
#define ANALOG_LIBS_INC_BMS_DATA_H_

#include "common.h"

/* -----------------------------------------------------------------------
 * Tamanhos dos registradores
 * ----------------------------------------------------------------------- */
#define CELL            16
#define AUX             12
#define RAUX            10
#define PWMA_REG        12
#define PWMB_REG        4
#define COMM_REG        3
#define RSID            6
#define TX_DATA         6
#define RX_DATA         8
#define RDCVALL_SIZE    34
#define RDSALL_SIZE     34
#define RDACALL_SIZE    34
#define RDFCALL_SIZE    34
#define RDCSALL_SIZE    66
#define RDASALL_SIZE    70
#define RDACSALL_SIZE   66

/* -----------------------------------------------------------------------
 * Macros de conversão – Datasheet ADBMS6830B (Rev.0)
 *   V[V] = (raw_code + 10000) × 150 µV    (para raw int16_t)
 * ----------------------------------------------------------------------- */
#define CELL_VOLTAGE_V(raw)    (((int16_t)(raw)) * 0.00015f + 1.5f)
#define AUX_VOLTAGE_V(raw)     (((int16_t)(raw)) * 0.00015f + 1.5f)
#define ITMP_TO_CELSIUS(raw)   ((AUX_VOLTAGE_V(raw) / 0.0075f) - 273.15f)

/* -----------------------------------------------------------------------
 * Limites de segurança (FSAE 2026 + Li-Ion típico)
 * Ajuste conforme especificação das células do acumulador.
 * ----------------------------------------------------------------------- */
#define BMS_OV_THRESHOLD_V      4.20f   /* Sobretensão (datasheet/célula)    */
#define BMS_UV_THRESHOLD_V      2.50f   /* Subtensão   (datasheet/célula)    */
/* Temperatura de CÉLULA é monitorada pelo TMS dedicado, fora deste IC.
 * Os limites abaixo são para o DIE do ADBMS6830B (auto-diagnóstico do
 * próprio chip), não para a bateria. Datasheet: faixa operacional
 * TJ = -40°C a +125°C; thermal shutdown trip ~150°C (seção
 * "Thermal Shutdown"). Margem de alerta antes do trip. */
#define BMS_DIE_TEMP_MAX_C      125.0f  /* Alerta de temp. de die (°C)      */
#define BMS_DIE_TEMP_MIN_C     (-40.0f) /* Faixa operacional mínima (°C)    */
#define BMS_BALANCE_DELTA_V     0.010f  /* Delta para início do balanceamento */
#define BMS_OW_DELTA_V          0.200f  /* (obsoleto, não usado)             */
/* Fio aberto: queda máx. tolerada (fração da tensão normal) com a chave de
 * OW ligada. Canal íntegro cai ~1/6 (C-ADC) ou ~1/10 (S-ADC); aberto cai ~100%.
 * Ajuste conforme o resistor de filtro do hardware. */
#define BMS_OW_DROP_MAX_FRAC    0.5f

/* -----------------------------------------------------------------------
 * Registrador de Configuração A – ADBMS6830B Table 55
 * ----------------------------------------------------------------------- */
typedef struct
{
    uint8_t  cth      : 3;   /* Comparator Threshold                       */
    uint8_t  res0     : 4;   /* Reservado                                  */
    uint8_t  refon    : 1;   /* Referência ligada: 1=ligada entre conversões*/

    uint8_t  flag_d   : 8;   /* Flag discharge bits                        */

    uint8_t  res1     : 3;
    uint8_t  owa      : 3;   /* Open-wire action                           */
    uint8_t  owrng    : 1;   /* Open-wire range                            */
    uint8_t  soakon   : 1;   /* Soak on                                    */

    uint8_t  gpo1_8   : 8;   /* GPIO Pull-down 1-8                         */

    uint8_t  gpo9_10  : 2;   /* GPIO Pull-down 9-10                        */
    uint8_t  res2     : 6;

    uint8_t  fc       : 3;   /* Filter corner freq                         */
    uint8_t  comm_bk  : 1;   /* Communication break                        */
    uint8_t  mute_st  : 1;   /* Mute state                                 */
    uint8_t  snap     : 1;   /* Snapshot                                   */
    uint8_t  res3     : 2;
} ad68_cfa_t;

/* -----------------------------------------------------------------------
 * Registrador de Configuração B – ADBMS6830B Table 56
 * Layout dos 6 bytes:
 *   [0] VUV[7:0]
 *   [1] VUV[11:8] | VOV[3:0]
 *   [2] VOV[11:4]
 *   [3] DCTO[5:0] | DTRNG | DTMEN
 *   [4] DCC[8:1]
 *   [5] DCC[16:9]
 * ----------------------------------------------------------------------- */
typedef struct
{
    uint8_t  vuv0_7   : 8;   /* Undervoltage comparação bits [7:0]         */

    uint8_t  vuv8_11  : 4;   /* Undervoltage bits [11:8]                   */
    uint8_t  vov0_3   : 4;   /* Overvoltage bits  [3:0]                    */

    uint8_t  vov4_11  : 8;   /* Overvoltage bits  [11:4]                   */

    uint8_t  dcto     : 6;   /* Discharge timer out (1 LSB = 1 min; DTRNG=0)*/
    uint8_t  dtrng    : 1;   /* Discharge timer range: 0=min, 1=sec        */
    uint8_t  dtmen    : 1;   /* Discharge timer monitor enable             */

    uint8_t  dcc1_8   : 8;   /* Discharge cells 1–8 (DCC bits)             */
    uint8_t  dcc9_16  : 8;   /* Discharge cells 9–16 (DCC bits)            */
} ad68_cfb_t;

/* -----------------------------------------------------------------------
 * Registradores PWM – ADBMS6830B Table 95/96
 * WRPWM1: células 1-12 (4 bits cada, 0x0=0%, 0xF=100%)
 * WRPWM2: células 13-16 (restante do 2º registrador reservado)
 * ----------------------------------------------------------------------- */
typedef struct
{
    uint8_t  pwm1  : 4;
    uint8_t  pwm2  : 4;
    uint8_t  pwm3  : 4;
    uint8_t  pwm4  : 4;
    uint8_t  pwm5  : 4;
    uint8_t  pwm6  : 4;
    uint8_t  pwm7  : 4;
    uint8_t  pwm8  : 4;
    uint8_t  pwm9  : 4;
    uint8_t  pwm10 : 4;
    uint8_t  pwm11 : 4;
    uint8_t  pwm12 : 4;
} ad68_pwma_t;

typedef struct
{
    uint8_t  pwm13 : 4;
    uint8_t  pwm14 : 4;
    uint8_t  pwm15 : 4;
    uint8_t  pwm16 : 4;
    uint8_t  rsv[4];         /* 4 bytes reservados para completar 6 bytes  */
} ad68_pwmb_t;

/* -----------------------------------------------------------------------
 * Registrador de Status A (RDSTATA)
 * ----------------------------------------------------------------------- */
typedef struct {
    uint16_t vref2;          /* VREF2 ADC code                             */
    uint16_t itmp;           /* Die temperature ADC code                   */
    uint16_t vref3;          /* VREF3 ADC code                             */
} ad68_sta_t;

/* -----------------------------------------------------------------------
 * Registrador de Status B (RDSTATB)
 * ----------------------------------------------------------------------- */
typedef struct {
    uint16_t vd;             /* VD supply code                             */
    uint16_t va;             /* VA supply code                             */
    uint16_t vr4k;           /* 4kΩ reference code                         */
} ad68_stb_t;

/* -----------------------------------------------------------------------
 * Registrador de Status C (RDSTATC) – OV/UV flags
 * ----------------------------------------------------------------------- */
typedef struct {
    uint16_t cs_flt;         /* Cell summary fault flags (OV/UV per cell)  */
    uint8_t  flags[4];       /* Fault flags                                */
} ad68_stc_t;

/* -----------------------------------------------------------------------
 * Comando ADCV – ADBMS6830B Table 50
 * Byte 0 (enviado como MSB primeiro):
 *   [7]   RD    – redundant measurement
 *   [6:5] RES   = 0b01 (fixo)
 *   [4:0] X     – 5-bit opcode baixo (0b00000 para 7kHz default)
 * Byte 1:
 *   [7:6] OW    – open-wire mode (00=off, 01=pup, 10=pdown)
 *   [5]   RSTF  – reset filter
 *   [4]   RES0  = 0
 *   [3]   DCP   – discharge permitted
 *   [2:1] RES1  = 0b11 (fixo)
 *   [0]   CONT  – continuous measurement
 * ----------------------------------------------------------------------- */
typedef struct
{
    uint8_t  rd    : 1;
    uint8_t  res2  : 2;   /* = 0b01 fixo                                   */
    uint8_t  x     : 5;   /* opcode 5 LSBs                                 */

    uint8_t  ow    : 2;
    uint8_t  rstf  : 1;
    uint8_t  res0  : 1;
    uint8_t  dcp   : 1;
    uint8_t  res1  : 2;   /* = 0b11 fixo                                   */
    uint8_t  cont  : 1;
} adcv_t;

extern const adcv_t ADCV_default;
extern adcv_t ADCV;

/* -----------------------------------------------------------------------
 * Comando ADSV – medição redundante (S-ADC)
 * ----------------------------------------------------------------------- */
typedef struct
{
    uint8_t  res2  : 3;
    uint8_t  x     : 5;

    uint8_t  ow    : 2;
    uint8_t  res0  : 2;
    uint8_t  dcp   : 1;
    uint8_t  res1  : 2;
    uint8_t  cont  : 1;
} adsv_t;

extern const adsv_t ADSV_default;
extern adsv_t ADSV;

/* -----------------------------------------------------------------------
 * Comando ADAX – medição auxiliar (GPIO/temperatura)
 * ----------------------------------------------------------------------- */
typedef struct
{
    uint8_t  ow    : 1;
    uint8_t  res0  : 2;
    uint8_t  x     : 5;

    uint8_t  ch    : 4;
    uint8_t  res1  : 2;
    uint8_t  ch4   : 1;
    uint8_t  pup   : 1;
} adax_t;

extern const adax_t ADAX_default;
extern adax_t ADAX;

typedef struct
{
    uint8_t  ch4   : 4;
    uint8_t  res   : 7;  /* na verdade 7 bits de reservado */
    uint8_t  x     : 5;
} adax2_t;

extern const adax2_t ADAX2_default;
extern adax2_t ADAX2;

#endif /* ANALOG_LIBS_INC_BMS_DATA_H_ */
