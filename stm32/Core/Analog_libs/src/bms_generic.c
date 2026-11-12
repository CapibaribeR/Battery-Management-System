/*
 * bms_generic.c
 *
 * Descrição: Implementação de CRC/PEC, transporte SPI e comandos ADCV/ADAX.
 *
 * CORREÇÕES em relação ao arquivo anterior:
 *   1. 'adBmsCsLow/High' → 'bmsCsLow/High' em spiWriteData().
 *   2. Definições das tabelas de comandos movidas para cá (eram extern no .h).
 *   3. Instâncias ADCV/ADSV/ADAX com valores padrão corretos (datasheet Table 13).
 *   4. Adicionada função bmsReadData/bmsWriteData encapsulando a leitura genérica
 *      de registradores (6 dados + 2 PEC por IC).
 *
 * Referência: ADBMS6830B Datasheet Rev.0, Tables 5, 13.
 *
 * Modificado em: jun 2026
 * Autor: vtaua
 */

#include "../inc/common.h"
#include "../inc/mcu_wrapper.h"
#include "main.h"
#include "../inc/bms_cmd.h"
#include "../inc/bms_data.h"
#include "../inc/bms_generic.h"

/* -----------------------------------------------------------------------
 * @brief Tabela de comandos – definições (declaradas extern em bms_cmd.h)
 * ----------------------------------------------------------------------- */
uint8_t WRCFGA[2]     = { 0x00, 0x01 };
uint8_t RDCFGA[2]     = { 0x00, 0x02 };
uint8_t WRCFGB[2]     = { 0x00, 0x24 };
uint8_t RDCFGB[2]     = { 0x00, 0x26 };

uint8_t RDCVA[2]      = { 0x00, 0x04 };
uint8_t RDCVB[2]      = { 0x00, 0x06 };
uint8_t RDCVC[2]      = { 0x00, 0x08 };
uint8_t RDCVD[2]      = { 0x00, 0x0A };
uint8_t RDCVE[2]      = { 0x00, 0x09 };
uint8_t RDCVF[2]      = { 0x00, 0x0B };
uint8_t RDCVALL[2]    = { 0x00, 0x0C };

uint8_t RDACA[2]      = { 0x00, 0x44 };
uint8_t RDACB[2]      = { 0x00, 0x46 };
uint8_t RDACC[2]      = { 0x00, 0x48 };
uint8_t RDACD[2]      = { 0x00, 0x4A };
uint8_t RDACE[2]      = { 0x00, 0x49 };
uint8_t RDACF[2]      = { 0x00, 0x4B };
uint8_t RDACALL[2]    = { 0x00, 0x4C };

uint8_t RDSVA[2]      = { 0x00, 0x03 };
uint8_t RDSVB[2]      = { 0x00, 0x05 };
uint8_t RDSVC[2]      = { 0x00, 0x07 };
uint8_t RDSVD[2]      = { 0x00, 0x0D };
uint8_t RDSVE[2]      = { 0x00, 0x0E };
uint8_t RDSVF[2]      = { 0x00, 0x0F };
uint8_t RDSALL[2]     = { 0x00, 0x10 };
uint8_t RDCSALL[2]    = { 0x00, 0x11 };
uint8_t RDACSALL[2]   = { 0x00, 0x51 };

uint8_t RDFCA[2]      = { 0x00, 0x12 };
uint8_t RDFCB[2]      = { 0x00, 0x13 };
uint8_t RDFCC[2]      = { 0x00, 0x14 };
uint8_t RDFCD[2]      = { 0x00, 0x15 };
uint8_t RDFCE[2]      = { 0x00, 0x16 };
uint8_t RDFCF[2]      = { 0x00, 0x17 };
uint8_t RDFCALL[2]    = { 0x00, 0x18 };

uint8_t RDAUXA[2]     = { 0x00, 0x19 };
uint8_t RDAUXB[2]     = { 0x00, 0x1A };
uint8_t RDAUXC[2]     = { 0x00, 0x1B };
uint8_t RDAUXD[2]     = { 0x00, 0x1F };
uint8_t RDRAXA[2]     = { 0x00, 0x1C };
uint8_t RDRAXB[2]     = { 0x00, 0x1D };
uint8_t RDRAXC[2]     = { 0x00, 0x1E };
uint8_t RDRAXD[2]     = { 0x00, 0x25 };
uint8_t RDASALL[2]    = { 0x00, 0x35 };

uint8_t RDSTATA[2]    = { 0x00, 0x30 };
uint8_t RDSTATB[2]    = { 0x00, 0x31 };
uint8_t RDSTATC[2]    = { 0x00, 0x32 };
uint8_t RDSTATCERR[2] = { 0x00, 0x72 };
uint8_t RDSTATD[2]    = { 0x00, 0x33 };
uint8_t RDSTATE[2]    = { 0x00, 0x34 };

uint8_t WRPWMA[2]     = { 0x00, 0x20 };
uint8_t RDPWMA[2]     = { 0x00, 0x22 };
uint8_t WRPWMB[2]     = { 0x00, 0x21 };
uint8_t RDPWMB[2]     = { 0x00, 0x23 };

uint8_t CLRCELL[2]    = { 0x07, 0x11 };
uint8_t CLRAUX[2]     = { 0x07, 0x12 };
uint8_t CLRSPIN[2]    = { 0x07, 0x16 };
uint8_t CLRFLAG[2]    = { 0x07, 0x17 };
uint8_t CLRFC[2]      = { 0x07, 0x14 };
uint8_t CLOVUV[2]     = { 0x07, 0x15 };

uint8_t PLADC[2]      = { 0x07, 0x18 };
uint8_t PLCADC[2]     = { 0x07, 0x1C };
uint8_t PLSADC[2]     = { 0x07, 0x1D };
uint8_t PLAUX[2]      = { 0x07, 0x1E };   /* nome único, sem duplicata PLAUX1/PLAUT */
uint8_t PLAUX2[2]     = { 0x07, 0x1F };

uint8_t WRCOMM[2]     = { 0x07, 0x21 };
uint8_t RDCOMM[2]     = { 0x07, 0x22 };
uint8_t STCOMM[13]    = { 0x07, 0x23, 0xB9, 0xE4,
                           0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00 };

uint8_t MUTE[2]       = { 0x00, 0x28 };
uint8_t UNMUTE[2]     = { 0x00, 0x29 };
uint8_t RSTCC[2]      = { 0x00, 0x2E };
uint8_t SNAP[2]       = { 0x00, 0x2D };
uint8_t UNSNAP[2]     = { 0x00, 0x2F };
uint8_t SRST[2]       = { 0x00, 0x27 };
uint8_t RDSID[2]      = { 0x00, 0x2C };

/* -----------------------------------------------------------------------
 * @brief Instâncias dos comandos de ADC com valores padrão
 * Valores alinhados com bms_cmdlist.c do repositório Manchester (Tab. 13)
 * ----------------------------------------------------------------------- */
const adcv_t ADCV_default = {
    .rd   = 0,
    .res2 = 0x01,   /* fixo = 0b01 conforme datasheet */
    .x    = 0x00,
    .ow   = 0x00,
    .rstf = 0,
    .res0 = 0,
    .dcp  = 0,
    .res1 = 0x03,   /* fixo = 0b11 conforme datasheet */
    .cont = 0,
};
adcv_t ADCV = { .rd=0, .res2=0x01, .x=0x00,
                .ow=0x00, .rstf=0, .res0=0, .dcp=0, .res1=0x03, .cont=0 };

const adsv_t ADSV_default = {
    .res2 = 0x01,
    .x    = 0x00,
    .ow   = 0x00,
    .res0 = 0x02,
    .dcp  = 0,
    .res1 = 0x03,
    .cont = 0,
};
adsv_t ADSV = { .res2=0x01, .x=0x00, .ow=0x00,
                .res0=0x02, .dcp=0, .res1=0x03, .cont=0 };

const adax_t ADAX_default = {
    .ow   = 0,
    .res0 = 0x02,
    .x    = 0x00,
    .ch   = 0x00,
    .res1 = 0x01,
    .ch4  = 0,
    .pup  = 0,
};
adax_t ADAX = { .ow=0, .res0=0x02, .x=0x00,
                .ch=0x00, .res1=0x01, .ch4=0, .pup=0 };

const adax2_t ADAX2_default = { .ch4=0x00, .res=0x000, .x=0x00 };
adax2_t ADAX2 = { .ch4=0x00, .res=0x000, .x=0x00 };

/* -----------------------------------------------------------------------
 * @brief Tabelas CRC
 * ----------------------------------------------------------------------- */
static const uint16_t Crc15Table[256] = {
    0x0000, 0xc599, 0xceab, 0x0b32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd,
    0xf407, 0x319e, 0x3aac, 0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa,
    0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1, 0xbbf3, 0x7e6a,
    0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d,
    0x5b2e, 0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3,
    0xaf29, 0x6ab0, 0x6182, 0xa41b, 0x77e6, 0xb27f, 0xb94d, 0x7cd4,
    0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd, 0x2544,
    0x02be, 0xc727, 0xcc15, 0x098c, 0xda71, 0x1fe8, 0x14da, 0xd143,
    0xf3c5, 0x365c, 0x3d6e, 0xf8f7, 0x2b0a, 0xee93, 0xe5a1, 0x2038,
    0x07c2, 0xc25b, 0xc969, 0x0cf0, 0xdf0d, 0x1a94, 0x11a6, 0xd43f,
    0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
    0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8,
    0xa8eb, 0x6d72, 0x6640, 0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16,
    0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba, 0x4a88, 0x8f11,
    0x057c, 0xc0e5, 0xcbd7, 0x0e4e, 0xddb3, 0x182a, 0x1318, 0xd681,
    0xf17b, 0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286,
    0xa213, 0x678a, 0x6cb8, 0xa921, 0x7adc, 0xbf45, 0xb477, 0x71ee,
    0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070, 0x85e9,
    0x0f84, 0xca1d, 0xc12f, 0x04b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79,
    0xfb83, 0x3e1a, 0x3528, 0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e,
    0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59, 0x2ac0,
    0x0d3a, 0xc8a3, 0xc391, 0x0608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7,
    0x54aa, 0x9133, 0x9a01, 0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757,
    0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9, 0x7350,
    0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b,
    0xa5d1, 0x6048, 0x6b7a, 0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c,
    0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25, 0x2fbc,
    0x0846, 0xcddf, 0xc6ed, 0x0374, 0xd089, 0x1510, 0x1e22, 0xdbbb,
    0x0af8, 0xcf61, 0xc453, 0x01ca, 0xd237, 0x17ae, 0x1c9c, 0xd905,
    0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b, 0x2d02,
    0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492,
    0x5368, 0x96f1, 0x9dc3, 0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
};

static const uint16_t crc10Table[256] = {
    0x000,0x08f,0x11e,0x191,0x23c,0x2b3,0x322,0x3ad,0x0f7,0x078,0x1e9,0x166,0x2cb,0x244,0x3d5,0x35a,
    0x1ee,0x161,0x0f0,0x07f,0x3d2,0x35d,0x2cc,0x243,0x119,0x196,0x007,0x088,0x325,0x3aa,0x23b,0x2b4,
    0x3dc,0x353,0x2c2,0x24d,0x1e0,0x16f,0x0fe,0x071,0x32b,0x3a4,0x235,0x2ba,0x117,0x198,0x009,0x086,
    0x232,0x2bd,0x32c,0x3a3,0x00e,0x081,0x110,0x19f,0x2c5,0x24a,0x3db,0x354,0x0f9,0x076,0x1e7,0x168,
    0x337,0x3b8,0x229,0x2a6,0x10b,0x184,0x015,0x09a,0x3c0,0x34f,0x2de,0x251,0x1fc,0x173,0x0e2,0x06d,
    0x2d9,0x256,0x3c7,0x348,0x0e5,0x06a,0x1fb,0x174,0x22e,0x2a1,0x330,0x3bf,0x012,0x09d,0x10c,0x183,
    0x0eb,0x064,0x1f5,0x17a,0x2d7,0x258,0x3c9,0x346,0x01c,0x093,0x102,0x18d,0x220,0x2af,0x33e,0x3b1,
    0x105,0x18a,0x01b,0x094,0x339,0x3b6,0x227,0x2a8,0x1f2,0x17d,0x0ec,0x063,0x3ce,0x341,0x2d0,0x25f,
    0x2e1,0x26e,0x3ff,0x370,0x0dd,0x052,0x1c3,0x14c,0x216,0x299,0x308,0x387,0x02a,0x0a5,0x134,0x1bb,
    0x30f,0x380,0x211,0x29e,0x133,0x1bc,0x02d,0x0a2,0x3f8,0x377,0x2e6,0x269,0x1c4,0x14b,0x0da,0x055,
    0x13d,0x1b2,0x023,0x0ac,0x301,0x38e,0x21f,0x290,0x1ca,0x145,0x0d4,0x05b,0x3f6,0x379,0x2e8,0x267,
    0x0d3,0x05c,0x1cd,0x142,0x2ef,0x260,0x3f1,0x37e,0x024,0x0ab,0x13a,0x1b5,0x218,0x297,0x306,0x389,
    0x1d6,0x159,0x0c8,0x047,0x3ea,0x365,0x2f4,0x27b,0x121,0x1ae,0x03f,0x0b0,0x31d,0x392,0x203,0x28c,
    0x038,0x0b7,0x126,0x1a9,0x204,0x28b,0x31a,0x395,0x0cf,0x040,0x1d1,0x15e,0x2f3,0x27c,0x3ed,0x362,
    0x20a,0x285,0x314,0x39b,0x036,0x0b9,0x128,0x1a7,0x2fd,0x272,0x3e3,0x36c,0x0c1,0x04e,0x1df,0x150,
    0x3e4,0x36b,0x2fa,0x275,0x1d8,0x157,0x0c6,0x049,0x313,0x39c,0x20d,0x282,0x12f,0x1a0,0x031,0x0be
};

/* -----------------------------------------------------------------------
 * @brief Implementação PEC15
 * ----------------------------------------------------------------------- */
uint16_t Pec15_Calc(uint8_t len, uint8_t *data) {
    uint16_t remainder = 16;
    for (uint8_t i = 0; i < len; i++) {
        uint16_t addr = (((remainder >> 7) ^ data[i]) & 0xFF);
        remainder = (remainder << 8) ^ Crc15Table[addr];
    }
    return remainder * 2;
}

/* -----------------------------------------------------------------------
 * @brief Implementação PEC10
 * ----------------------------------------------------------------------- */
uint16_t pec10_calc(bool rx_cmd, int len, uint8_t *data) {
    uint16_t remainder = 16;
    uint16_t polynom   = 0x8F;

    for (uint8_t pbyte = 0; pbyte < len; ++pbyte) {
        remainder ^= (uint16_t)(data[pbyte] << 2);
        for (uint8_t bit_ = 8; bit_ > 0; --bit_) {
            if ((remainder & 0x200) > 0) {
                remainder = (uint16_t)(remainder << 1) ^ polynom;
            } else {
                remainder = (uint16_t)(remainder << 1);
            }
        }
    }
    if (rx_cmd) {
        remainder ^= (uint16_t)((data[len] & 0xFC) << 2);
        for (uint8_t bit_ = 6; bit_ > 0; --bit_) {
            if ((remainder & 0x200) > 0) {
                remainder = (uint16_t)(remainder << 1) ^ polynom;
            } else {
                remainder = (uint16_t)(remainder << 1);
            }
        }
    }
    return (uint16_t)(remainder & 0x3FF);
}

/* -----------------------------------------------------------------------
 * @brief Envio de comando (2 bytes + PEC15 = 4 bytes)
 * ----------------------------------------------------------------------- */
void spiSendCmd(uint8_t tx_cmd[2]) {
    uint8_t cmd[4];
    cmd[0] = tx_cmd[0];
    cmd[1] = tx_cmd[1];
    uint16_t pec = Pec15_Calc(2, cmd);
    cmd[2] = (uint8_t)(pec >> 8);
    cmd[3] = (uint8_t)(pec);
    bmsCsLow();
    spiWriteBytes(4, cmd);
    bmsCsHigh();
}

/* -----------------------------------------------------------------------
 * @brief Envio de CLRFLAG com 6 bytes de dados de registrador
 * ----------------------------------------------------------------------- */
void spiSendCLRFLAG(uint8_t tx_cmd[2], uint8_t st_reg[6]) {
    uint8_t cmd[10];
    cmd[0] = tx_cmd[0];
    cmd[1] = tx_cmd[1];
    for (int i = 0; i < 6; i++) cmd[2 + i] = st_reg[i];
    uint16_t pec = Pec15_Calc(8, cmd);
    cmd[8] = (uint8_t)(pec >> 8);
    cmd[9] = (uint8_t)(pec);
    bmsCsLow();
    spiWriteBytes(10, cmd);
    bmsCsHigh();
}

/* -----------------------------------------------------------------------
 * @brief Leitura de dados de todos os ICs
 * ----------------------------------------------------------------------- */
void spiReadData(uint8_t tIC, uint8_t tx_cmd[2],
                 uint8_t *rx_data, uint8_t *pec_error,
                 uint8_t *cmd_cntr, uint8_t regData_size)
{
    uint8_t RX_BUFFER = regData_size * tIC;
    uint8_t *data     = (uint8_t *)calloc(RX_BUFFER, sizeof(uint8_t));
    uint8_t *copy     = (uint8_t *)calloc(regData_size, sizeof(uint8_t));

    if (!data || !copy) { free(data); free(copy); return; }

    uint8_t cmd[4];
    cmd[0] = tx_cmd[0];
    cmd[1] = tx_cmd[1];
    uint16_t pec = Pec15_Calc(2, cmd);
    cmd[2] = (uint8_t)(pec >> 8);
    cmd[3] = (uint8_t)(pec);

    bmsCsLow();
    spiWriteReadBytes(cmd, data, RX_BUFFER);
    bmsCsHigh();

    for (uint8_t ic = 0; ic < tIC; ic++) {
        for (uint8_t b = 0; b < regData_size - 2; b++) {
            rx_data[ic * regData_size + b] = data[ic * regData_size + b];
        }
        cmd_cntr[ic] = data[ic * regData_size + regData_size - 2] >> 2;
        uint16_t rx_pec = (uint16_t)(
            ((data[ic * regData_size + regData_size - 2] & 0x03) << 8) |
              data[ic * regData_size + regData_size - 1]);
        memcpy(copy, &data[ic * regData_size], regData_size);
        uint16_t calc_pec = pec10_calc(true, regData_size - 2, copy);
        pec_error[ic] = (rx_pec != calc_pec) ? 1 : 0;
    }

    free(data);
    free(copy);
}

/* -----------------------------------------------------------------------
 * @brief Escrita de dados em todos os ICs (daisy-chain, último IC recebe primeiro)
 * ----------------------------------------------------------------------- */
void spiWriteData(uint8_t tIC, uint8_t tx_cmd[2], uint8_t *data) {
    uint8_t CMD_LEN = 4 + (RX_DATA * tIC);
    uint8_t *cmd = (uint8_t *)calloc(CMD_LEN, sizeof(uint8_t));
    if (!cmd) return;

    cmd[0] = tx_cmd[0];
    cmd[1] = tx_cmd[1];
    uint16_t cpec = Pec15_Calc(2, cmd);
    cmd[2] = (uint8_t)(cpec >> 8);
    cmd[3] = (uint8_t)(cpec);

    uint8_t idx = 4;
    for (uint8_t ic = tIC; ic > 0; ic--) {
        uint8_t src = (ic - 1) * TX_DATA;
        for (uint8_t b = 0; b < TX_DATA; b++) cmd[idx++] = data[src + b];
        uint8_t copyArr[TX_DATA];
        memcpy(copyArr, &data[src], TX_DATA);
        /* Write PEC cobre só os 6 bytes de dados (sem command counter) */
        uint16_t dpec = pec10_calc(false, TX_DATA, copyArr);
        cmd[idx++] = (uint8_t)(dpec >> 8);
        cmd[idx++] = (uint8_t)(dpec);
    }

    bmsCsLow();
    spiWriteBytes(CMD_LEN, cmd);
    bmsCsHigh();
    free(cmd);
}

/* -----------------------------------------------------------------------
 * @brief Poll ADC – aguarda EOC (SDO = 0xFF)
 * ----------------------------------------------------------------------- */
uint32_t bmsPollAdc(uint8_t tx_cmd[2]) {
    uint8_t cmd[4];
    cmd[0] = tx_cmd[0];
    cmd[1] = tx_cmd[1];
    uint16_t pec = Pec15_Calc(2, cmd);
    cmd[2] = (uint8_t)(pec >> 8);
    cmd[3] = (uint8_t)(pec);

    startTimer();
    bmsCsLow();
    spiWriteBytes(4, cmd);

    uint8_t  rd = 0x00;
    uint32_t t0 = HAL_GetTick();
    while (rd != 0xFF) {
        if (!spiReadBytes(1, &rd) ||
            (HAL_GetTick() - t0) > BMS_POLL_TIMEOUT_MS) {
            bmsCsHigh();
            stopTimer();
            return UINT32_MAX;   /* timeout ou falha SPI */
        }
    }

    bmsCsHigh();
    uint32_t count = getTimCount();
    stopTimer();
    return count;
}
