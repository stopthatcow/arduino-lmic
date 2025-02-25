// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#include "lmic.h"

#if defined(CFG_sx127x_radio)
#define CFG_sx1276_radio  // TODO(nwiles) Support SX1272, it doesn't currently compile.
// ----------------------------------------
// Registers Mapping
#define RegFifo                                    0x00 // common
#define RegOpMode                                  0x01 // common
#define FSKRegBitrateMsb                           0x02
#define FSKRegBitrateLsb                           0x03
#define FSKRegFdevMsb                              0x04
#define FSKRegFdevLsb                              0x05
#define RegFrfMsb                                  0x06 // common
#define RegFrfMid                                  0x07 // common
#define RegFrfLsb                                  0x08 // common
#define RegPaConfig                                0x09 // common
#define RegPaRamp                                  0x0A // common
#define RegOcp                                     0x0B // common
#define RegLna                                     0x0C // common
#define FSKRegRxConfig                             0x0D
#define LORARegFifoAddrPtr                         0x0D
#define FSKRegRssiConfig                           0x0E
#define LORARegFifoTxBaseAddr                      0x0E
#define FSKRegRssiCollision                        0x0F
#define LORARegFifoRxBaseAddr                      0x0F
#define FSKRegRssiThresh                           0x10
#define LORARegFifoRxCurrentAddr                   0x10
#define FSKRegRssiValue                            0x11
#define LORARegIrqFlagsMask                        0x11
#define FSKRegRxBw                                 0x12
#define LORARegIrqFlags                            0x12
#define FSKRegAfcBw                                0x13
#define LORARegRxNbBytes                           0x13
#define FSKRegOokPeak                              0x14
#define LORARegRxHeaderCntValueMsb                 0x14
#define FSKRegOokFix                               0x15
#define LORARegRxHeaderCntValueLsb                 0x15
#define FSKRegOokAvg                               0x16
#define LORARegRxPacketCntValueMsb                 0x16
#define LORARegRxpacketCntValueLsb                 0x17
#define LORARegModemStat                           0x18
#define LORARegPktSnrValue                         0x19
#define FSKRegAfcFei                               0x1A
#define LORARegPktRssiValue                        0x1A
#define FSKRegAfcMsb                               0x1B
#define LORARegRssiValue                           0x1B
#define FSKRegAfcLsb                               0x1C
#define LORARegHopChannel                          0x1C
#define FSKRegFeiMsb                               0x1D
#define LORARegModemConfig1                        0x1D
#define FSKRegFeiLsb                               0x1E
#define LORARegModemConfig2                        0x1E
#define FSKRegPreambleDetect                       0x1F
#define LORARegSymbTimeoutLsb                      0x1F
#define FSKRegRxTimeout1                           0x20
#define LORARegPreambleMsb                         0x20
#define FSKRegRxTimeout2                           0x21
#define LORARegPreambleLsb                         0x21
#define FSKRegRxTimeout3                           0x22
#define LORARegPayloadLength                       0x22
#define FSKRegRxDelay                              0x23
#define LORARegPayloadMaxLength                    0x23
#define FSKRegOsc                                  0x24
#define LORARegHopPeriod                           0x24
#define FSKRegPreambleMsb                          0x25
#define LORARegFifoRxByteAddr                      0x25
#define LORARegModemConfig3                        0x26
#define FSKRegPreambleLsb                          0x26
#define FSKRegSyncConfig                           0x27
#define LORARegFeiMsb                              0x28
#define FSKRegSyncValue1                           0x28
#define LORAFeiMib                                 0x29
#define FSKRegSyncValue2                           0x29
#define LORARegFeiLsb                              0x2A
#define FSKRegSyncValue3                           0x2A
#define FSKRegSyncValue4                           0x2B
#define LORARegRssiWideband                        0x2C
#define FSKRegSyncValue5                           0x2C
#define FSKRegSyncValue6                           0x2D
#define FSKRegSyncValue7                           0x2E
#define FSKRegSyncValue8                           0x2F
#define FSKRegPacketConfig1                        0x30
#define FSKRegPacketConfig2                        0x31
#define LORARegDetectOptimize                      0x31
#define FSKRegPayloadLength                        0x32
#define FSKRegNodeAdrs                             0x33
#define LORARegInvertIQ                            0x33
#define FSKRegBroadcastAdrs                        0x34
#define FSKRegFifoThresh                           0x35
#define FSKRegSeqConfig1                           0x36
#define FSKRegSeqConfig2                           0x37
#define LORARegDetectionThreshold                  0x37
#define FSKRegTimerResol                           0x38
#define FSKRegTimer1Coef                           0x39
#define LORARegSyncWord                            0x39
#define FSKRegTimer2Coef                           0x3A
#define FSKRegImageCal                             0x3B
#define LORARegInvertIQ2                           0x3B
#define FSKRegTemp                                 0x3C
#define FSKRegLowBat                               0x3D
#define FSKRegIrqFlags1                            0x3E
#define FSKRegIrqFlags2                            0x3F
#define RegDioMapping1                             0x40 // common
#define RegDioMapping2                             0x41 // common
#define RegVersion                                 0x42 // common
// #define RegAgcRef                                  0x43 // common
// #define RegAgcThresh1                              0x44 // common
// #define RegAgcThresh2                              0x45 // common
// #define RegAgcThresh3                              0x46 // common
// #define RegPllHop                                  0x4B // common
#define SX1272_RegTcxo                             0x58 // common
#define SX1276_RegTcxo                             0x4B // common
#define SX1272_RegPaDac                            0x5A // common
#define SX1276_RegPaDac                            0x4D // common
// #define RegPll                                     0x5C // common
// #define RegPllLowPn                                0x5E // common
// #define RegFormerTemp                              0x6C // common
// #define RegBitRateFrac                             0x70 // common

// ----------------------------------------
// SX1272 RegModemConfig1 settings
#define SX1272_MC1_CR_4_5                    0x08
#define SX1272_MC1_CR_4_6                    0x10
#define SX1272_MC1_CR_4_7                    0x18
#define SX1272_MC1_CR_4_8                    0x20
#define SX1272_MC1_IMPLICIT_HEADER_MODE_ON   0x04 // required for receive
#define SX1272_MC1_RX_PAYLOAD_CRCON          0x02
#define SX1272_MC1_LOW_DATA_RATE_OPTIMIZE    0x01

// SX1272 RegModemConfig2 settings
#define SX1272_MC2_AGCAUTO                   0x04

// opmodes
#define OPMODE_MASK           0x07
#define OPMODE_SLEEP          0
#define OPMODE_STANDBY        1
#define OPMODE_FSTX           2
#define OPMODE_TX             3
#define OPMODE_FSRX           4
#define OPMODE_RX             5
#define OPMODE_RX_SINGLE      6
#define OPMODE_CAD            7

// LoRa opmode bits:
#define OPMODE_LORA           0x80
// SX1272: bit7=1 (LoRa), bit6=0 (AccessSharedReg=LoRa), bit5+4+3=000 (unused), bit2+1+0=mode
// SX1276: bit7=1 (LoRa), bit6=0 (AccessSharedReg=LoRa), bit5+4=00 (reserved), bit3=0 (access HF test regs), bit2+1+0=mode
#define OPMODE_LORA_SLEEP     (0b10000000+OPMODE_SLEEP)
#define OPMODE_LORA_STANDBY   (0b10000000+OPMODE_STANDBY)
#define OPMODE_LORA_FSTX      (0b10000000+OPMODE_FSTX)
#define OPMODE_LORA_TX        (0b10000000+OPMODE_TX)
#define OPMODE_LORA_FSRX      (0b10000000+OPMODE_FSRX)
#define OPMODE_LORA_RX        (0b10000000+OPMODE_RX)
#define OPMODE_LORA_RX_SINGLE (0b10000000+OPMODE_RX_SINGLE)
#define OPMODE_LORA_CAD       (0b10000000+OPMODE_CAD)

// FSK opmode bits:
#ifdef CFG_sx1272_radio
// SX1272: bit7=0 (FSK), bit6+5=00 (modulation=FSK), bit4+3=00 (no shaping), bits2+1+0=mode
#define OPMODE_FSK_SLEEP      (0b00000000+OPMODE_SLEEP)
#define OPMODE_FSK_STANDBY    (0b00000000+OPMODE_STANDBY) // (reset value of RegOpMode)
#define OPMODE_FSK_FSTX       (0b00000000+OPMODE_FSTX)
#define OPMODE_FSK_TX         (0b00000000+OPMODE_TX)
#define OPMODE_FSK_FSRX       (0b00000000+OPMODE_FSRX)
#define OPMODE_FSK_RX         (0b00000000+OPMODE_RX)
#endif

#ifdef CFG_sx1276_radio
// SX1276: bit7=0 (FSK), bit6+5=00 (modulation=FSK), bit4=0 (reserved), bit3=1 (access LF test regs), bits2+1+0=mode
#define OPMODE_FSK_SLEEP      (0b00001000+OPMODE_SLEEP)
#define OPMODE_FSK_STANDBY    (0b00001000+OPMODE_STANDBY) // (reset value of RegOpMode)
#define OPMODE_FSK_FSTX       (0b00001000+OPMODE_FSTX)
#define OPMODE_FSK_TX         (0b00001000+OPMODE_TX)
#define OPMODE_FSK_FSRX       (0b00001000+OPMODE_FSRX)
#define OPMODE_FSK_RX         (0b00001000+OPMODE_RX)
#endif

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK            0x80
#define IRQ_LORA_RXDONE_MASK            0x40
#define IRQ_LORA_CRCERR_MASK            0x20
#define IRQ_LORA_HEADER_MASK            0x10
#define IRQ_LORA_TXDONE_MASK            0x08
#define IRQ_LORA_CDDONE_MASK            0x04
#define IRQ_LORA_FHSSCH_MASK            0x02
#define IRQ_LORA_CDDETD_MASK            0x01

// interrupt flags when large packet successfully: rcvd sent
#define IRQ_FSK1_MODEREADY_MASK         0x80     // 1    1
#define IRQ_FSK1_RXREADY_MASK           0x40     // 1    0
#define IRQ_FSK1_TXREADY_MASK           0x20     // 0    1
#define IRQ_FSK1_PLLLOCK_MASK           0x10     // 1    1
#define IRQ_FSK1_RSSI_MASK              0x08     // 1    0
#define IRQ_FSK1_TIMEOUT_MASK           0x04     // 0    0
#define IRQ_FSK1_PREAMBLEDETECT_MASK    0x02     // 1    0
#define IRQ_FSK1_SYNCADDRESSMATCH_MASK  0x01     // 1    0
#define IRQ_FSK2_FIFOFULL_MASK          0x80     // 0    0
#define IRQ_FSK2_FIFOEMPTY_MASK         0x40     // 0    1
#define IRQ_FSK2_FIFOLEVEL_MASK         0x20     // 0    0
#define IRQ_FSK2_FIFOOVERRUN_MASK       0x10     // 0    0
#define IRQ_FSK2_PACKETSENT_MASK        0x08     // 0    1
#define IRQ_FSK2_PAYLOADREADY_MASK      0x04     // 1    0
#define IRQ_FSK2_CRCOK_MASK             0x02     // 1    0
#define IRQ_FSK2_LOWBAT_MASK            0x01     // 0    0

// ----------------------------------------
// DIO function mappings            MAP1:D0D1D2D3
#define MAP1_LORA_DIO0_RXDONE   0x00  // 00------
#define MAP1_LORA_DIO0_TXDONE   0x40  // 01------
#define MAP1_LORA_DIO0_NOP      0xC0  // 11------
#define MAP1_LORA_DIO1_RXTOUT   0x00  // --00----
#define MAP1_LORA_DIO1_NOP      0x30  // --11----
#define MAP1_LORA_DIO2_NOP      0x0C  // ----11--
#define MAP1_LORA_DIO3_CDDONE   0x00  // ------00
#define MAP1_LORA_DIO3_NOP      0x03  // ------11
//                                  MAP2:D4D5XXXX
#define MAP2_LORA_DIO4_NOP      0xC0  // 11------
#define MAP2_LORA_DIO5_NOP      0x30  // --11----
#define MAP2_LORA_RFU           0x00  // ----000-
#define MAP2_LORA_IRQ_PREAMBLE  0x01  // -------1

//                                  MAP1:D0D1D2D3
#define MAP1_FSK_DIO0_RXDONE    0x00  // 00------
#define MAP1_FSK_DIO0_TXDONE    0x00  // 00------
#define MAP1_FSK_DIO1_LEVEL     0x00  // --00----
#define MAP1_FSK_DIO1_EMPTY     0x10  // --01----
#define MAP1_FSK_DIO1_FULL      0x20  // --10----
#define MAP1_FSK_DIO1_NOP       0x30  // --11----
#define MAP1_FSK_DIO2_TXNOP     0x04  // ----01--
#define MAP1_FSK_DIO2_RXTOUT    0x08  // ----10--

// FSK ImageCal defines
#define RF_IMAGECAL_IMAGECAL_START      0x40
#define RF_IMAGECAL_IMAGECAL_RUNNING    0x20

// IQ Inversion
#define IQRXNORMAL  0x27 // (see AN1200.24 SX1276 settings for LoRaWAN)
#define IQ2RXNORMAL 0x1D
#define IQRXINVERT  0x67
#define IQ2RXINVERT 0x19

// operating mode transition times
#define TS_OSC    us2osticksCeil(250)     // (sleep to standby)

// radio-specific settings
#if defined(CFG_sx1276_radio)
#define RADIO_VERSION               0x12
#define RST_PIN_RESET_STATE         0
#define RSSI_HF_CONST               157
#define RegPaDac                    SX1276_RegPaDac
#define RegTcxo                     SX1276_RegTcxo
#define LORA_TXDONE_FIXUP           us2osticksRound(67)  // determined by timestamping DIO0 with SX1301 (mku/20190315)
#define LORA_RXSTART_FIXUP          us2osticksRound(101) // determined by osc measurement GPIO vs with DIO5 (mode-ready) (mku/20190315)
#define FSK_TXDONE_FIXUP            us2osticks(0) // XXX
#define FSK_RXDONE_FIXUP            us2osticks(0) // XXX
#define PARAMP50                    0b00001000 // unused=000, reserved=0, PaRamp=1000

// transmit power configuration for RegPaConfig
#define SX1276_PAC_PA_SELECT_PA_BOOST 0x80
#define SX1276_PAC_PA_SELECT_RFIO_PIN 0x00
#define SX1276_PAC_MAX_POWER_MASK     0x70

// the bits to change for max power.
#define SX127X_PADAC_POWER_MASK       0x07
#define SX127X_PADAC_POWER_NORMAL     0x04
#define SX127X_PADAC_POWER_20dBm      0x07

// bit in RegOcp that enables overcurrent protect.
#define SX127X_OCP_ENA                     0x20

// convert milliamperes to equivalent value for
// RegOcp; delivers conservative value.
#define SX127X_OCP_MAtoBITS(mA)                 \
    ((mA) < 45   ? 0 :                          \
     (mA) <= 120 ? ((mA) - 45) / 5 :            \
     (mA) < 130  ? 0xF :                        \
     (mA) < 240  ? ((mA) - 130) / 10 + 0x10 :   \
                   27)

#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 0x0A
#ifdef CFG_sx1276_radio
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x70
#elif CFG_sx1272_radio
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x74
#endif

static const u2_t LORA_RXDONE_FIXUP_125[] = {
    [FSK]  =     us2osticks(0),
    [SF7]  =     us2osticks(0),
    [SF8]  =  us2osticks(1648),
    [SF9]  =  us2osticks(3265),
    [SF10] =  us2osticks(7049),
    [SF11] = us2osticks(13641),
    [SF12] = us2osticks(31189),
};

static const u2_t LORA_RXDONE_FIXUP_500[] = {
    [FSK]  = us2osticks(    0),
    [SF7]  = us2osticks(    0),
    [SF8]  = us2osticks(    0),
    [SF9]  = us2osticks(    0),
    [SF10] = us2osticks(    0),
    [SF11] = us2osticks(    0),
    [SF12] = us2osticks(    0),
};

#elif defined(CFG_sx1272_radio)
#define RADIO_VERSION               0x22
#define RST_PIN_RESET_STATE         1
#define RSSI_HF_CONST               139
#define RegPaDac                    SX1272_RegPaDac
#define RegTcxo                     SX1272_RegTcxo
#define LORA_TXDONE_FIXUP           us2osticks(43) // XXX
#define LORA_RXSTART_FIXUP          us2osticksRound(101) // XXX
#define FSK_TXDONE_FIXUP            us2osticks(0) // XXX
#define FSK_RXDONE_FIXUP            us2osticks(0) // XXX
#define PARAMP50                    0b00011000 // unused=000, LowPnTxPllOff=1, PaRamp=1000

static const u2_t LORA_RXDONE_FIXUP_125[] = {
    [FSK]  = us2osticksRound(    0),
    [SF7]  = us2osticksRound(  749),
    [SF8]  = us2osticksRound( 1343),
    [SF9]  = us2osticksRound( 3265),
    [SF10] = us2osticksRound( 7049),
    [SF11] = us2osticksRound(13641),
    [SF12] = us2osticksRound(31189),
};

// Based Nucleo board regr tests rxlatency-regr
static const u2_t LORA_RXDONE_FIXUP_500[] = {
    [FSK]  = us2osticksRound(   0),
    [SF7]  = us2osticksRound( 193),
    [SF8]  = us2osticksRound( 344),
    [SF9]  = us2osticksRound( 737),
    [SF10] = us2osticksRound(1521),
    [SF11] = us2osticksRound(3240),
    [SF12] = us2osticksRound(6972),
};

#endif

#define FIFOTHRESH 32

// state
static struct {
    // large packet handling
    unsigned char* fifoptr;
    int fifolen;
} state;

// ----------------------------------------
static void writeReg (u1_t addr, u1_t data) {
    hal_spi_select(1);
    hal_spi(addr | 0x80);
    hal_spi(data);
    hal_spi_select(0);
}

static u1_t readReg (u1_t addr) {
    hal_spi_select(1);
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
    hal_spi_select(0);
    return val;
}

// (used by perso)
void sx127x_writeBuf (u1_t addr, u1_t* buf, u1_t len) {
    hal_spi_select(1);
    hal_spi(addr | 0x80);
    for (u1_t i = 0; i < len; i++) {
        hal_spi(buf[i]);
    }
    hal_spi_select(0);
}

// (used by  perso)
void sx127x_readBuf (u1_t addr, u1_t* buf, u1_t len) {
    hal_spi_select(1);
    hal_spi(addr & 0x7F);
    for (u1_t i = 0; i < len; i++) {
        buf[i] = hal_spi(0x00);
    }
    hal_spi_select(0);
}

void sx127x_sleep (void) {
    writeReg(RegOpMode, OPMODE_LORA_SLEEP); // LoRa/FSK bit is ignored when not in SLEEP mode
}

// set and wait for opmode (nsornin 2019-09-26)
static void setopmode (u1_t opmode) {
    writeReg(RegOpMode, opmode);
    ostime_t t0 = os_getTime();
    while (readReg(RegOpMode) != opmode) {
        if (os_getTime() - t0 > ms2osticks(20)) {
            // panic when opmode is not reached within 20ms
            LMIC_DEBUG_PRINTF("FAILED TO SET OPMODE %02x within 20ms\r\n", opmode);
            ASSERT(0);
        }
    }
}

// fill fifo when empty
static void LoadFifo (void) {
    if (state.fifolen > 0) {
        int n = (state.fifolen > FIFOTHRESH) ? FIFOTHRESH : state.fifolen;
        sx127x_writeBuf(RegFifo, state.fifoptr, n);
        state.fifoptr += n;
        state.fifolen -= n;
    }
}

// read fifo when level or ready
static void UnloadFifo (void) {
    if (state.fifolen < 0) { // first byte
        state.fifolen = 0;
        sx127x_readBuf(RegFifo, &LMIC.dataLen, 1);
    }
    int n = (LMIC.dataLen - state.fifolen > (FIFOTHRESH-1)) ? (FIFOTHRESH-1) : (LMIC.dataLen - state.fifolen); // errata: unload one byte less
    if (n) {
        sx127x_readBuf(RegFifo, state.fifoptr, n);
        state.fifoptr += n;
        state.fifolen += n;
    }
}

// configure LoRa modem
static void configLoraModem (bool txcont) {
#if defined(CFG_sx1276_radio)
    // set ModemConfig1 'bbbbccch' (bw=xxxx, cr=xxx, implicitheader=x)
    writeReg(LORARegModemConfig1,
             ((getBw(LMIC.rps) - BW125 + 7) << 4) | // BW125 --> 7
             ((getCr(LMIC.rps) - CR_4_5 + 1) << 1) | // CR_4_5 --> 1
             (getIh(LMIC.rps) != 0));       // implicit header

    // set ModemConfig2 'sssstcmm' (sf=xxxx, txcont=x, rxpayloadcrc=x, symtimeoutmsb=00)
    writeReg(LORARegModemConfig2,
             ((getSf(LMIC.rps)-SF7+7) << 4) |   // SF7 --> 7
             (txcont ? 0x08 : 0x00)       |     // txcont: 0x08
             ((getNocrc(LMIC.rps) == 0) << 2)); // rxcrc

    // set ModemConfig3 'uuuuoarr' (unused=0000, lowdatarateoptimize=x, agcauto=1, reserved=00)
    writeReg(LORARegModemConfig3,
             (enDro(LMIC.rps) << 3) | // symtime >= 16ms
             (1 << 2));               // autoagc

    // SX1276 Errata: 2.1 Sensitivity Optimization with a 500kHz Bandwith
    if (getBw(LMIC.rps) == BW500) {
        writeReg(0x36, 0x02);
        writeReg(0x3A, 0x64);
    } else {
        writeReg(0x36, 0x03);
        // no need to reset register 0x3a
    }
#elif defined(CFG_sx1272_radio)
    // set ModemConfig1 'bbccchco' (bw=xx, cr=xxx, implicitheader=x, rxpayloadcrc=x, lowdatarateoptimize=x)
    writeReg(LORARegModemConfig1,
             ((getBw(LMIC.rps) - BW125) << 6) |       // BW125 --> 0
             ((getCr(LMIC.rps) - CR_4_5 + 1) << 3) |  // CR_4_5 --> 1
             ((getIh(LMIC.rps) != 0) << 2) |    // implicit header
             ((getNocrc(LMIC.rps) == 0) << 1) | // rxcrc
             enDro(LMIC.rps));                  // symtime >= 16ms

    // set ModemConfig2 'sssstamm' (sf=xxxx, txcont=x, agcauto=1 symtimeoutmsb=00)
    writeReg(LORARegModemConfig2,
             ((getSf(LMIC.rps)-SF7+7) << 4) | // SF7 --> 7
             (txcont ? 0x08 : 0x00)       | // txcont: 0x08
             (1 << 2));                     // autoagc
#endif // CFG_sx1272_radio

    if (getIh(LMIC.rps)) {
        writeReg(LORARegPayloadLength, getIh(LMIC.rps)); // required length
    }
}

static void configChannel (void) {
    // set frequency: FQ = (FRF * 32 Mhz) / (2 ^ 19)
    u4_t frf = ((uint64_t)LMIC.freq << 19) / 32000000;
    writeReg(RegFrfMsb, frf >> 16);
    writeReg(RegFrfMid, frf >> 8);
    writeReg(RegFrfLsb, frf >> 0);
}

// board-specific macro to determine which PA is to be used based on frequency and output power
#ifndef BRD_PABOOSTSEL
#define BRD_PABOOSTSEL(f,p) true
#endif

// board-specific macro to determine which antenna switch port is to be used based on frequency and output power
#ifndef BRD_TXANTSWSEL
#define BRD_TXANTSWSEL(f,p) HAL_ANTSW_TX
#endif

// On the SX1276, we have several possible configs.
// 1) using RFO, MaxPower==0: in that case power is -4 to 11 dBm
// 2) using RFO, MaxPower==7: in that case, power is 0 to 14 dBm
//      (can't select 15 dBm).
//	note we can use -4..11 w/o Max and then 12..14 w/Max, and
//	we really don't need to ask anybody.
// 3) using PA_BOOST, PaDac = 4: in that case power range is 2 to 17 dBm;
//	use this for 15..17 if authorized.
// 4) using PA_BOOST, PaDac = 7, OutputPower=0xF: in that case, power is 20 dBm
//		(and perhaps 0xE is 19, 0xD is 18 dBm, but datasheet isn't clear.)
//    and duty cycle must be <= 1%.
//
// In addition, there are some boards for which PA_BOOST can only be used if the
// channel frequency is greater than SX127X_FREQ_LF_MAX.
//
// The SX1272 is similar but has no MaxPower bit:
// 1) using RFO: power is -1 to 13 dBm (datasheet implies max OutputPower value is 14 for 13 dBm)
// 2) using PA_BOOST, PaDac = 0x84: power is 2 to 17 dBm;
//	use this for 14..17 if authorized
// 3) using PA_BOOST, PaDac = 0x87, OutputPower = 0xF: power is 20dBm
//    and duty cycle must be <= 1%
//
// The general policy is to use the lowest power variant that will get us where we
// need to be.
//

static void configPower () {
    // our input paramter -- might be different than LMIC.txpow!
    s1_t const req_pw = (s1_t)LMIC.radio_txpow;
    // the effective power
    s1_t eff_pw;
    // the policy; we're going to compute this.
    u1_t policy;
    // what we'll write to RegPaConfig
    u1_t rPaConfig;
    // what we'll write to RegPaDac
    u1_t rPaDac;
    // what we'll write to RegOcp
    u1_t rOcp;

#ifdef CFG_sx1276_radio
    if (req_pw >= 20) {
        policy = LMICHAL_radio_tx_power_policy_20dBm;
        eff_pw = 20;
    } else if (req_pw >= 14) {
        policy = LMICHAL_radio_tx_power_policy_paboost;
        if (req_pw > 17) {
            eff_pw = 17;
        } else {
            eff_pw = req_pw;
        }
    } else {
        policy = LMICHAL_radio_tx_power_policy_rfo;
        if (req_pw < -4) {
            eff_pw = -4;
        } else {
            eff_pw = req_pw;
        }
    }

    policy = hal_getTxPowerPolicy(policy, eff_pw, LMIC.freq);

    switch (policy) {
    default:
    case LMICHAL_radio_tx_power_policy_rfo:
        rPaDac = SX127X_PADAC_POWER_NORMAL;
        rOcp = SX127X_OCP_MAtoBITS(80);

        if (eff_pw > 14)
            eff_pw = 14;
        if (eff_pw > 11) {
            // some Semtech code uses this down to eff_pw == 0.
            rPaConfig = eff_pw | SX1276_PAC_MAX_POWER_MASK;
        } else {
            if (eff_pw < -4)
                eff_pw = -4;
            rPaConfig = eff_pw + 4;
        }
        break;

    // some radios (HopeRF RFM95W) don't support RFO well,
    // so the policy might *raise* rfo to paboost. That means
    // we have to re-check eff_pw, which might be too small.
    // (And, of course, it might also be too large.)
    case LMICHAL_radio_tx_power_policy_paboost:
        // It seems that SX127x doesn't like eff_pw 10 when in FSK mode.
        if (getSf(LMIC.rps) == FSK && eff_pw < 11) {
            eff_pw = 11;
        }
        rPaDac = SX127X_PADAC_POWER_NORMAL;
        rOcp = SX127X_OCP_MAtoBITS(100);
        if (eff_pw > 17)
            eff_pw = 17;
        else if (eff_pw < 2)
            eff_pw = 2;
        rPaConfig = (eff_pw - 2) | SX1276_PAC_PA_SELECT_PA_BOOST;
        break;

    case LMICHAL_radio_tx_power_policy_20dBm:
        rPaDac = SX127X_PADAC_POWER_20dBm;
        rOcp = SX127X_OCP_MAtoBITS(130);
        rPaConfig = 0xF | SX1276_PAC_PA_SELECT_PA_BOOST;
        break;
    }

#elif CFG_sx1272_radio
    if (req_pw >= 20) {
        policy = LMICHAL_radio_tx_power_policy_20dBm;
            eff_pw = 20;
    } else if (eff_pw >= 14) {
        policy = LMICHAL_radio_tx_power_policy_paboost;
        if (eff_pw > 17) {
            eff_pw = 17;
        } else {
            eff_pw = req_pw;
        }
    } else {
        policy = LMICHAL_radio_tx_power_policy_rfo;
        if (req_pw < -1) {
            eff_pw = -1;
        } else {
            eff_pw = req_pw;
        }
    }

    policy = hal_getTxPowerPolicy(policy, eff_pw, LMIC.freq);

    switch (policy) {
    default:
    case LMICHAL_radio_tx_power_policy_rfo:
        rPaDac = SX127X_PADAC_POWER_NORMAL;
        rOcp = SX127X_OCP_MAtoBITS(50);

        if (eff_pw > 13)
            eff_pw = 13;

        rPaConfig = eff_pw + 1;
        break;

    case LMICHAL_radio_tx_power_policy_paboost:
        rPaDac = SX127X_PADAC_POWER_NORMAL;
        rOcp = SX127X_OCP_MAtoBITS(100);

        if (eff_pw > 17)
            eff_pw = 17;

        rPaConfig = (eff_pw - 2) | SX1272_PAC_PA_SELECT_PA_BOOST;
        break;

    case LMICHAL_radio_tx_power_policy_20dBm:
        rPaDac = SX127X_PADAC_POWER_20dBm;
        rOcp = SX127X_OCP_MAtoBITS(130);

        rPaConfig = 0xF | SX1276_PAC_PA_SELECT_PA_BOOST;
        break;
    }
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif /* CFG_sx1272_radio */

    writeReg(RegPaConfig, rPaConfig);
    writeReg(RegPaDac, (readReg(RegPaDac) & ~SX127X_PADAC_POWER_MASK) | rPaDac);
    writeReg(RegOcp, rOcp | SX127X_OCP_ENA);
}

static void power_tcxo (void) {
    // power-up TCXO and set tcxo as input
    if ( hal_pin_tcxo(1) ) {
        writeReg(RegTcxo, 0b00011001); // reserved=000, tcxo=1, reserved=1001
        // delay to allow TCXO to wake up
        hal_waitUntil(os_getTime() + ms2osticks(1));
    }
}

// continuous wave
void sx127x_cw (void) {
    // select FSK modem (from sleep mode)
    setopmode(OPMODE_FSK_SLEEP);

    // power-up tcxo
    power_tcxo();

    // enter standby mode
    setopmode(OPMODE_FSK_STANDBY);

    // set frequency deviation
    writeReg(FSKRegFdevMsb, 0x00);
    writeReg(FSKRegFdevLsb, 0x00);

    // configure frequency
    configChannel();

    // configure output power
    configPower();

    // set continuous mode
    writeReg(FSKRegPacketConfig2, 0x00);

    // initialize the payload size and address pointers
    writeReg(FSKRegPayloadLength, 1);
    writeReg(RegFifo, 0);

    // enable antenna switch for TX
    //hal_ant_switch(BRD_TXANTSWSEL(LMIC.freq, pw));
    hal_pin_rxtx(1);

    // now we actually start the transmission
    writeReg(RegOpMode, OPMODE_FSK_TX);
}

static void txfsk (bool txcont) {
    // select FSK modem (from sleep mode)
    setopmode(OPMODE_FSK_SLEEP);

    // power-up tcxo
    power_tcxo();

    // enter standby mode
    setopmode(OPMODE_FSK_STANDBY);

    // set bitrate 50kbps
    writeReg(FSKRegBitrateMsb, 0x02); // 32000000 / 50000 = 640 = 0x0280
    writeReg(FSKRegBitrateLsb, 0x80);

    // set frequency deviation +/-25kHz
    writeReg(FSKRegFdevMsb, 0x01);
    writeReg(FSKRegFdevLsb, 0x99);

    // frame and packet handler settings
    writeReg(FSKRegPreambleMsb, 0x00); // 5 bytes preamble
    writeReg(FSKRegPreambleLsb, 0x05);
    writeReg(FSKRegSyncConfig, 0x12);  // 3 bytes sync word 0xC194C1
    writeReg(FSKRegSyncValue1, 0xC1);
    writeReg(FSKRegSyncValue2, 0x94);
    writeReg(FSKRegSyncValue3, 0xC1);
    writeReg(FSKRegPacketConfig1, 0xD0); // varlen + whitening + crc + noaddr
    writeReg(FSKRegPacketConfig2, (txcont) ? 0x00 : 0x40); // continuous mode or packet mode

    // configure frequency
    configChannel();

    // configure output power
    configPower();

    // set the IRQ mapping DIO0=PacketSent DIO1=FifoEmpty DIO2=NOP
    writeReg(RegDioMapping1, MAP1_FSK_DIO0_TXDONE | MAP1_FSK_DIO1_EMPTY | MAP1_FSK_DIO2_TXNOP);

    // setup FIFO
    writeReg(FSKRegFifoThresh, 0x80); // TxStartCondition !FifoEmpty
    // write length byte
    writeReg(RegFifo, LMIC.dataLen);
    // write payload (full or partial)
    state.fifoptr = LMIC.frame;
    state.fifolen = LMIC.dataLen;
    LoadFifo();

    if (!txcont) {
        // enable IRQs in HAL
        hal_irqmask_set(HAL_IRQMASK_DIO0 | HAL_IRQMASK_DIO1);

        // set tx timeout
        radio_set_irq_timeout(os_getTime() + us2osticks((u4_t)(FIFOTHRESH+10)*8*1000/50));
    }

    // enable antenna switch for TX
    //hal_ant_switch(BRD_TXANTSWSEL(LMIC.freq, pw));
    hal_pin_rxtx(1);

    // now we actually start the transmission
    writeReg(RegOpMode, OPMODE_FSK_TX);
}

static void txlora (bool txcontinuous) {
    // select LoRa modem (from sleep mode)
    setopmode(OPMODE_LORA_SLEEP);

    // power-up tcxo
    power_tcxo();

    // enter standby mode
    setopmode(OPMODE_LORA_STANDBY);

    // configure LoRa modem
    configLoraModem(txcontinuous);

    // configure frequency
    configChannel();

    // configure output power
    configPower();

    // set sync word
    writeReg(LORARegSyncWord, 0x34);

    // set IQ inversion mode
    writeReg(LORARegInvertIQ,  IQRXNORMAL);
    writeReg(LORARegInvertIQ2, IQ2RXNORMAL);

    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP DIO3=NOP DIO4=NOP DIO5=NOP
    writeReg(RegDioMapping1, MAP1_LORA_DIO0_TXDONE | MAP1_LORA_DIO1_NOP | MAP1_LORA_DIO2_NOP | MAP1_LORA_DIO3_NOP);
    writeReg(RegDioMapping2, MAP2_LORA_DIO4_NOP | MAP2_LORA_DIO5_NOP);

    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);

    // mask all IRQs but TxDone
    writeReg(LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO0);

    // initialize the payload size and address pointers
    writeReg(LORARegFifoTxBaseAddr, 0x00);
    writeReg(LORARegFifoAddrPtr, 0x00);
    writeReg(LORARegPayloadLength, LMIC.dataLen);

    // download buffer to the radio FIFO
    sx127x_writeBuf(RegFifo, LMIC.frame, LMIC.dataLen);

    // enable antenna switch for TX
    //hal_ant_switch(BRD_TXANTSWSEL(LMIC.freq, pw));
    hal_pin_rxtx(1);

    // now we actually start the transmission
    writeReg(RegOpMode, OPMODE_LORA_TX);
}

static void setuprxlora (void) {
    // select LoRa modem (from sleep mode)
    setopmode(OPMODE_LORA_SLEEP);

    // power-up tcxo
    power_tcxo();

    // enter standby mode
    setopmode(OPMODE_LORA_STANDBY);

    // configure LoRa modem (cfg1, cfg2, cfg3)
    configLoraModem(false);

    // configure frequency
    configChannel();

    // set LNA gain 'gggbbrbb' (LnaGain=001 (max), LnaBoostLf=00 (default), reserved=0, LnaBoostHf=11 (150%))
    writeReg(RegLna, 0b00100011);

    // set max payload size
    writeReg(LORARegPayloadMaxLength, MAX_LEN_FRAME);

    // set IQ inversion mode
    writeReg(LORARegInvertIQ,  (LMIC.noRXIQinversion) ? IQRXNORMAL  : IQRXINVERT);
    writeReg(LORARegInvertIQ2, (LMIC.noRXIQinversion) ? IQ2RXNORMAL : IQ2RXINVERT);

    // set max preamble length 8
    writeReg(LORARegPreambleMsb, 0x00);
    writeReg(LORARegPreambleLsb, 0x08);

    // set symbol timeout (for single rx)
    writeReg(LORARegSymbTimeoutLsb, LMIC.rxsyms);

    // set sync word
    writeReg(LORARegSyncWord, 0x34);
}


// workaround to improve likelihood of preamble detection when receiver started at symbol boundary
//
//  - use 7 symbols as symbtimeout (see lmic.c: MINRX_SYMS=7)
//      --> aim for center of symbol, not beginning of symbol
//      --> allow for 4 out of 6 identical peaks in sliding window of 6
//          (receiver is stopped exactly after SymbTimeout, but there is processing overhead, so it's one symbol less)
//
//  - shift start time of receiver randomly by up to 500us
//
static ostime_t bugfix_rxtime (ostime_t rxtime) {
    // Note: in ticks, the resolution is 30.5 us @ 32.768 kHz
    //       use ceil() to ensure a delay of at least 1 tick
    if( getSf(LMIC.rps) >= SF9 ) {
        // SX127x bug workaround: random delay 1-512 us
        rxtime += us2osticksCeil(1 + (os_getRndU2() & 0x1ff));
    }
    return rxtime;
}

static void rxlorasingle (void) {
    ostime_t t0 = os_getTime();

    // select modem, setup TCXO, freq, modulation
    setuprxlora();

    // configure DIO mapping DIO0=RxDone DIO1=Timeout DIO2=NOP DIO3=NOP DIO4=NOP DIO5=NOP
    writeReg(RegDioMapping1, (MAP1_LORA_DIO0_RXDONE | MAP1_LORA_DIO1_RXTOUT | MAP1_LORA_DIO2_NOP | MAP1_LORA_DIO3_NOP));
    writeReg(RegDioMapping2, MAP2_LORA_DIO4_NOP | MAP2_LORA_DIO5_NOP);

    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);

    // enable required radio IRQs
    writeReg(LORARegIrqFlagsMask, (uint8_t) ~(IRQ_LORA_RXDONE_MASK | IRQ_LORA_RXTOUT_MASK));

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO0 | HAL_IRQMASK_DIO1);

    // now instruct the radio to receive
    // (lock interrupts only for final fine tuned rx timing...)
    hal_disableIRQs();
    // busy wait until exact rx time
    ostime_t rxtime = LMIC.rxtime - LORA_RXSTART_FIXUP;
    // SX127x bug fix: move exact RX time away from symbol boundary
    rxtime = bugfix_rxtime(rxtime);
    // wait for it...
    ostime_t now = os_getTime();
    hal_waitUntil(rxtime);
    // enable antenna switch for RX (and account power consumption)
    //hal_ant_switch(HAL_ANTSW_RX);
    hal_pin_rxtx(0);

    // rx now...
    writeReg(RegOpMode, OPMODE_LORA_RX_SINGLE);
    // re-enable interrupts
    hal_enableIRQs();
    // warn about delayed rx
    if( LMIC.rxtime - now < 0 ) {
        LMIC_DEBUG_PRINTF("WARNING: LMIC.rxtime is %d ticks in the past! (ramp-up time %d ms / %d ticks)\r\n",
                     now - LMIC.rxtime, osticks2ms(now - t0), now - t0);
    }
}

static void rxloracont (void) {
    // select modem, setup TCXO, freq, modulation
    setuprxlora();

    // configure DIO mapping DIO0=RxDone DIO1=NOP DIO2=NOP DIO3=NOP DIO4=NOP DIO5=NOP
    writeReg(RegDioMapping1, MAP1_LORA_DIO0_RXDONE | MAP1_LORA_DIO1_NOP | MAP1_LORA_DIO2_NOP | MAP1_LORA_DIO3_NOP);
    writeReg(RegDioMapping2, MAP2_LORA_DIO4_NOP | MAP2_LORA_DIO5_NOP);

    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);

    // enable required radio IRQs
    writeReg(LORARegIrqFlagsMask, ~IRQ_LORA_RXDONE_MASK);

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO0);

    // now instruct the radio to receive
    // enable antenna switch for RX (and account power consumption)
    // hal_ant_switch(HAL_ANTSW_RX);
    hal_pin_rxtx(0);

    // rx now...
    writeReg(RegOpMode, OPMODE_LORA_RX);
}

static void rxloracad (void) {
    // select modem, setup TCXO, freq, modulation
    setuprxlora();

    // configure DIO mapping DIO0=RxDone DIO1=RxTout DIO2=NOP DIO3=CadDone DIO4=NOP DIO5=NOP
    writeReg(RegDioMapping1, MAP1_LORA_DIO0_RXDONE | MAP1_LORA_DIO1_RXTOUT | MAP1_LORA_DIO2_NOP | MAP1_LORA_DIO3_CDDONE);
    writeReg(RegDioMapping2, MAP2_LORA_DIO4_NOP | MAP2_LORA_DIO5_NOP);

    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);

    // enable required radio IRQs
    writeReg(LORARegIrqFlagsMask, (uint8_t) ~(IRQ_LORA_CDDONE_MASK | IRQ_LORA_CDDETD_MASK | IRQ_LORA_RXDONE_MASK | IRQ_LORA_RXTOUT_MASK));

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO0 | HAL_IRQMASK_DIO1 | HAL_IRQMASK_DIO3);

    // now instruct the radio to receive
    // enable antenna switch for RX (and account power consumption)
    //hal_ant_switch(HAL_ANTSW_RX);
    hal_pin_rxtx(0);

    // start CAD...
    writeReg(RegOpMode, OPMODE_LORA_CAD);
}

static void rxfsk (bool rxcontinuous) {
    // configure radio (needs rampup time)
    ostime_t t0 = os_getTime();

    // select FSK modem (from sleep mode)
    setopmode(OPMODE_FSK_SLEEP);

    // power-up tcxo
    power_tcxo();

    // enter standby mode
    setopmode(OPMODE_FSK_STANDBY);

    // configure frequency
    configChannel();

    // set bitrate 50kbps
    writeReg(FSKRegBitrateMsb, 0x02);  // 32000000 / 50000 = 640 = 0x0280
    writeReg(FSKRegBitrateLsb, 0x80);

    // set LNA gain
    writeReg(RegLna, 0b00100011); // highest gain, boost enable

    // configure receiver
    writeReg(FSKRegRxConfig, 0b00011110); // no restart, auto afc, auto agc, trigger on preamble

    // set receiver bandwidth
    writeReg(FSKRegRxBw, 0b00001011); // 50kHz SSB

    // set AFC bandwidth
    writeReg(FSKRegAfcBw, 0b00010010); // 83.3kHz SSB

    // set preamble detection
    writeReg(FSKRegPreambleDetect, 0b10101010); // enable, 2 bytes, 10 chip errors

    // set sync config
    writeReg(FSKRegSyncConfig, 0b00010010); // no auto restart, preamble 0xaa, sync addr enable, fill fifo, 3 bytes sync word

    // set sync word
    writeReg(FSKRegSyncValue1, 0xC1);
    writeReg(FSKRegSyncValue2, 0x94);
    writeReg(FSKRegSyncValue3, 0xC1);

    // set packet config
    writeReg(FSKRegPacketConfig1, 0b11011000); // var-length, whitening, crc, no auto-clear irq, no adr filter, ccitt crc
    writeReg(FSKRegPacketConfig2, 0b01000000); // packet mode

    // set max payload length
    writeReg(FSKRegPayloadLength, MAX_LEN_FRAME);

    // set fifo threshold
    writeReg(FSKRegFifoThresh, FIFOTHRESH);

    state.fifolen = -1;
    state.fifoptr = LMIC.frame;

    // configure DIO mapping DIO0=RxPayloadReady DIO1=FifoLevel DIO2=RxTimeOut
    writeReg(RegDioMapping1, MAP1_FSK_DIO0_RXDONE | MAP1_FSK_DIO1_LEVEL | MAP1_FSK_DIO2_RXTOUT);

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO0 | HAL_IRQMASK_DIO1 | HAL_IRQMASK_DIO2);

    // now instruct the radio to receive
    hal_disableIRQs();

    if (rxcontinuous) {
        // XXX not suppported - receiver does not automatically restart
        radio_set_irq_timeout(os_getTime() + sec2osticks(5)); // time out after 5 sec
    } else {
        // set preamble timeout
        writeReg(FSKRegRxTimeout2, (LMIC.rxsyms + 1) / 2); // (TimeoutRxPreamble * 16 * Tbit)
        // set rx timeout
        radio_set_irq_timeout(LMIC.rxtime + us2osticks((u4_t)(2*FIFOTHRESH)*8*1000/50));
        // busy wait until exact rx time
        ostime_t now = os_getTime();
        if (LMIC.rxtime - now < 0) {
            LMIC_DEBUG_PRINTF("WARNING: rxtime is %d ticks in the past! (ramp-up time %d ms / %d ticks)\r\n",
                         now - LMIC.rxtime, osticks2ms(now - t0), now - t0);
        }
        hal_waitUntil(LMIC.rxtime);
    }

    // enable antenna switch for RX (and account power consumption)
    //hal_ant_switch(HAL_ANTSW_RX);
    hal_pin_rxtx(0);

    // rx
    writeReg(RegOpMode, OPMODE_FSK_RX);
    hal_enableIRQs();
}

void sx127x_startrx (bool rxcontinuous) {
    ASSERT( (readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );

    if (isFsk(LMIC.rps)) { // FSK modem
        rxfsk(rxcontinuous);
    } else { // LoRa modem
        if (rxcontinuous) {
            rxloracont();
        } else {
            rxlorasingle();
        }
    }
}

void sx127x_cad (void) {
    ASSERT( (readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );

    rxloracad();
}

void sx127x_starttx (bool txcontinuous) {
    ASSERT( (readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );
    if (isFsk(LMIC.rps)) { // FSK modem
        txfsk(txcontinuous);
    } else { // LoRa modem
        txlora(txcontinuous);
    }
}

// LMIC.rssi = max_rssi(threshold=LMIC.rssi, duration=LMIC.rxtime, freq=LMIC.freq, bw=LMIC.rps)
void sx127x_cca (void) {
    // select FSK modem (from sleep mode)
    ASSERT( (readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );
    setopmode(OPMODE_FSK_SLEEP);

    // power-up tcxo
    power_tcxo();

    // enter standby mode
    setopmode(OPMODE_FSK_STANDBY);

    // configure frequency
    configChannel();

    // set LNA gain
    writeReg(RegLna, 0b00100011); // highest gain, boost enable

    // set receiver bandwidth (SSB)
    writeReg(FSKRegRxBw, (isFsk(LMIC.rps)) ? 0x0B /* 50kHz SSB */ :
             3 - getBw(LMIC.rps)); // 62.5/125/250kHz SSB (RxBwMant=0, RxBwExp 3/2/1)

    // enable antenna switch for RX (and account power consumption)
    hal_pin_rxtx(0);

    // start receiver, don't receive frames
    setopmode(OPMODE_FSK_RX);

    // initialize threshold
    int rssi;
    int rssi_th = LMIC.rssi;
    int rssi_max = -128 + RSSI_OFF;
    ostime_t t0 = os_getTime();

    // sample rssi values
    do {
        rssi = -readReg(FSKRegRssiValue) / 2 + RSSI_OFF;
        if (rssi > rssi_max) {
            rssi_max = rssi;
        }
    } while (rssi < rssi_th && os_getTime() - t0 < LMIC.rxtime);

    // return max observed rssi value
    LMIC.rssi = rssi_max;

    // shutdown receiver
    radio_sleep();

    // disable antenna switch
    hal_pin_rxtx(0);

    // power-down TCXO
    hal_pin_tcxo(0);
}

// reset radio
static void sx127x_reset (void) {
    // drive RST pin
    hal_pin_rst(RST_PIN_RESET_STATE);

    // power-down TCXO
    hal_pin_tcxo(0);

    // wait > 100us
    hal_waitUntil(os_getTime() + ms2osticks(1));

    // configure RST pin floating
    hal_pin_rst(2);

    // wait > 5ms
    hal_waitUntil(os_getTime() + ms2osticks(10));

    // check opmode
    ASSERT( readReg(RegOpMode) == OPMODE_FSK_STANDBY );
}

int sx127x_init (bool calibrate) {
    hal_disableIRQs();

    // power-up tcxo
    power_tcxo();

    // reset radio (FSK/STANDBY)
    sx127x_reset();

    // sanity check, read version number
    if( readReg(RegVersion) != RADIO_VERSION ) {
      return 0;
    }

    // disable automatic image rejection calibration
    writeReg(FSKRegImageCal, 0x00);

    // optionally perform receiver chain calibration in FSK/STANDBY mode
    if (calibrate) {
        // set band/frequency
        configChannel();

        // run receiver chain calibration
        writeReg(FSKRegImageCal, RF_IMAGECAL_IMAGECAL_START); // (clear auto-cal)
        while ( readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING );
    }

    // go to SLEEP mode
    setopmode(OPMODE_FSK_SLEEP);

    // power-down TCXO
    hal_pin_tcxo(0);

    hal_enableIRQs();
    return 1;
}

void sx127x_generate_random (u1_t *buffer, u1_t len){
    // select LoRa modem (from sleep mode)
    setopmode(OPMODE_LORA_SLEEP);

    // power-up tcxo
    power_tcxo();

    // enter standby mode
    setopmode(OPMODE_LORA_STANDBY);
    // Override MAC settings for random generation.
    writeReg(LORARegModemConfig1, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1);
    writeReg(LORARegModemConfig2, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2);
    hal_irqmask_set(0);
    setopmode(OPMODE_LORA_RX);
    for(int i=0; i<len; i++) {
        for(int j=0; j<8; j++) {
            u1_t b; // wait for two non-identical subsequent least-significant bits
            while( (b = readReg(LORARegRssiWideband) & 0x01) == (readReg(LORARegRssiWideband) & 0x01) );
            buffer[i] = (buffer[i] << 1) | b;
        }
    }
    // shutdown receiver
    radio_sleep();

    // disable antenna switch
    hal_pin_rxtx(0);

    // power-down TCXO
    hal_pin_tcxo(0);
}


// (run by irqjob)
bool sx127x_irq_process (ostime_t irqtime, u1_t diomask) {
    (void)diomask; //unused

    // dispatch modem
    if (isFsk(LMIC.rps)) { // FSK modem
        u1_t irqflags1 = readReg(FSKRegIrqFlags1);
        u1_t irqflags2 = readReg(FSKRegIrqFlags2);

        if (irqflags2 & IRQ_FSK2_PACKETSENT_MASK) { // TXDONE

            // save exact tx time
            LMIC.txend = irqtime - FSK_TXDONE_FIXUP;

        } else if (irqflags2 & IRQ_FSK2_PAYLOADREADY_MASK) { // RXDONE

            // read rx quality parameters (at end of packet, not optimal since energy might already be gone)
            // (unfortunately in SX1272/SX1276 no averaged RSSI available in FSK mode, better in SX1261)
            LMIC.rssi = -readReg(FSKRegRssiValue) / 2 + RSSI_OFF;
            LMIC.snr = 0; // N/A

            // read FIFO
            UnloadFifo();

            // save exact rx timestamps
            LMIC.rxtime  = irqtime - FSK_RXDONE_FIXUP; // end of frame timestamp
            //LMIC.rxtime0 = LMIC.rxtime - calcAirTime(LMIC.rps, LMIC.dataLen); // beginning of frame timestamp
#ifdef DEBUG_RX
            LMIC_DEBUG_PRINTF("RX[rssi=%d,len=%d]: %h\r\n",
                         LMIC.rssi - RSSI_OFF, LMIC.dataLen, LMIC.frame, LMIC.dataLen);
#endif
        } else if (irqflags1 & IRQ_FSK1_TIMEOUT_MASK) { // TIMEOUT
            // indicate timeout
            LMIC.dataLen = 0;
#ifdef DEBUG_RX
            LMIC_DEBUG_PRINTF("RX: TIMEOUT (%d us)\r\n", osticks2us(irqtime - LMIC.rxtime));
#endif
        } else if( irqflags2 & IRQ_FSK2_FIFOEMPTY_MASK ) { // FIFOEMPTY (TX)

            // fill FIFO buffer
            LoadFifo();

            // update tx timeout
            radio_set_irq_timeout(irqtime + us2osticks((u4_t)(FIFOTHRESH+10)*8*1000/50));

            // keep waiting for FifoEmpty or PacketSent interrupt
            return false;

        } else if( irqflags2 & IRQ_FSK2_FIFOLEVEL_MASK ) { // FIFOLEVEL (RX)

            // read FIFO buffer
            UnloadFifo();

            // update rx timeout
            radio_set_irq_timeout(irqtime + us2osticks((u4_t)(FIFOTHRESH+10)*8*1000/50));

            // keep waiting for FifoLevel or PayloadReady interrupt
            return false;

        } else {
            // unexpected irq
            LMIC_DEBUG_PRINTF("UNEXPECTED FSK IRQ %02x %02x\r\n", irqflags1, irqflags2);
            ASSERT(0);
        }

        // clear FSK IRQ flags
        writeReg(FSKRegIrqFlags1, 0xFF);
        writeReg(FSKRegIrqFlags2, 0xFF);

    } else { // LORA modem
        u1_t irqflags = readReg(LORARegIrqFlags);

        if (irqflags & IRQ_LORA_TXDONE_MASK) { // TXDONE

            // save exact tx time
            LMIC.txend = irqtime - LORA_TXDONE_FIXUP;

        } else if (irqflags & IRQ_LORA_RXDONE_MASK) { // RXDONE (rx or scan)

            // read rx quality parameters (averaged over packet)
            LMIC.snr  = readReg(LORARegPktSnrValue);  // SNR [dB] * 4
            LMIC.rssi = readReg(LORARegPktRssiValue); // final values -128..127 correspond to -196...+63 dBm (subtract RSSI_OFF)
            if (LMIC.snr < 0) {
                LMIC.rssi = -RSSI_HF_CONST + LMIC.rssi + LMIC.snr/4 + RSSI_OFF;
            } else {
                LMIC.rssi = -RSSI_HF_CONST + LMIC.rssi * 16/15 + RSSI_OFF;
            }

            // get PDU length
            LMIC.dataLen = readReg(LORARegRxNbBytes);

            // save exact rx timestamps
            LMIC.rxtime = irqtime; // end of frame timestamp
            if (getBw(LMIC.rps) == BW125) {
                LMIC.rxtime -= LORA_RXDONE_FIXUP_125[getSf(LMIC.rps)];
            }
            else if (getBw(LMIC.rps) == BW500) {
                LMIC.rxtime -= LORA_RXDONE_FIXUP_500[getSf(LMIC.rps)];
            }
            //LMIC.rxtime0 = LMIC.rxtime - calcAirTime(LMIC.rps, LMIC.dataLen); // beginning of frame timestamp

            // set FIFO read address pointer (to address of last packet received)
            writeReg(LORARegFifoAddrPtr, readReg(LORARegFifoRxCurrentAddr));

            // read FIFO
            sx127x_readBuf(RegFifo, LMIC.frame, LMIC.dataLen);
#ifdef DEBUG_RX
            LMIC_DEBUG_PRINTF("RX[rssi=%d,snr=%.2F,len=%d]: %.80h\r\n",
                         LMIC.rssi - RSSI_OFF, (s4_t)(LMIC.snr * 100 / SNR_SCALEUP), 2,
                         LMIC.dataLen, LMIC.frame, LMIC.dataLen);
#endif
        } else if (irqflags & IRQ_LORA_RXTOUT_MASK) { // RXTOUT
            // indicate timeout
            LMIC.dataLen = 0;
#ifdef DEBUG_RX
            LMIC_DEBUG_PRINTF("RX: TIMEOUT (%d us)\r\n");
#endif
        } else if (irqflags & IRQ_LORA_CDDONE_MASK) { // CDDONE
            // check if preamble symbol was detected
            if (irqflags & IRQ_LORA_CDDETD_MASK) {
                // switch to receiving (continuous)
                writeReg(RegOpMode, OPMODE_LORA_RX);
                // continue waiting
                return false;
            } else {
                // indicate timeout
                LMIC.dataLen = 0;
            }
        } else {
            // unexpected irq
            ASSERT(0);
        }

        // mask all LoRa IRQs
        writeReg(LORARegIrqFlagsMask, 0xFF);

        // clear LoRa IRQ flags
        writeReg(LORARegIrqFlags, 0xFF);
    }

    // radio operation completed
    return true;
}

static const oslmic_radio_interface_t sx127x_radio = {
    sx127x_init,
    sx127x_irq_process,
    sx127x_starttx,
    sx127x_startrx,
    sx127x_sleep,
    sx127x_cca,
    sx127x_cad,
    sx127x_cw,
    sx127x_generate_random
};
const oslmic_radio_interface_t *sx127x_interface(){
    return &sx127x_radio;
}

#endif
