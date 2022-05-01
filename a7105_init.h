#ifndef A7105_INIT_H_
#define A7105_INIT_H_

// yongnuo specific register setup
// 0xff means do not set
//
// GIO2 is programmed as "packet received marker"

static const uint8_t A7105_regs[] = {
  0xFF, // MODE_REG           0x00
  0x42, // MODECTRL_REG       0x01 // DDPC = disable; ARSSI = enable; AIF (Auto IF exchange) = disable; CD = disable; FMS = FIFO mode; ADCM = disable
  0xFF, // CALIBRATION_REG    0x02
  2 - 1, // FIFO1_REG   0x03        // default packet size is 2
  0x00, // FIFO2_REG          0x04
  0x00, // FIFO_REG           0x05
  0x00, // IDCODE_REG         0x06
  0x00, // RCOSC1_REG         0x07
  0x00, // RCOSC2_REG         0x08
  0x00, // RCOSC3_REG         0x09
  0x00, // CKO_REG            0x0A  // ECKOE = disable; CKOS = DCK / RCK; CKOE = disable
  0x01, // GPIO1_REG          0x0B  // GIO1E = enable; GIO1L = non-inverted; GIO1 = WTR
// 0x21, // GPIO2_REG          0x0C // GIO2OE = High. Z; GIO2I = non inverted output; GIO2S = Preamble Detect Output
  0x05, // 000001 0 1        0x0C  // GIO2OE = enable; GIO2I = non inverted output; GIO2S = FSYNC (frame sync)
  0x05, // CLOCK_REG          0x0D  // CGS = disable; XS = Crystal; CSC = 01b =F_MCLK/2;
  0x00, // DATARATE_REG       0x0E  // SDR = 0
  0x50, // PLL1_REG           0x0F  // CHN = 16
  0x9E, // PLL2_REG           0x10  // (IP8 = 0;) CHR = 16; RRC = 0; DBL = 1;
  0x4B, // PLL3_REG           0x11  // IP = x4B
  0x00, // PLL4_REG           0x12
  0x02, // PLL5_REG           0x13  // BFP = 0x0002
  0x16, // TX1_REG            0x14
  0x2B, // TX2_REG            0x15
  0x12, // DELAY1_REG         0x16  // PDL = b010; TDL = b10; DPR = b000
  0x00, // DELAY2_REG         0x17
  0x62, // RX_REG             0x18  // ULS = 0 (upside band); BWS = 1 ( = 500kHz bandwidth F_IF = 500kHz)
  0x80, // RXGAIN1_REG        0x19
  0x80, // RXGAIN2_REG        0x1A
  0x00, // RXGAIN3_REG        0x1B
  0x0A, // RXGAIN4_REG        0x1C
  0x32, // RSSI_REG           0x1D
  0xC3, // ADC_REG            0x1E
  0x07, // CODE1_REG          0x1F  // WHTS = disable; FECS = disable; CRCS = disable; IDL = 4 byte; PML = 4 byte
  0x16, // CODE2_REG          0x20  // PMD = 8bits; ETH = 1bit; DCL = 
  0x00, // CODE3_REG          0x21
  0x00, // IFCAL1_REG         0x22
  0x00, // IFCAL2_REG         0x23
  0x00, // VCOCCAL_REG        0x24
  0x00, // VCOCAL1_REG        0x25
  0x3B, // VCOCAL2_REG        0x26
  0x00, // BATTERY_REG        0x27
  0x17, // TXTEST_REG         0x28  // TBG = 7; PAC = 2; TXCS = 0 -> 0.1dBm output power
  0x47, // RXDEM1_REG         0x29
  0x80, // RXDEM2_REG         0x2A
  0x03, // CPC_REG            0x2B
  0x01, // CRYSTALTEST_REG    0x2C
  0x45, // PLLTEST_REG        0x2D
  0x18, // VCOTEST1_REG       0x2E
  0x00, // VCOTEST2_REG       0x2F
  0x01, // IFAT_REG           0x30
  0x0F, // RSCALE_REG         0x31
  0x00, // FILTERTEST_REG     0x32
};


// these frequency/channel values are specific to the yongnuo setup
// the correspond to the 16 channels used by the yongnuo devices
// 
// they are written in PLL1_REG (0x0f) to set F_OFFSET
// F_LO_BASE = 2400,001 MHz, F_CHSP = 500 kHz
// F_LO = F_LO_BASE + CHN * F_CHSP
// SDR = 0 -> F_SYNK / 32 / 1 = 500 kBaud/s
const uint8_t channel_table[16] = {
  0x71, 0x6B, 0x65, 0x59, 0x53, 0x4D, 0x41, 0x3B, 0x35, 0x29, 0x23, 0x1D, 0x17, 0x11, 0x0B, 0x05
};

#endif
