/******************************************************* 
 *  A7105.h
* RF Chip-A7105 Hardware 
*
* The class CA7105 for interfacing the A7105 chip
*
*******************************************************/
#ifndef _A7105_H_INCLUDE_
#define _A7105_H_INCLUDE_

// the SPI communication is done via the 3 user defined pins
// SDIpin, SCLKpin and CSpin
// to be compatible with all boards the software uses the functions 
// digitalRead() / digitalWrite() to access these pins.
// These functions are compatible with all boards, but may be for some
// boards too slow. If you encountered timing problems, you can access
// the pins directly via the appropiated ports. This is faster, but highly
// hardware specific!
// You must define DIRECT_PORT_ACCESS and rewrite these parts (mainly
// the _spi_foo functions) of the code according to the hardware you use!

// #define DIRECT_PORT_ACCESS

// define the address/register 
#define MODE_REG           0x00
#define MODECTRL_REG       0x01
#define CALIBRATION_REG    0x02
#define FIFO1_REG          0x03
#define FIFO2_REG          0x04
#define FIFO_REG           0x05
#define IDCODE_REG         0x06
#define RCOSC1_REG         0x07
#define RCOSC2_REG         0x08
#define RCOSC3_REG         0x09
#define CKO_REG            0x0A
#define GPIO1_REG          0x0B
#define GPIO2_REG          0x0C
#define CLOCK_REG          0x0D
#define DATARATE_REG       0x0E
#define PLL1_REG           0x0F
#define PLL2_REG           0x10
#define PLL3_REG           0x11
#define PLL4_REG           0x12
#define PLL5_REG           0x13
#define TX1_REG            0x14
#define TX2_REG            0x15
#define DELAY1_REG         0x16
#define DELAY2_REG         0x17
#define RX_REG             0x18
#define RXGAIN1_REG        0x19
#define RXGAIN2_REG        0x1A
#define RXGAIN3_REG        0x1B
#define RXGAIN4_REG        0x1C
#define RSSI_REG           0x1D
#define ADC_REG            0x1E
#define CODE1_REG          0x1F
#define CODE2_REG          0x20
#define CODE3_REG          0x21
#define IFCAL1_REG         0x22
#define IFCAL2_REG         0x23
#define VCOCCAL_REG        0x24
#define VCOCAL1_REG        0x25
#define VCOCAL2_REG        0x26
#define BATTERY_REG        0x27
#define TXTEST_REG         0x28
#define RXDEM1_REG         0x29
#define RXDEM2_REG         0x2A
#define CPC_REG            0x2B
#define CRYSTALTEST_REG    0x2C
#define PLLTEST_REG        0x2D
#define VCOTEST1_REG       0x2E
#define VCOTEST2_REG       0x2F
#define IFAT_REG           0x30
#define RSCALE_REG         0x31
#define FILTERTEST_REG     0x32

#define ADDRESS_READ      0x40 // set R/W bit: (FOO_REG | REGISTER_READ) for reading data register
#define ADDRESS_STROBE    0x80 // set CMD bit for strobe command

//strobe commands
enum A7105_State {
  A7105_SLEEP     = 0x80,   //1000,xxxx SLEEP mode
  A7105_IDLE      = 0x90,   //1001,xxxx IDLE mode
  A7105_STANDBY   = 0xA0,   //1010,xxxx Standby mode
  A7105_PLL       = 0xB0,   //1011,xxxx PLL mode
  A7105_RX        = 0xC0,   //1100,xxxx RX mode
  A7105_TX        = 0xD0,   //1101,xxxx TX mode
  A7105_RST_WRPTR = 0xE0,   //1110,xxxx TX FIFO reset
  A7105_RST_RDPTR = 0xF0,   //1111,xxxx RX FIFO reset
};


class CA7105
{
public:

  CA7105(int SDIpin, int SCLKpin, int CSpin);

  bool Init(const uint8_t* reg_init);

  void Reset(void);

  void InitRegs(const uint8_t* reg_init);
  void DumpRegs(void);

  void WriteRegister(uint8_t addr, uint8_t value);
  uint8_t ReadRegister(uint8_t addr);

  void WriteID(const uint8_t* data);
  void ReadID(uint8_t* ida);

  void ReadData(int len, uint8_t *data);
  void WriteData(int len, const uint8_t* data);

  void SetPacketLength(uint8_t len);
  uint8_t GetPacketLength(void) const;

  void Calibrate(void);

  void Strobe(uint8_t command);

protected:
  void _spi_write(uint8_t command);
  uint8_t _spi_read(void);
  void _spi_write_adress(uint8_t address, uint8_t data);
  uint8_t _spi_read_adress(uint8_t address);
  void _spi_strobe(uint8_t address);

protected:

  int CS_pin;
  int SCLK_pin;
  int SDI_pin;

  uint8_t m_packet_length;

#ifdef DIRECT_PORT_ACCESS
  // needed for direct access of pins
  Pio* Port_SCLK;
  Pio* Port_SDI; 
  Pio* Port_CS;
  unsigned int PortMask_SCLK;
  unsigned int PortMask_SDI;
  unsigned int PortMask_CS;
#endif
};


#endif // _A7105_H_INCLUDE_
