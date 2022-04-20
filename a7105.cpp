/******************************************************* 
 *  A7102REG.h
* RF Chip-A7102 Hardware Definitions
*
* * This file provides the constants associated with the
* AMICCOM A7102 device.
*
********************************************************************/

#include "Arduino.h"
#include "a7105.h"

//---------------------------------
// see comment in a7105.h

#ifdef DIRECT_PORT_ACCESS
  // the direct way
  #define SCK_on (Port_SCLK->PIO_SODR = PortMask_SCLK)
  #define SCK_off (Port_SCLK->PIO_CODR = PortMask_SCLK)

  #define SDI_on (Port_SDI->PIO_SODR = PortMask_SDI)
  #define SDI_off (Port_SDI->PIO_CODR = PortMask_SDI)

  #define CS_on (Port_CS->PIO_SODR = PortMask_CS)
  #define CS_off (Port_CS->PIO_CODR = PortMask_CS)

#else
  // the compatible way
  #define SCK_on digitalWrite(SCLK_pin, HIGH)
  #define SCK_off digitalWrite(SCLK_pin, LOW)

  #define SDI_on digitalWrite(SDI_pin, HIGH)
  #define SDI_off digitalWrite(SDI_pin, LOW)

  #define CS_on digitalWrite(CS_pin, HIGH)
  #define CS_off digitalWrite(CS_pin, LOW)

#endif

#define NOP() __asm__ __volatile__("nop")


CA7105::CA7105(int SDIpin, int SCLKpin, int CSpin)
  : m_packet_length(2)
  , CS_pin(CSpin)
  , SCLK_pin(SCLKpin)
  , SDI_pin(SDIpin)
#ifdef DIRECT_PORT_ACCESS
  , Port_SCLK(nullptr)
  , Port_SDI(nullptr)
  , Port_CS(nullptr)
  , PortMask_SCLK(0)
  , PortMask_SDI(0)
  , PortMask_CS(0)
#endif
{
}

bool CA7105::Init(const uint8_t* reg_init) {

  // configure and initialize SPI pins
  // RF module pins
  pinMode(SDI_pin, OUTPUT);   // SDI   SDIO
  pinMode(SCLK_pin, OUTPUT);  // SCLK SCL
  pinMode(CS_pin, OUTPUT);    // CS output

#ifdef DIRECT_PORT_ACCESS

  Port_SCLK = g_APinDescription[SCLK_pin].pPort; 
  PortMask_SCLK = g_APinDescription[SCLK_pin].ulPin;

  Port_SDI = g_APinDescription[SDI_pin].pPort; 
  PortMask_SDI = g_APinDescription[SDI_pin].ulPin;

  Port_CS = g_APinDescription[CS_pin].pPort; 
  PortMask_CS = g_APinDescription[CS_pin].ulPin;

#endif
  
  CS_on;    // SC high
  SDI_on;
  SCK_off;

  //
  delay(10); // wait 10ms for A7105 wakeup
  Reset(); // reset A7105
  delay(100);

  InitRegs(reg_init);

  return true;
}


//--------------------------------------
void CA7105::Reset(void) {
  _spi_write_adress(MODE_REG, 0x00);
  // Serial.println("A7105_reset");
}

//--------------------------------------
void CA7105::WriteID(const uint8_t *ida) {

  CS_off;
  _spi_write(IDCODE_REG);
  for (int i = 0; i < 4; i++) {
    _spi_write(*ida++);
  }
  CS_on;
}

//--------------------------------------
void CA7105::ReadID(uint8_t* ida) {

  CS_off;
  _spi_write((IDCODE_REG | ADDRESS_READ));
  for (int i = 0; i < 4; i++) {
    ida[i] = _spi_read();
  }
  CS_on;
}

//--------------------------------------
void CA7105::InitRegs(const uint8_t* reg_init) {

  for (int i = 0; i < 0x33; i++) {

    if (reg_init[i] != 0xff)
      _spi_write_adress(i, reg_init[i]);
  }
  m_packet_length = reg_init[FIFO1_REG] + 1;

  // Serial.println("A7105_InitRegs");
}

void CA7105::DumpRegs(void) {

  uint8_t r;
  for (int i = 0; i < 0x33; i++) {
    r = _spi_read_adress(i);
    Serial.print(r, HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

//--------------------------------------
void CA7105::Calibrate(void) {
  uint8_t if_calibration1;
  uint8_t vco_calibration0;
  uint8_t vco_calibration1;

  _spi_strobe(0xA0);//stand-by
  _spi_write_adress(0x02, 0x01);
  while (_spi_read_adress(0x02)) {
    if_calibration1 = _spi_read_adress(0x22);
    if (if_calibration1 & 0x10) { //do nothing
    }
  }
  _spi_write_adress(0x24, 0x13);
  _spi_write_adress(0x26, 0x3b);
  _spi_write_adress(0x0F, 0x00); //channel 0
  _spi_write_adress(0x02, 0x02);
  while (_spi_read_adress(0x02)) {
    vco_calibration0 = _spi_read_adress(0x25);
    if (vco_calibration0 & 0x08) { //do nothing
    }
  }
  _spi_write_adress(0x0F, 0xA0);
  _spi_write_adress(0x02, 0x02);
  while (_spi_read_adress(0x02)) {
    vco_calibration1 = _spi_read_adress(0x25);
    if (vco_calibration1 & 0x08) { //do nothing
    }
  }
  _spi_write_adress(0x25, 0x08);
  _spi_write_adress(0x28, 0x1F); //set power to 1db maximum
  _spi_strobe(0xA0);//stand-by strobe command
  delay(100);
  _spi_strobe(A7105_RST_WRPTR);
  _spi_write_adress(0x0F, 0x70);
  _spi_strobe(A7105_STANDBY);
  _spi_strobe(A7105_RST_RDPTR);
  _spi_strobe(A7105_RX);
}

//--------------------------------------
void CA7105::ReadData(int len, uint8_t *data) {

  CS_off;
  _spi_write((FIFO_REG | ADDRESS_READ));
  for (int i = 0; i < len; i++) {
    data[i] = _spi_read();
  }
  CS_on;
}

//--------------------------------------
void CA7105::WriteData(int len, const uint8_t* data) {

  CS_off;
  _spi_write(FIFO_REG);
  for (int i = 0; i < len; i++) {
    _spi_write(data[i]);
  }
  CS_on;
}

void CA7105::SetPacketLength(uint8_t len) {
  _spi_write_adress(FIFO1_REG, len-1);
  m_packet_length = len;
}

uint8_t CA7105::GetPacketLength(void) const {
   return m_packet_length;
 }

void CA7105::Strobe(uint8_t command) {

  _spi_strobe(command);
}

void CA7105::WriteRegister(uint8_t addr, uint8_t value) {
   _spi_write_adress(addr, value);
}

uint8_t CA7105::ReadRegister(uint8_t addr) {
 return _spi_read_adress(addr);
}

// /////
//
// own SDI handling

//--------------------------------------
// _spi_write
//  write command to SPI 
void CA7105::_spi_write(uint8_t command) {
  uint8_t n = 8;
  SCK_off;
  SDI_off;
  while (n--) {
    if (command & 0x80)
      SDI_on;
    else
      SDI_off;
    SCK_on;
    NOP();
    SCK_off;
    command = command << 1;
  }
  SDI_on;
}


#ifdef DIRECT_PORT_ACCESS
  #define SDI_1 ((Port_SDI->PIO_PDSR & PortMask_SDI) != 0)
#else
  #define SDI_1 (digitalRead(SDI_pin) == HIGH)
#endif

//-----------------------------------------
// _spi_read
//  read one byte from SDI
// returns byte
uint8_t CA7105::_spi_read(void) {
  uint8_t result(0);

  pinMode(SDI_pin, INPUT); // make SDIO pin input
  // SDI_on;
  for (int i = 0; i < 8; i++) {
    result = result << 1;
    if (SDI_1) result |= 0x01;
    SCK_on;
    NOP();
    SCK_off;
    NOP();
  }
  pinMode(SDI_pin, OUTPUT); // make SDIO pin output again
  return result;
}

//--------------------------------------
//
// _spi_write_adress
//  wrute data to adress (aka data register) address
void CA7105::_spi_write_adress(uint8_t address, uint8_t data) {
  CS_off;
  _spi_write(address);
  NOP();
  _spi_write(data);
  CS_on;
}

//--------------------------------------------
uint8_t CA7105::_spi_read_adress(uint8_t address) {
  uint8_t result(0);
  CS_off;
  address |= 0x40;  // | ADDRESS_READ
  _spi_write(address);
  result = _spi_read();
  CS_on;
  return (result);
}

//------------------------
void CA7105::_spi_strobe(uint8_t address) {
  CS_off;
  _spi_write(address);
  CS_on;
}
