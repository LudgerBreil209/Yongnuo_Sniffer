// Yongnuo RF-603 sniffer
// using an A7105 chip and a Arduino /pro/micro with 3.3V
// based on Flysky Tx Code by midelic on RCgroups.com
//
// created by RobotFreak & Seegel Systeme
// www.photofreak.de
//
// implemented the 560 protocol : Ludger Breil

#include "a7105.h"
#include "a7105_init.h" // register initialization, frequence table

// channel to use (see channel_table )
const uint8_t CHANNEL = 16; /* 1 - 16 */

const uint8_t RX_MODE_TRX = 1; // receive FOCUS & SHOOT commands from 603 in TRX mode
const uint8_t RX_MODE_TX = 0; // receive FLASH commands from 603 in TX mode or TRX mode & mounted on cam

const uint8_t RX_MODE = RX_MODE_TX;

// used DataID's by Yongnuo
const uint8_t id_1[4] = { 0x35, 0x99, 0x9A, 0x5A }; // id RF603 mode
const uint8_t id_2[4] = { 0x35, 0x99, 0x9A, 0x33 }; // id 560 mode
const uint8_t id_3[4] = { 0x22, 0x99, 0x9A, 0x33 }; // dito


// ??? 
// RF-603
//
// AIF = 0 = disable Auto IF exchange (d.h. F_RXLO = F_LO)
// 
const uint8_t TRX_CH_OFFSET = 2; // in TRX mode, different channels are used!
const uint8_t RX_CH_OFFSET = 1;  // Receiving one IF lower

// define the RX modes
enum RXMode {
  mode_603 = 0,   // RF-603 compatible mode (ID1 with two bytes payload)
  mode_560,       // 560 mode 
  };

enum EnhancedPhase {
  EnhancedPhase0 = 0,   // not in enhanced mode, waiting for switch command
  EnhancedPhase1,       // received switch command, using ID2 and 4 bytes payload
  EnhancedPhase2,       // using ID3, variable payload 
  };

const unsigned long _MODE_TIMEOUT_ = 500;

// SPI communication pins with A7105/PPM
const int SDI_pin = 11;
const int SCLK_pin = 13;
const int CS_pin = 10;
const int GIO2_pin = 2; // GIO2  pin

//
class LED {
  public:

  LED(int pin = LED_BUILTIN)
    : m_pin(pin)
    , m_duration(0)
    , m_start_millis(0)
    , m_on(false)
    {
    }

  void begin(void) {
    pinMode(m_pin, OUTPUT);
  }

  void loop(void) {
    if (m_on && ((millis() - m_start_millis) > m_duration)) {
      off();
    }
  }

  void on(unsigned long duration) {
    digitalWrite(m_pin, HIGH);
    m_duration = duration;
    m_start_millis = millis();
    m_on = true;
  }

  void off() {
    digitalWrite(m_pin, LOW);
    m_start_millis = 0;
    m_on = false;
  }
  
protected:
  bool m_on;
  int m_pin;
  unsigned long m_duration;
  unsigned long m_start_millis;
};

// ########## Variables #################
//
RXMode rx_mode = mode_603;
EnhancedPhase enhanced_phase = EnhancedPhase0;
// static uint8_t aid[4]; // for debug only
uint8_t in[64];     // input buffer, the A7105 FIFO is 64 byte long
volatile bool received_packet = false; // 
unsigned long start_560_mode = 0; // start ticks of 560-mode, needed for timeout 
unsigned long last_packet_time = 0;  // for debugging timing, time of last received packet
int dataID = 0;     // current ID set, for debugging output
bool received_parameter = false; // flag 
unsigned long timeout_560 = 500; // timeout for 560-mode 

LED led;

// 
//
// console input
static const char charCR = '\r';
static const char charLF = '\n';

char consoleBuffer[64];               // read buffer
unsigned short idxConsoleBuffer = 0;  // index of last valid character in consoleBuffer
unsigned long lastConsoleInput = 0;   // time stamp of last console input

CA7105 a7105(SDI_pin, SCLK_pin, CS_pin); // the A7105 object

//
// Grp A, Zoom = 80 mm; Mode = M
// pwr = 1/4/ 1/4; Multi = 3x6 Hz
uint8_t parameter[15] = { 0x30, 0xCF, 0x20, 0, 0x50, 0x02, 0, 0,
    0x14, 0x14, 0x03, 0x06, 0, 0, 0xa2 };

// set DataID
// id = 1, 2, 3 
void SetDataID(int id) {
  const uint8_t* p(nullptr);

  switch(id) {
    case 1 : p = id_1; break;
    case 2 : p = id_2; break;
    case 3 : p = id_3; break;
    default:
      Serial.println("wrong id identifier!");
    }

    if (p != nullptr) {
      a7105.WriteID(p);
      dataID = id;
      // Serial.print("set DataID ");
      // Serial.println(id);
    }
}

// send a command in RF603 mode
// cmd = command bytes (2) to send
// rep : repetition count
void send603Command(const uint8_t* cmd, int rep) {

  a7105.SetPacketLength(2);
  a7105.WriteID(id_1);
  a7105.WriteRegister(PLL1_REG, channel_table[CHANNEL - 1]);
  //_spi_write_adress(PLL1_REG, channel_table[CHANNEL - 1]);

  for (int c = 0; c < rep; c++) {
    a7105.WriteData(2, cmd);
    a7105.Strobe(A7105_PLL);
    a7105.Strobe(A7105_TX);
    delay(20);
  }
}

// commands in RF603-mode
void sendFlash() {

  const uint8_t data[2] = {0x88, 0x77};

  send603Command(data, 10);
  Serial.println("tx flash");
}

void sendShoot() {

  const uint8_t data[2] = {0x22, 0xdd};

  send603Command(data, 30);
  Serial.println("tx shoot");
}

void sendFocus() {

  const uint8_t data[2] = {0x11, 0xee};

  send603Command(data, 50);
  Serial.println("tx focus");
}

void sendWakeup() {

  const uint8_t data[2] = {0x44, 0xBB};

  send603Command(data, 10);
  Serial.println("tx wakeup");
}

// send parameter
// parameter = uint8_t[15] = parameter packet
void sendParameters(const uint8_t* parameter) {

  uint8_t data[4] = { 0x44, 0xBB, 0x0F, 0x0D };

  // using the YN560-TX sequence:
  // send packets only once with a gap of approx. 20 ms

  SetDataID(1);
  a7105.SetPacketLength(2);

  a7105.Strobe(A7105_STANDBY);
  a7105.WriteRegister(PLL1_REG, channel_table[CHANNEL - 1]); // PLL1_REG set frequency
  a7105.Strobe(A7105_PLL);

  // send wake-up (44 BB)
  a7105.WriteData(2, data);
  a7105.Strobe(A7105_TX);

  delay(20);

  // send switch cmd (32 CD)
  data[0] = 0x32; data[1] = 0xCD;
  a7105.WriteData(2, data);
  a7105.Strobe(A7105_TX);

  delay(20);

  // switch to ID2 and set packet length to 4
  // a7105.Strobe(A7105_STANDBY);
  SetDataID(2);
  a7105.SetPacketLength(4);

  // send parameter command (32 CD 0F 0D)
  a7105.WriteData(4, data);
  a7105.Strobe(A7105_TX);

  delay(20);

  // switch to ID3 and set packet length 15
  SetDataID(3);
  a7105.SetPacketLength(15);

  // send parameters
  a7105.WriteData(15, parameter);
  a7105.Strobe(A7105_TX);

  delay(20);

  // send switch command (32 CD)
  a7105.SetPacketLength(2);
  a7105.WriteData(2, data);
  a7105.Strobe(A7105_TX);

  // switch back to iD1
  a7105.Strobe(A7105_STANDBY);
 
  SetDataID(1);

  received_packet = false; // clear flag
}

// starts the receiving mode
void startRx() {

  a7105.Strobe(A7105_STANDBY);

  // PLL1_REG set frequency
  if (RX_MODE == RX_MODE_TRX) {
    a7105.WriteRegister(PLL1_REG, channel_table[CHANNEL - 1] - RX_CH_OFFSET + TRX_CH_OFFSET);
  }
  else if (RX_MODE == RX_MODE_TX) {
    a7105.WriteRegister(PLL1_REG, channel_table[CHANNEL - 1] - RX_CH_OFFSET);
  }
 
  // a7105.Strobe(A7105_STANDBY);
  a7105.Strobe(A7105_RST_RDPTR);
  a7105.Strobe(A7105_RX);

  received_packet = false;

  // Serial.println("startRx");
}

// calculates and returns the crc for the parameter data
uint8_t calculate_crc(const uint8_t* data, int len) {
  uint8_t crc(0);

  for (int i = 0; i < len; i++) {
    crc += *data;
    data++;
  }
  return crc;
 }

// GIO2 is programmed for FSYNC (register GPIO2_REG)
// pin is high after recognising the preamble/ID and will go low
// after receiving the last bit of the payload
// -> a falling edge means: the A7105 had received a packet
// ATTN: this is also true while transmitting!
void GIO2_ISR() {

  received_packet = true;
}

// switch to RF603 mode
void switch_to_603_mode(void) {

  SetDataID(1);
  a7105.SetPacketLength(2);

  rx_mode = mode_603;
  enhanced_phase = EnhancedPhase0;
  timeout_560 = _MODE_TIMEOUT_;
  // Serial.println("switching to basic mode");
}

// process received data in RF603 modus
void process_data_603() {

  print_data(in, 2);

  // if received the switch command 0x32 0xCD
  // switch to enhanced mode
  if (in[0] == 0x32 && in[1] == 0xCD) {

    rx_mode = mode_560;
    
    enhanced_phase = EnhancedPhase1;
    SetDataID(2);
    a7105.SetPacketLength(4);
 
    start_560_mode = millis();
    // Serial.println("switching to enhanced mode");
  }
}

// debug output the current DataID
void print_id() {
   Serial.print("[");
   Serial.print(dataID);
   Serial.print("] : ");
}

// print out the power level
void print_power_level(uint8_t pwr) {

  const char* half_steps[4] = { "", "0.3", "0.5", "0.7" };
 
  uint8_t j = 8 - (pwr / 4) - 1;
  Serial.print("1/");
  Serial.print((1 << j));
  j = (pwr % 4);
  if (j != 0) {
    Serial.print("+");
    Serial.print(half_steps[j]);
  }
}


void print_group(uint8_t grp) {
  
  const static char grp_id[8] = {'-', 'A', 'b', 'c', 'd', 'E', 'F', '?' };

  // the group id is coded in bit [7..5]
  //  A = 20, b = 40, ...  if grp = 0 -> 0F is send
  // otherwise all other bits are 0
  grp = (grp & 0xE0) >> 5;

  Serial.print(grp_id[grp]);

  /*
  if (grp == 0x0F) Serial.print("- ");      // b000
  else if (grp == 0x20) Serial.print("A "); // b001
  else if (grp == 0x40) Serial.print("b "); // b010
  else if (grp == 0x60) Serial.print("c "); // b011
  else if (grp == 0x80) Serial.print("d "); // b100
  else if (grp == 0xA0) Serial.print("E "); // b101
  else if (grp == 0xC0) Serial.print("F "); // b110
  */
}

// print out the parameters (as long as i know them)
void decode_parameter(const uint8_t* data) {

  // [0], [1] : code = 30 CF

  Serial.print("  Grp: ");

  // [2] : group id
  print_group(data[2]);

  // [3] ?
  Serial.print(" / [3]: ");
  if ((data[3] & 0xF0) == 0) Serial.print("0");
  Serial.print(data[3], HEX);
  Serial.print(" / Z: ");

  // [4] : Zoom
  Serial.print(data[4]);
  Serial.print(" mm / Mode: ");

  // [5] : mode
  if (data[5] == 0) Serial.print("--");
  else if (data[5] == 1) Serial.print("Multi");
  else if (data[5] == 2) Serial.print("M");

  // [6], [7] ?
  Serial.print(" / [6]: ");
  if ((data[6] & 0xF0) == 0) Serial.print("0");
  Serial.print(data[6], HEX);
  Serial.print(" / [7]: ");
  if ((data[7] & 0xF0) == 0) Serial.print("0");
  Serial.print(data[7], HEX);

  // [8], [9] : power levels
  // M-mode
  Serial.print(" / P: ");
  print_power_level(data[8]);
  Serial.print(" - ");
  // Multi-mode
  print_power_level(data[9]);

  // [10], [11] : Multi-mode parameter
  Serial.print(" / m: ");
  Serial.print(data[10]);
  Serial.print("x");
  Serial.print(data[11]);
  Serial.print("Hz ");

  // [12], [13] ?
  Serial.print(" / [12]: ");
  if ((data[12] & 0xF0) == 0) Serial.print("0");
  Serial.print(data[12], HEX);
  Serial.print(" / [13]: ");
  if ((data[13] & 0xF0) == 0) Serial.print("0");
  Serial.print(data[13], HEX);
}

// process received data in 560 mode
void process_data_560() {

  // the YN560-TX send packets only once, with approx. 20 ms space
  // the YN560 IV sends every packet upto 5 times with 1-2 ms space
  if (enhanced_phase == EnhancedPhase1) {
 
    print_data(in, 4);
 
    // received the 0x32 0xCD command ?
    // bytes 2 and 3:
    // a) 0x0F 0x0D : send by the YN560TX before sending the channel parameters
    // b) 0x0F 0x0E : send by the YN560 IV  "        "           "
    // c) 0x29 0x28 : send by YN560 IV in ACT mode before sending the ACT parameters
    // d) 0x29 0x07 : send by YN560-TX  "
    if (in[0] == 0x32 && in[1] == 0xcd) {
      if (in[2] == 0x0f && (in[3] == 0x0d || in[3] == 0x0e)) {
        enhanced_phase = EnhancedPhase2;
        SetDataID(3);
        a7105.SetPacketLength(16);
      }
      else if (in[2] == 0x29 /*&& (in[3] == 0x28 || in[3] == 0x00)*/) { 
        // 
        enhanced_phase = EnhancedPhase2;
        SetDataID(3);
        a7105.SetPacketLength(41); // ACT mode : to do: verify packet length

        timeout_560 = 20 * _MODE_TIMEOUT_; // ACT packet is send every 1s
      }
    }

    // if at this point enhanced_phase is still ..Phase1, we received
    // an unknown command
    if (enhanced_phase == EnhancedPhase1) {
      switch_to_603_mode();
      Serial.println("unknown Phase1 command -> switching back to 603 mode");
    }
  }
  else if (enhanced_phase == EnhancedPhase2) {

    // switch command 0x32 0xCD ?
    // if yes, switch back to 306 mode
    if (in[0] == 0x32 && in[1] == 0xcd) {
      switch_to_603_mode();
      print_data(in, 2);
      received_parameter = false;
    }
    // channels parameters? starting with 0x30 0xCF
    else if (in[0] == 0x30 && in[1] == 0xcf) {
      // we received the parameters, check CRC
      if (!received_parameter) {

        const uint8_t crc = calculate_crc(in, 14);
      
        if (crc == in[14]) {
          print_data(in, 2);
          Serial.print(" ");
          decode_parameter(in);
        }
        else {
          Serial.print(" CRC failure");
        }
        received_parameter = true;
        // set the flag, that we received the parameters
        // the YN560 IV send the parameter package 5 times

        led.on(500);
      }
      else {
        // only for debugging: indicate, that we received another parameter packet
        print_data(in, 3);
        Serial.print(" ... ");
      }
    }
    else if (in[0] == 0xA1 && in[1] == 0x5E) {
      Serial.print(" ACT ");
      // print_data(in, 41);
      
      // switch_to_603_mode();
    }
    else {
      // unknown packet id, output first two bytes of packet
      print_data(in, 2);
    }
  }
}

//

// console input

// HandleConsole
//
// Eingabe über console verarbeiten
// jede Eingabe muss mit 'CR + LF' abgeschlossen werden
void HandleConsole(void) {

  while (Serial.available() > 0) {
    char c = Serial.read();

    /*
    if (c == '\r')
      Serial.print("<CR>");
    else if (c == '\n')
      Serial.print("<LF>");
    else 
      Serial.print(c);
    */
    
    if (c == charCR) {
      // eingabe mit 0 Zeichen abschliessen, vereinfacht die Verarbeitung
      consoleBuffer[idxConsoleBuffer] = '\0';
      ProcessConsoleInput();

      ResetConsoleInput();
    }
    else if (c == charLF) {
      // ignore <LF> characters ?
      lastConsoleInput = millis();
    }
    else {
      // pufferueberlauf verhindern 
      // -> sizeof()-1, da letztes Zeichen ein 0-Zeichen sein muss
      if (idxConsoleBuffer >= sizeof(consoleBuffer) - 1) {
        ResetConsoleInput();
      }
      else {
        consoleBuffer[idxConsoleBuffer++] = c;
      }

      lastConsoleInput = millis();
    }
  }

  // timeout -> reset
  if ((idxConsoleBuffer != 0) && ((millis() - lastConsoleInput) > 5000)) {
    ResetConsoleInput();
  }


}

void ResetConsoleInput(void) {
   memset(consoleBuffer, 0, sizeof(consoleBuffer));
  idxConsoleBuffer = 0;
}

// true falls Befehl abgearbeitet wurde
bool ProcessConsoleInput(void) {
  bool rtn(false);
  char s[64];

  /*
  Serial.print("ProcessConsoleInput \'");
  Serial.print(consoleBuffer);
  Serial.println("\'");
  */
   if (strlen(consoleBuffer) == 1) {
    // 'f'
    if (consoleBuffer[0] == 'f') {
      sendFlash();
      return true;
    }
    // 's'
    else if (consoleBuffer[0] == 's') {
      sendShoot();
      return true;
    }
    // 'l'
    else if (consoleBuffer[0] == 'l') {
      sendFlash();
      return true;
    }
  }
  // 'p=xx<CR>' 'p+<CR>' 'p-<CR>' set power
  else if ((strlen(consoleBuffer) >= 2) && (consoleBuffer[0] == 'p')) { 
    // power in M mode
    uint8_t newValue = parameter[8];
 
    if (consoleBuffer[1] == '=') { // auf 'p=value' testen
      int set(0);
      int iItems = sscanf(consoleBuffer + 2, "%d", &set);
      if ((iItems == 1) && ((set >= 0)  && (set <= 28))) {
        // set pwr
        newValue = (uint8_t)set; // power in M mode
      }
    }
    else if (consoleBuffer[1] == '+') { // auf 'p+' testen
      if (parameter[8] < 28) newValue = parameter[8] + 1;
    }
    else if (consoleBuffer[1] == '-') { // auf 'p+' testen
      if (parameter[8] > 0) newValue = parameter[8] - 1;
    }
    if (newValue != parameter[8]) {
      // set pwr
      parameter[8] = newValue; // power in M mode
      parameter[14] = calculate_crc(parameter, 14);

      sendParameters(parameter);
    }
  }
   // 'z=xx<CR>' 'z+<CR>' 'z-<CR>' set zoom
  else if ((strlen(consoleBuffer) >= 2) && (consoleBuffer[0] == 'z')) {

    static const uint8_t zoom_level[7] = { 24, 28, 35, 50, 70, 80, 105 };

    int oldIdx(0);
    int newIdx(0);

    for (oldIdx = 0; oldIdx < 7; oldIdx++)
    {
      if (zoom_level[oldIdx] == parameter[4]) break;  
    }
    newIdx = oldIdx;
    
    if (consoleBuffer[1] == '=') { // auf 'z=value' testen
      int set(0);
      int iItems = sscanf(consoleBuffer + 2, "%d", &set);

      for (newIdx = 0; newIdx < 7; newIdx++)
      {
        if (zoom_level[newIdx] == set) break;  
      }
      if (newIdx >= 7) {
       Serial.println("invalid zoom value");
       return false; 
      }
    }
    else if (consoleBuffer[1] == '+') { // auf 'z+' testen
      if (oldIdx < 6) newIdx = oldIdx + 1;
    }
    else if (consoleBuffer[1] == '-') { // auf 'z+' testen
      if (oldIdx > 0) newIdx = oldIdx - 1;
    }
    if (newIdx != oldIdx) {
      // set zoom
      parameter[4] = zoom_level[newIdx];
      parameter[14] = calculate_crc(parameter, 14);

      sendParameters(parameter);
    }
  }
  // 'g=x' mit x = '-', 'a', ... 'f'
  else if (strlen(consoleBuffer) == 3 && consoleBuffer[0] == 'g' && consoleBuffer[1] == '=') {
    uint8_t grp = 0;

    switch (consoleBuffer[2]) {
      case '-' : grp = 0x0F; break;
      case 'a' : grp = 0x20; break; // 0010
      case 'b' : grp = 0x40; break; // 0100
      case 'c' : grp = 0x60; break; // 0110
      case 'd' : grp = 0x80; break; // 1000
      case 'e' : grp = 0xA0; break; // 1010
      case 'f' : grp = 0xC0; break; // 1100
      default:break;
    }

    if (grp == 0){
      // unkown parameter
      Serial.println("invalid group parameter");
    }
    else {
      parameter[2] = grp;
      Serial.print("set group to ");
      print_group(grp);
      Serial.println(" ");
    }
  }
  else {
      Serial.println("invalid command");
    }
}

void setup() {
  
  Serial.begin(115200);
  
  led.begin();

  a7105.Init(A7105_regs);

  // a7105.DumpRegs();
  
  SetDataID(1);
 
  a7105.Calibrate();  // calibrate A7105

  a7105.Strobe(A7105_STANDBY);

  // falling edge marks end of the received packet
  // -> 
  received_packet = false;
  attachInterrupt(digitalPinToInterrupt(GIO2_pin), GIO2_ISR, FALLING);

  a7105.SetPacketLength(2);

  // start receiving mode
  startRx();

  ResetConsoleInput();
  
}

void loop() {

  led.loop();

  if (received_packet) {

    received_packet = false;

    a7105.Strobe(A7105_RST_RDPTR);
    a7105.ReadData(a7105.GetPacketLength(), in);

    startRx();

    // process data
    Serial.print((millis() - last_packet_time));
    Serial.print("ms ");
    print_id();
 
    if (rx_mode == mode_603) process_data_603();
    else if (rx_mode == mode_560) process_data_560();

    Serial.println(" ");

    memset(in, 0, sizeof(in));
    
    last_packet_time = millis();
    // restart rx-mode
    startRx();
  }
  else {
    HandleConsole();
    received_packet = false;
  }

  if (rx_mode == mode_560) {
    // timeout check, may be we missed a packet
    if ((millis() - start_560_mode) > timeout_560) {
      // timeout: switch back to 603 mode
      switch_to_603_mode();
      Serial.println("timeout -> switching back to 603 mode");
    }
  }
}

void print_data(const uint8_t* data, int len) {

  for (int i = 0; i < len; i++) {
    if ((data[i] & 0xf0) == 0) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
    }
}
