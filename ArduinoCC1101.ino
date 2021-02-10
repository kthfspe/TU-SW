
#include <SPI.h>
//#include "cc1101.h"
#define wait_Miso()  while(digitalRead(12)>0)
#define CC1101_MARCSTATE         0x35
#define READ_SINGLE              0x80
#define CONFIG_REGISTER          0x80
#define STATUS_REGISTER          0xC0
#define CC1101_TXBYTES           0x3A        // Underflow and Number of Bytes
#define TX_FIFO                  0x3F
#define STX                      0x35
// CONFIGURATION REGISTERS
#define IOCFG2         0x00        //GDO2 output pin configuration
#define IOCFG1         0x01        //GDO1 output pin configuration
#define IOCFG0         0x02        //GDO0 output pin configuration
#define FIFOTHR        0x03        //RX FIFO and TX FIFO thresholds
#define SYNC1          0x04        //Sync word, high byte
#define SYNC0          0x05        //Sync word, low byte
#define PKTLEN         0x06        //Packet length
#define PKTCTRL1       0x07        //Packet automation control
#define PKTCTRL0       0x08        //Packet automation control
#define ADDR           0x09        //Device address
#define CHANNR         0x0A        //Channel number
#define FSCTRL1        0x0B        //Frequency synthesizer control
#define FSCTRL0        0x0C        //Frequency synthesizer control
#define FREQ2          0x0D        //Frequency control word, high byte
#define FREQ1          0x0E        //Frequency control word, middle byte
#define FREQ0          0x0F        //Frequency control word, low byte
#define MDMCFG4        0x10        //Modem configuration
#define MDMCFG3        0x11        //Modem configuration
#define MDMCFG2        0x12        //Modem configuration
#define MDMCFG1        0x13        //Modem configuration
#define MDMCFG0        0x14        //Modem configuration
#define DEVIATN        0x15        //Modem deviation setting
#define MCSM2          0x16        //Main Radio Control State Machine configuration
#define MCSM1          0x17        //Main Radio Control State Machine configuration
#define MCSM0          0x18        //Main Radio Control State Machine configuration
#define FOCCFG         0x19        //Frequency Offset Compensation configuration
#define BSCFG          0x1A        //Bit Synchronization configuration
#define AGCCTRL2       0x1B        //AGC control
#define AGCCTRL1       0x1C        //AGC control
#define AGCCTRL0       0x1D        //AGC control
#define WOREVT1        0x1E        //High byte Event 0 timeout
#define WOREVT0        0x1F        //Low byte Event 0 timeout
#define WORCTRL        0x20        //Wake On Radio control
#define FREND1         0x21        //Front end RX configuration
#define FREND0         0x22        //Front end TX configuration
#define FSCAL3         0x23        //Frequency synthesizer calibration
#define FSCAL2         0x24        //Frequency synthesizer calibration
#define FSCAL1         0x25        //Frequency synthesizer calibration
#define FSCAL0         0x26        //Frequency synthesizer calibration
#define RCCTRL1        0x27        //RC oscillator configuration
#define RCCTRL0        0x28        //RC oscillator configuration
#define FSTEST         0x29        //Frequency synthesizer calibration control
#define PTEST          0x2A        //Production test
#define AGCTEST        0x2B        //AGC test
#define TEST2          0x2C        //Various test settings
#define TEST1          0x2D        //Various test settings
#define TEST0          0x2E        //Various test settings

byte value;
byte value2;
byte message = 74;
char datachar = message;

byte strobe = 0x00;


byte TXBYTES = 0x3A; // Number of bytes
void setup() {
  pinMode(10, OUTPUT); // set the SS pin as an output
  pinMode(8, INPUT);
  pinMode(7, INPUT);
  pinMode(6, INPUT);
  pinMode(5, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  SPI.begin();     // initialize the SPI library
  Serial.begin(9600);
  Serial.println("Serial Monitor Engaged");
  //byte regAddrSend = ;
  //byte value = ;
  delay(100);
  // Startup settings
  halRfWriteReg(IOCFG2,0x0B);    //GDO2 Output Pin Configuration
  halRfWriteReg(IOCFG0,0x0C);    //GDO0 Output Pin Configuration
  halRfWriteReg(FIFOTHR,0x47);   //RX FIFO and TX FIFO Thresholds
  halRfWriteReg(PKTCTRL0,0x12);  //Packet Automation Control
  halRfWriteReg(FSCTRL1,0x06);   //Frequency Synthesizer Control
  halRfWriteReg(FREQ2,0x21);     //Frequency Control Word, High Byte
  halRfWriteReg(FREQ1,0x62);     //Frequency Control Word, Middle Byte
  halRfWriteReg(FREQ0,0x76);     //Frequency Control Word, Low Byte
  halRfWriteReg(MDMCFG4,0xF5);   //Modem Configuration
  halRfWriteReg(MDMCFG3,0x83);   //Modem Configuration
  halRfWriteReg(MDMCFG2,0x10);   //Modem Configuration
  halRfWriteReg(DEVIATN,0x15);   //Modem Deviation Setting
  halRfWriteReg(MCSM0,0x18);     //Main Radio Control State Machine Configuration
  halRfWriteReg(FOCCFG,0x16);    //Frequency Offset Compensation Configuration
  halRfWriteReg(WORCTRL,0xFB);   //Wake On Radio Control
  halRfWriteReg(FSCAL3,0xE9);    //Frequency Synthesizer Calibration
  halRfWriteReg(FSCAL2,0x2A);    //Frequency Synthesizer Calibration
  halRfWriteReg(FSCAL1,0x00);    //Frequency Synthesizer Calibration
  halRfWriteReg(FSCAL0,0x1F);    //Frequency Synthesizer Calibration
  halRfWriteReg(TEST2,0x81);     //Various Test Settings
  halRfWriteReg(TEST1,0x35);     //Various Test Settings
  halRfWriteReg(TEST0,0x09);     //Various Test Settings
  

 
  

}
// WRITE TO REGISTER
void halRfWriteReg(byte regAddr, byte valuewrite) {
    byte addr;
    addr = regAddr | 0x00;            // | 0x00 Works for R/W=0(write)/burst=0  
    digitalWrite(10, LOW);            // set the SS pin to LOW
    wait_Miso();                          // Wait until MISO goes low
    SPI.transfer(addr);                   // Send register address
    SPI.transfer(valuewrite);                  // Send value
    digitalWrite(10, HIGH);                     // set the SS pin to HIGH
}
// SEND COMMAND STROBE (FIFO_BYTES_AVAILABLE CASE 1)
void command_strobe1(byte cmd){
    cmd = cmd | 0x00;                // | 0x00 means that R/W=0(FIFO_BYTES_AVAILABLE means the TXFIFO when status byte has been sent)/ burst=0(strobe)
    digitalWrite(10, LOW);            // set the SS pin to LOW
    wait_Miso();                          // Wait until MISO goes low
    SPI.transfer(cmd);                   // Send register address
    digitalWrite(10, HIGH); 
  }

void sendbyte(byte data){
    digitalWrite(10, LOW);                // set the SS pin to LOW
    wait_Miso();                          // Wait until MISO goes low
    SPI.transfer(TX_FIFO);                // Send length of data (1 byte)
    SPI.transfer(1);
    SPI.transfer(TX_FIFO);                // Send the data
    SPI.transfer(data);                  // Send data
    
    delay(100);
    digitalWrite(10, HIGH);
}

uint8_t readReg(byte regAddr, byte regType) {
    byte addr, val;
    Serial.print("INPUT TO READREG:  ");
    Serial.print("ADDRESS: ");
    Serial.print(regAddr,HEX);
    Serial.print("   TYPE: ");
    Serial.println(regType,HEX);
    addr = regAddr | regType;
    digitalWrite(10, LOW);                      // set the SS pin to LOW
    wait_Miso();                          // Wait until MISO goes low
    SPI.transfer(addr);                   // Send register address
    val = SPI.transfer(0x00);             // Read result (need to send dummy message)
    digitalWrite(10, HIGH);                    // set the SS pin to HIGH
  
    return val;
}

void loop() {


  // SWITCH TO TX STATE
  if (digitalRead(7) == HIGH) {
    command_strobe1(STX);                   
    Serial.println("Gone to tx");
    delay(1000);
  }
  // READ THE CURRENT STATE
  if (digitalRead(8) == HIGH) {
    value = readReg(CC1101_MARCSTATE, STATUS_REGISTER);
    Serial.print("Current State:");
    Serial.print(value);
    Serial.print("   In Binary:");
    Serial.println(value,BIN);
    delay(1000);
  }
  // SEND 1 BYTE DATA TO TX REGISTER
  if (digitalRead(6) == HIGH) {
    sendbyte(message);
    Serial.print("Data Sent to FIFO_TX:");
    Serial.print(message,DEC );
    Serial.print("   ASSCI Character:" );
    Serial.println(datachar);
    delay(100);
  }

  // READ THE AMOUNT OF BYTES AVAILABLE IN THE TX REGISTER TO MAKE SURE THE DATA HAS BEEN SENT
  if (digitalRead(5) == HIGH) {
    value = readReg(CC1101_TXBYTES, STATUS_REGISTER);
    Serial.print("Current Amount of Bytes in FIFOTX:");
    Serial.println(value);
    delay(1000);
  }
    // READ SETTING
    if (digitalRead(3) == HIGH) {
    value = readReg(AGCCTRL2, CONFIG_REGISTER);
    Serial.print("Current Value in Register: ");
    Serial.println(value);
    delay(1000);
  }
   if (digitalRead(2) == HIGH) {
    value = readReg(FSCAL3, CONFIG_REGISTER);
    Serial.print("Current Value in RegisterAGC: ");
    Serial.println(value,HEX);
    delay(1000);
  }
  
}
