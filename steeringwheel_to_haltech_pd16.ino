#include <mcp_can.h>  // library for MCP2515 ic
#include <SPI.h>      // library for SPI communication

long unsigned int rxId;  // storage for can data
unsigned char len = 0;   // storage for can data
unsigned char rxBuf[8];  // storage for can data

#define CAN0_INT 3  // Set INT to pin 2
#define CAN1_INT 2
MCP_CAN CAN0(10);   // set CS pin to 10r
MCP_CAN CAN1(9); 


byte out1 = 14;
byte out2 = 15;
byte out3 = 16;



int scaledvalue1 = 0;             // storage for 12 bit analog value
int scaledvalue2 = 0;             // storage for 12 bit analog value
int scaledvalue3 = 0;             // storage for 12 bit analog value
int scaledvalue4 = 0;             // storage for 12 bit analog value


bool blinkl = 0;
bool blinkr = 0;
bool grootp = 0;
bool groot  = 0;
bool parkl  = 0;
bool parkr  = 0;

bool wiper_pul   = 0;
bool wiper_int   = 0;
bool wiper_st1   = 0;
bool wiper_st2   = 0;
byte wiper_stage = 0;
byte wiper_sta   = 0;


unsigned long task1Interval = 50;              // 50ms interval for keep aliv frame
unsigned long task2Interval = 20;      // 30ms interval for button info frame
unsigned long KAintervalMillis = 0;         // storage for millis counter
unsigned long ButtonInfoIntervalMillis = 0; // storage for millis counter

unsigned long task1Millis = 0;    // storage for millis counter
unsigned long task2Millis = 0;    // storage for millis counter
unsigned long task3Millis = 0;    // storage for millis counter
unsigned long task4Millis = 0;    // storage for millis counter



void setup() {
  // start serial port an send a message with delay for starting
  Serial.begin(115200);
  Serial.println("Haltech 3x5 keypad ID B emulator");
  delay(50);

  // initialize canbus with 1000kbit and 16mhz xtal
  if (CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 CAN0 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 CAN0...");

    // initialize canbus with 1000kbit and 16mhz xtal
  if (CAN1.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 CAN1 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 CAN1...");

  // Set operation mode to normal so the MCP2515 sends acks to received data.
  CAN0.setMode(MCP_NORMAL);
  CAN1.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT);      // set INT pin to be an input
  digitalWrite(CAN0_INT, HIGH);  // set INT pin high to enable interna pullup

  pinMode(CAN1_INT, INPUT);      // set INT pin to be an input
  digitalWrite(CAN1_INT, HIGH);  // set INT pin high to enable interna pullup

  pinMode(4, INPUT);
  digitalWrite(4, HIGH);

  pinMode(out1, OUTPUT);
  pinMode(out2, OUTPUT);
  pinMode(out3, OUTPUT);

  Serial.println("All OK");  // all ready to go !
}


void loop() {

  unsigned long currentMillis = millis();  // Get current time in milliseconds

  if (currentMillis - task1Millis >= task1Interval) {
    task1Millis = currentMillis;
    SendKeepAlive();
  }

    if (currentMillis - task2Millis >= task2Interval) {
    task2Millis = currentMillis;
    SendAnalogValues();
    SendSpi();
    }

  // read can buffer when interrupted and jump to canread for processing.
  if (!digitalRead(CAN0_INT))  // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);  // Read data: len = data length, buf = data byte(s)
    digitalWrite(out1, HIGH);
    canRead(); // execute canRead function to negotiate with ecu
    digitalWrite(out1, LOW);
  }
  
    // read can buffer when interrupted and jump to canread for processing.
  if (!digitalRead(CAN1_INT))  // If CAN0_INT pin is low, read receive buffer
  {
    CAN1.readMsgBuf(&rxId, &len, rxBuf);  // Read data: len = data length, buf = data byte(s)
    digitalWrite(out2, HIGH);
    canRead1();
    digitalWrite(out2, LOW);
  }
}


void SendKeepAlive() {
  byte KeepAlive[8] = { 0X10, 0x01, 0x21, 0x07, 0x00, 0x00, 0x00, 0x00  };
  CAN1.sendMsgBuf(0x6ED, 0, 8, KeepAlive);
}

void canRead(){
 if (rxId == 0x2c1) 
    {
      //Serial.println(rxBuf[1]);
      blinkl = bitRead(rxBuf[0], 0);
      blinkr = bitRead(rxBuf[0], 1);
      parkl  = bitRead(rxBuf[0], 5);
      parkr  = bitRead(rxBuf[0], 6);
      groot  = bitRead(rxBuf[0], 3);
      grootp = bitRead(rxBuf[0], 2);

      wiper_pul = bitRead(rxBuf[1], 0);
      wiper_int = bitRead(rxBuf[1], 1);
      wiper_st1 = bitRead(rxBuf[1], 2);
      wiper_st2 = bitRead(rxBuf[1], 3);



      wiper_stage = (rxBuf[2] & 0x0F);
        if (wiper_stage ==  1) {wiper_sta = 1;}
        if (wiper_stage ==  5) {wiper_sta = 2;}
        if (wiper_stage ==  9) {wiper_sta = 3;}
        if (wiper_stage == 13) {wiper_sta = 4;}
        
      Serial.print("groot : ");
      Serial.print(groot);
      Serial.println(" ");

    }

}

void SendAnalogValues() 
{


    // struct the analogue values
  struct m2C1truct {
    unsigned int AVI1_V : 16;  //0:3-1:0
    unsigned int AVI2_V : 16;  //2:3-3:0
    unsigned int AVI3_V : 16;  //4:3-5:0
    unsigned int AVI4_V : 16;  //6:3-7:0
  };

  // union / make a struct
  union union_m2C1 {
    struct m2C1truct data;
    byte bytes[8];
  };

  // construct the can message
  struct canMsg {
    union_m2C1 m2C1;
  } canMsg;

  scaledvalue1 = map(blinkl, 0, 1, 1200, 4000);  // read analogue value and scale to 12 bit
  scaledvalue2 = map(blinkr, 0, 1, 1200, 4000);  // read analogue value and scale to 12 bit
  scaledvalue3 = map(groot, 0, 1, 0, 5000);  // read analogue value and scale to 12 bit
  scaledvalue4 = map(wiper_sta, 0, 4, 0000, 4000);  // read analogue value and scale to 12 bit


  byte HB1 = highByte(scaledvalue1);
  byte LB1 = lowByte(scaledvalue1);
  byte HB2 = highByte(scaledvalue2);
  byte LB2 = lowByte(scaledvalue2);
  byte HB3 = highByte(scaledvalue3);
  byte LB3 = lowByte(scaledvalue3);
  byte HB4 = highByte(scaledvalue4);
  byte LB4 = lowByte(scaledvalue4);


  byte Analog1[8] = { 0x80, blinkl, HB1, LB1, 0x00, 0x00, 0x00, 0x00 };
  byte Analog2[8] = { 0x81, blinkr, HB2, LB2, 0x00, 0x00, 0x00, 0x00 };
  byte Analog3[8] = { 0x82, groot , HB3, LB3, 0x00, 0x00, 0x00, 0x00 };
  byte Analog4[8] = { 0x83, grootp, HB4, LB4, 0x00, 0x00, 0x00, 0x00 };

  CAN1.sendMsgBuf(0x6EB, 0, 8, Analog1);  // send the can message onto the bus
  CAN1.sendMsgBuf(0x6EB, 0, 8, Analog2);  // send the can message onto the bus
  CAN1.sendMsgBuf(0x6EB, 0, 8, Analog3);  // send the can message onto the bus
  CAN1.sendMsgBuf(0x6EB, 0, 8, Analog4);  // send the can message onto the bus
}

void SendSpi() {


//  byte Spi1[8] = { 0x60, wiper_int, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//  byte Spi2[8] = { 0x61, wiper_st1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//  byte Spi3[8] = { 0x62, wiper_st2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//  byte Spi4[8] = { 0x63, wiper_pul, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


//  CAN1.sendMsgBuf(0x6EB, 0, 8, Spi1);  // send the can message onto the bus
//  CAN1.sendMsgBuf(0x6EB, 0, 8, Spi2);  // send the can message onto the bus
//  CAN1.sendMsgBuf(0x6EB, 0, 8, Spi3);  // send the can message onto the bus
//  CAN1.sendMsgBuf(0x6EB, 0, 8, Spi4);  // send the can message onto the bus

}


void canRead1() {}





