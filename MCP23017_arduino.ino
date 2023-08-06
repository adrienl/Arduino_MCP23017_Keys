#include <Wire.h>

typedef enum RegistersBank0 {
  IODIRA = 0x00,
  IODIRB = 0x01,
  IPOLA = 0x02,
  IPOLB = 0x03,
  GPINTENA = 0x04,
  GPINTENB = 0x05,
  DEFVALA = 0x06,
  DEFVALB = 0x07,
  INTCONA = 0x08,
  INTCONB = 0x09,
  IOCON_AB = 0x0A,
  IOCON_BA = 0x0B,
  GPPUA = 0x0C,
  GPPUB = 0x0D,
  INTFA = 0x0E,
  INTFB = 0x0F,
  INTCAPA = 0x10,
  INTCAPB = 0x11,
  GPIOA = 0x12,
  GPIOB = 0x13,
  OLATA = 0x14,
  OLATB = 0x15
} RegistersBank0;

typedef enum IOCONSetup {
  CON_BANK =    0b10000000,
  CON_MIRROR =  0b01000000,
  CON_SEQOP =   0b00100000,
  CON_DISSLW =  0b00010000,
  CON_HAEN =    0b00001000,
  CON_ODR =     0b00000100,
  CON_INTPOL =  0b00000010,
} IOCONSetup;

uint8_t MCP23017AddrConv(uint8_t addr){
  if (addr > 7){
    addr = 7;
  }
  uint8_t fixedPart = 0b0100;
  uint8_t filteredAddr = 0b0000111 & addr;
  uint8_t slaveAddr = fixedPart << 3 | filteredAddr;
  return slaveAddr;
}

uint8_t I2CRead(uint8_t addr, uint8_t reg){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.requestFrom(0x20, 1, true);
  uint8_t data = Wire.read();
  Wire.endTransmission();
  delay(5);
  return data;
}

void I2CWrite(uint8_t addr, uint8_t reg, uint8_t data){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
  delay(50);
}

static uint8_t _devAddr = 0;

uint16_t readNotes(uint8_t addr){
  uint8_t AData = I2CRead(addr, GPIOA);
  uint8_t BData = I2CRead(addr, GPIOB);
  return BData << 8 | AData;
}

void initDevice(){
  _devAddr = MCP23017AddrConv(0);
  I2CWrite(_devAddr, IOCON_AB, CON_MIRROR);
  uint8_t config = I2CRead(_devAddr, IOCON_AB);
  Serial.println(config, BIN);

  I2CWrite(_devAddr, IODIRA, 0b11111111); //I/O DIRECTION REGISTER (ADDR 0x00)
  I2CWrite(_devAddr, IODIRB, 0b11111111); //I/O DIRECTION REGISTER (ADDR 0x00)
  I2CWrite(_devAddr, GPPUA, 0b11111111); //GPIO PULL-UP RESISTOR REGISTER (ADDR 0x06)
  I2CWrite(_devAddr, GPPUB, 0b11111111); //GPIO PULL-UP RESISTOR REGISTER (ADDR 0x06)
  I2CWrite(_devAddr, GPINTENA, 0b11111111); //INTERRUPT-ON-CHANGE PINS (ADDR 0x02)
  I2CWrite(_devAddr, GPINTENB, 0b11111111); //INTERRUPT-ON-CHANGE PINS (ADDR 0x02)
  I2CWrite(_devAddr, INTCONA, 0b00000000); //INTERRUPT-ON-CHANGE CONTROL REGISTER (ADDR 0x04)
  I2CWrite(_devAddr, INTCONB, 0b00000000); //INTERRUPT-ON-CHANGE CONTROL REGISTER (ADDR 0x04)
  I2CWrite(_devAddr, DEFVALA, 0b11111111); //DEFAULT VALUE REGISTER (ADDR 0x03)
  I2CWrite(_devAddr, DEFVALB, 0b11111111); //DEFAULT VALUE REGISTER (ADDR 0x03)

  Serial.println("MCP23017 Initiated");
}

void setup() {
  // put your setup code here, to run once:
 // Wire.begin();
  Serial.begin(9600);
  Serial.println("Setup Begin");
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), &octaveChange, FALLING);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  initDevice();
  Serial.println("Setup End");
  //Make One Read First
  uint16_t notes = readNotes(_devAddr);
}

static bool interrupt = 0;

void octaveChange() {
  interrupt = 1;
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  if (interrupt == 1){
    interrupt = 0;
    digitalWrite(LED_BUILTIN, LOW);
    uint16_t notes = readNotes(_devAddr);
    Serial.println(notes, BIN);
  }
}
