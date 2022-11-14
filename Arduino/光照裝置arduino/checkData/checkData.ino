#define CODEBENDER true

#if defined(ARDUINO_ARCH_SAMD)
  #define SerialMonitorInterface SerialUSB
#else
  #define SerialMonitorInterface Serial
#endif

#include "DSRTCLib.h"
#include <avr/power.h>
#include <avr/sleep.h>


#define BLE_DEBUG true

#include "SPI.h"
#include "lib_aci.h"
#include "aci_setup.h"
#include "uart_over_ble.h"
#include "services.h"

#include <Wire.h>
#include "BMA250.h"
#include <SoftwareSerial.h>
#define TSL2572_I2CADDR     0x39


#define EEPROM_A0 0
#define EEPROM_A1 0
#define EEPROM_ADDR 0x50|(EEPROM_A1<<1)|(EEPROM_A0<<0)

#define GAIN_1X 0
#define GAIN_8X 1
#define GAIN_16X 2
#define GAIN_120X 3
#define GAIN_DIVIDE_6 true 
//LightSensor
  int readData[]={5};
  long dateda[]={2};
void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    EEPROMread(0,(uint8_t*)dateda,4);
      Serial.println(dateda[0]);
      int addr=4;
      
  // put your main code here, to run repeatedly:
    for(int i=0;i<12000;i++)
    {
      EEPROMread(addr,(uint8_t*)readData,2);
      Serial.print(addr);
      Serial.print(":");
      Serial.println(readData[0]);
      addr+=2;
    }
}

void loop() {


}


void EEPROMread(unsigned long addr, uint8_t* buff, unsigned long count){
  for(unsigned long i=0;i<count;i++){
    buff[i]=EEPROMread(addr+i);
  }
}

void EEPROMwrite(unsigned long addr, uint8_t* buff, unsigned long count){
  for(unsigned long i=0;i<count;i++){
    EEPROMwrite(addr+i,buff[i]);
  }
}

byte EEPROMread(unsigned long addr){
  uint8_t val=255;
  uint8_t I2Caddr=EEPROM_ADDR;
  if(addr>0x0000FFFF){
    I2Caddr|=0x04;
  }
  Wire.beginTransmission(I2Caddr);
  Wire.write(addr>>8);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(I2Caddr,(uint8_t)1);
  while(Wire.available()){
    val=Wire.read();
  }
  return val;
}

byte EEPROMwrite(unsigned long addr, byte val){
  uint8_t I2Caddr=EEPROM_ADDR;
  if(addr>0x0000FFFF){
    I2Caddr|=0x04;
  }
  Wire.beginTransmission(I2Caddr);
  Wire.write(addr>>8);
  Wire.write(addr);
  Wire.write(val);
  Wire.endTransmission();
  Wire.beginTransmission(I2Caddr);
  unsigned long timeout=millis();
  while(Wire.endTransmission() && millis()-timeout<10){
    Wire.beginTransmission(I2Caddr);
  }
}
