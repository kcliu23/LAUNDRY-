
#include <Wire.h>                                          // Library of communicate with I2C devices.
#include <SoftwareSerial.h>
#include <MPU6050.h>
#define BT_SERIAL_TX 8
#define BT_SERIAL_RX 7
SoftwareSerial BluetoothSerial(BT_SERIAL_TX, BT_SERIAL_RX);

const int MPU_ADDR = 0x68;                                 // MPU-6050 I2C address
int16_t Acc_x, Acc_y, Acc_z;                               // variables for 3-axis accelerometer
unsigned long Time;
MPU6050 MPU;
unsigned long int milli_time;
unsigned long lastSensorRead = 0;
unsigned long sensorReadInterval = 10;
void setup() 
{
  BluetoothSerial.begin(115200);                             // Initialize BlueTooth
  Wire.begin();
 Wire.setClock(400000L);
 MPU.initialize();
Serial.begin(115200);
Wire.beginTransmission(MPU_ADDR);
Wire.write(0x1A);
Wire.write(0b00000000);
Wire.endTransmission(true);
Wire.beginTransmission(MPU_ADDR);
Wire.write(0x24);
Wire.write(0b00001100);
Wire.endTransmission(true);                          // End the transmission
}
void loop() //Infinite loop
{
  Time = millis();
   if(millis() - lastSensorRead > sensorReadInterval){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                                        // Start with register 0x3B
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);                     // Request 6 registers, each axis is stored in 2 registers
//  
  Acc_x = (Wire.read()<<8 | Wire.read());                  // Read the raw data  
  Acc_y = (Wire.read()<<8 | Wire.read());
  Acc_z = (Wire.read()<<8 | Wire.read()); 
  lastSensorRead += sensorReadInterval;
  // print data 
  Serial.print((float)Time/1000,3);Serial.print(" DEV1 ");
  Serial.print((float)Acc_x/16384,3);  //16384 Represent 1g (gravity) 
  Serial.print(" "); Serial.print((float)Acc_y/16384,3);
  Serial.print(" ");Serial.println((float)Acc_z/16384,3 );
//  Serial.println(" DEV3 ");
  BluetoothSerial.print((float)Time/1000,3);BluetoothSerial.print(" DEV1 ");
  BluetoothSerial.print((float)Acc_x/16384,3);  //16384 Represent 1g (gravity) 
  BluetoothSerial.print(" "); BluetoothSerial.print((float)Acc_y/16384,3);
  BluetoothSerial.print(" "); BluetoothSerial.println((float)Acc_z/16384,3 );
//  BluetoothSerial.println(" DEV3 ");
   }
}
