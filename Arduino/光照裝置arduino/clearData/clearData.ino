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

#if defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#else
#define SerialMonitorInterface Serial
#endif

BMA250 accel;
int gain_val = 0;

//bluetooth
uint8_t ble_rx_buffer[21];    //手機傳送資料到arduino
uint8_t ble_rx_buffer_len = 0;
uint8_t ble_connection_state = false;

#if CODEBENDER
#include "UART.h"
#endif
int connectedPhone=0;
int writeAddress=4;
int DataReadAddress=0;
long TotalLight=0;
int count=1;
int mode=2;
int RealTime=0;
//A 30mins mode3
//B 2mins mode2
int openswitch=0;
//  rrfloat threshhold=80.0;
float totvectdect=0;
float xavg=0;
float yavg=0;
float zavg=0;
float xsum=0;
float ysum=0;
float zsum=0;
int steps=1;
int delaycontrol=100;
int test=4;
DS1339 RTC = DS1339();


void setup()
{
  int readData[]={5};
  openswitch=1;
  Serial.begin(9600);
  Wire.begin();
  accel.begin(BMA250_range_2g, BMA250_update_time_64ms);//This sets up the BMA250 accelerometer
  TSL2572nit(GAIN_1X);
int WriteClearAddress=0;
      for(int i=0; i<12000; i++)
      {
        int writeClear[]={0};
        EEPROMwrite(WriteClearAddress,(uint8_t*)writeClear,2);
        WriteClearAddress+=2;
      }
  BLEsetup();
  //calibrate();

  RTC.start();
  long writeTimeData[]={0};
      RTC.readTime();
      long timedata=int(RTC.getYears()-2000)*100000000
            +int(RTC.getMonths())*1000000
            +long(RTC.getDays())*10000
            +int(RTC.getHours())*100
            +int(RTC.getMinutes());
      writeTimeData[0]=timedata;
      Serial.println(timedata);
      EEPROMwrite(0,(uint8_t*)writeTimeData,4);
  long readtime[]={5};
  EEPROMread(0,(uint8_t*)readtime,4);
      Serial.println(readtime[0]);


 
  /*for(int i=0;i<512;i++)
      {        
        EEPROMread(test,(uint8_t*)readData,2);
        if(readData[0]!=0)
        {
        
          Serial.print(test);
          Serial.print(":");
          Serial.println(readData[0]);
          
          if(readData[0]<-1000||readData[0]>1000)
          {
            EEPROMread(test,(uint8_t*)readtime,4);
          Serial.println(readtime[0]);
          
          }        
        }
        
        else{
          Serial.println(test);
          EEPROMwrite(test,(uint8_t*)writeTimeData,4);
          EEPROMread(test,(uint8_t*)readtime,4);
          Serial.println(readtime[0]);
          break;
          }
        test+=4;
      }*/
      Serial.print("我已經清空了!");
  //EEPROMread(8,(uint8_t*)readData,2);
  //Serial.println(readData[0]);
}

void loop()
{
  int AmbientLightLux = 0;
  count++;      //1count=0.1秒
  if(count%10==0) //改成10//累積到count==10算一次=1秒
  {
    AmbientLightLux= Tsl2572ReadAmbientLight();
  }
  else
  {
   AmbientLightLux=0;
  }
  
   accel.read();
  float totvect=0;
  //float totave[100]={0};
  float xaccl=accel.X;
  float yaccl=accel.Y;
  float zaccl=accel.Z;
  xsum+=xaccl;
  ysum+=yaccl;
    zsum+=zaccl;
  
  if(count % 10==0)//累積到count==10算一次=1秒
  {
    xavg=xsum/10;       //取平均
    yavg=ysum/10;
    zavg=zsum/10;
    Serial.print(F("Xavg"));
    Serial.println(xavg);
    xsum=ysum=zsum=0;
  }
    //totave[i] = (totvect[i] + totvect[i-1]) / 2 ;
   totvect = sqrt(((xaccl - xavg)* (xaccl - xavg))+ ((yaccl - yavg)*(yaccl - yavg)) + ((zaccl - zavg)*(zaccl - zavg)));//晃動計算
  if(count % 3==0)
  {
    totvectdect=totvect;
  }
  //delay(200);
  //Serial.println(totvect);
  if (abs(totvectdect-totvect) > 5 && (xavg!=0 || yavg!=0 || zavg!=0 ) && totvectdect!=0)//計步數
    {
    
      Serial.print(F("steps:"));
     //Serial.println(totvect);
    steps=steps+1;
    
    Serial.println(steps);
  Serial.print(F("totvect:"));
  Serial.println(totvect);
  }
  int eeAddress = sizeof(int);
  aci_loop();//Process any ACI commands or events from the NRF8001- main BLE handler, must run often. Keep main loop short.
  
  if (ble_rx_buffer_len) 
  {  //Check if data is available//手機傳電腦，電腦接收手機傳送
    Serial.print(ble_rx_buffer_len);
    Serial.print(" : ");
    Serial.println((char*)ble_rx_buffer);
    
    if(ble_rx_buffer[0]=='A')//30分鐘傳一次(mode3
    {
    RealTime=0;
      connectedPhone=1;//mode==3
    delaycontrol=1000;
    }
    else if(ble_rx_buffer[0]=='B')//2分鐘傳一次(mode2
    {
      RealTime=0;
    connectedPhone=2;//mode==2
  delaycontrol=1000;
    }
    else if(ble_rx_buffer[0]=='r' && ble_rx_buffer[1]=='t')//now
    {
    //read_time();
    RealTime=1;
  delaycontrol=1000;
    }
  else if(ble_rx_buffer[0]=='c' && ble_rx_buffer[1]=='t')//close
    {
      //read_time();
    RealTime=0;
  delaycontrol=100;
    }
    else if(ble_rx_buffer[0]=='s' && ble_rx_buffer[1]=='t')//set time
    {
      set_time((2000+(ble_rx_buffer[2]-48)*10+(ble_rx_buffer[3]-48)),
          ((ble_rx_buffer[4]-48)*10+(ble_rx_buffer[5]-48)),
          ((ble_rx_buffer[6]-48)*10+(ble_rx_buffer[7]-48)),
          ((ble_rx_buffer[8]-48)*10+(ble_rx_buffer[9]-48)),
          ((ble_rx_buffer[10]-48)*10+(ble_rx_buffer[11]-48)),
          ((ble_rx_buffer[12]-48)*10+(ble_rx_buffer[13]-48)));
    }
    ble_rx_buffer_len = 0;//clear afer reading
    delay(100);
  }

  else if(connectedPhone==2)//B mode
  {
    int readData[]={5};
    long readtime[]={5};
    if(DataReadAddress==0)
    {
      EEPROMread(DataReadAddress,(uint8_t*)readtime,4);
      Serial.println(readtime[0]);
    
      String msg="";
    msg+=String(mode);
    msg+="+";
      msg+=String(readtime[0]);
      char sendBuffer[20];
      int str_len = msg.length()+1;
      msg.toCharArray(sendBuffer, str_len);
    
      //str_len++;
      sendBuffer[str_len] = '\0'; //Terminate string
    
      lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, str_len);
      ble_rx_buffer_len = 0;
    
      DataReadAddress+=4;
    }
    else
    {
      EEPROMread(DataReadAddress,(uint8_t*)readData,2);        
      Serial.println(readData[0]);
      
      String msg=""; 
      if(DataReadAddress%4!=0)
    {
    msg+="B"; 
    }
      if(readData[0]==0)
    {
      msg+="end";
    delaycontrol=100;
    }
    else
    {
      
      msg+=String(readData[0]);
    }
      char sendBuffer[20];
      int str_len = msg.length()+1;
      msg.toCharArray(sendBuffer, str_len);
    
      //str_len++;
      sendBuffer[str_len] = '\0'; //Terminate string
    
      lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, str_len);
      ble_rx_buffer_len = 0;
    
      DataReadAddress+=2;
    }
    mode=2;
    TotalLight=0;
  steps=1;
 
    if(DataReadAddress==1024 || readData[0]==0)
    {
      Serial.print("I am in!");
      count=1;
      int WriteClearAddress=0;
      for(int i=0; i<2048; i++)
      {
        int writeClear[]={0};
        EEPROMwrite(WriteClearAddress,(uint8_t*)writeClear,2);
        WriteClearAddress+=2;
      }
      delay(250);
    
      
  
      DataReadAddress=0;
      connectedPhone=0;
      writeAddress=4;
    }
  }
  //紀錄
  TotalLight+=AmbientLightLux;//非同步 一直累加
  if( mode==2)
    {
    if(RealTime==1 && count%1200!=0 ) //改成1200  =2min    //////real time的資料->圖表用
    {
      String msg="L";
      int tempLight=Tsl2572ReadAmbientLight();
      msg+=String(tempLight,DEC);
      msg+="XS";
      //msg+=String(totvect);
    msg+="Z";
      msg+=String(totvectdect);
      char sendBuffer[20];
      int str_len = msg.length();
      msg.toCharArray(sendBuffer, str_len);

      str_len++;
      sendBuffer[str_len] = '\0'; //Terminate string

      lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, str_len);
      msg="";
    }
    else if(count%600==0) //改成1200
    {
      Serial.println(F("2 mode Begin record"));//////////兩分鐘寫到PROM一次
      int readData[]={1};     //why??
      for(int i=0;i<512;i++)
      {        
        EEPROMread(writeAddress,(uint8_t*)readData,2);
        if(readData[0]==0)//兩byte存取
        {
          int rootnum=0;
          Serial.print(F("Total_Light:"));
          
          Serial.println(TotalLight);
          Serial.println(writeAddress);
      
          rootnum=sqrt(TotalLight);
          Serial.println("rootnum");
          Serial.println(rootnum);
          int writeData[]={(rootnum)};
          //int writeData[]={(TotalLight/10)};
          EEPROMwrite(writeAddress,(uint8_t*)writeData,2);//光資料 2個位置
          writeAddress+=2;    
          Serial.println(writeAddress);
          Serial.print(F("Steps:"));
     
          Serial.println(steps);
       
          int writeStepData[]={(steps/(3.5))+1};
          EEPROMwrite(writeAddress,(uint8_t*)writeStepData,2);//部署兩個位置
          writeAddress+=2;
          break;
        }
        else
        { 
          EEPROMread(writeAddress,(uint8_t*)readData,2);//資料存滿 下兩個位置給他
          Serial.print(F("Else:"));
          Serial.println(readData[0]);
          writeAddress+=2;
        }
      }
      TotalLight=0;
    steps=1;
    } 
  }

    delay(delaycontrol); //改成100    8360
//Serial.println(TotalLight);
}
void TSL2572nit(uint8_t gain)//LightSensor
{
  Tsl2572RegisterWrite( 0x0F, gain );//set gain
  Tsl2572RegisterWrite( 0x01, 0xED );//51.87 ms
  Tsl2572RegisterWrite( 0x00, 0x03 );//turn on
  if(GAIN_DIVIDE_6)
  Tsl2572RegisterWrite( 0x0D, 0x04 );//scale gain by 0.16+*2-+
  if(gain==GAIN_1X)
  gain_val=1;
  else if(gain==GAIN_8X)
  gain_val=8;
  else if(gain==GAIN_16X)
  gain_val=16;
  else if(gain==GAIN_120X)
  gain_val=120;
}

void Tsl2572RegisterWrite( byte regAddr, byte regData )//LightSensor
{
  Wire.beginTransmission(TSL2572_I2CADDR);
  Wire.write(0x80 | regAddr); 
  Wire.write(regData);
  Wire.endTransmission(); 
}

float Tsl2572ReadAmbientLight()//LightSensor
{     
  uint8_t data[4]; 
  int c0,c1;
  float lux1,lux2,cpl;

  Wire.beginTransmission(TSL2572_I2CADDR);
  Wire.write(0xA0 | 0x14);
  Wire.endTransmission();
  Wire.requestFrom(TSL2572_I2CADDR,4);
  for(uint8_t i=0;i<4;i++)
  { 
  data[i] = Wire.read();
  }
  c0 = data[1]<<8 | data[0];
  c1 = data[3]<<8 | data[2];
  //see TSL2572 datasheet
  cpl = 51.87 * (float)gain_val / 60.0;
  if(GAIN_DIVIDE_6)
  {
  cpl/=6.0;
  }
  lux1 = ((float)c0 - (1.87 * (float)c1)) / cpl;
  lux2 = ((0.63 * (float)c0) - (float)c1) / cpl;
  cpl = max(lux1, lux2);
  return max(cpl, 0.0);
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

void set_time(int year,int month,int day,int hour,int minute,int second)
{
    RTC.setSeconds(second);
    RTC.setMinutes(minute);
    RTC.setHours(hour);
    RTC.setDays(day);
    RTC.setMonths(month);
    RTC.setYears(year);
    RTC.writeTime();
    read_time();
}

void read_time() 
{
  RTC.readTime(); // update RTC library's buffers from chip
  printTime(0);
}

void printTime(byte type)
{
  if(!type)
  {
    Serial.print(RTC.getYears());
  Serial.print(int(RTC.getMonths()));
    Serial.print(int(RTC.getDays()));
    
  }
  else
  {
    //if(RTC.getDays() == 0) // Day-Of-Week repeating alarm will have DayOfWeek *instead* of date, so print that.
    {
      Serial.print(int(RTC.getDayOfWeek()));
      Serial.print(F("th day of week, "));
    }
    //else
    {
      Serial.print(int(RTC.getDays()));
      Serial.print(F("th day of month, "));      
    }
  }
  
  //Serial.print("  ");
  Serial.print(int(RTC.getHours()));
  //Serial.print(":");
  Serial.print(int(RTC.getMinutes()));
  //Serial.print(":");
  Serial.print(int(RTC.getSeconds()));  
}
