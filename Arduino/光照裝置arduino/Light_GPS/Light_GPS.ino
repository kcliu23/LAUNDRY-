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
#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include "SPI.h" 
#include "lib_aci.h"
#include "aci_setup.h"
#include "uart_over_ble.h"
#include "services.h"
SoftwareSerial serial_connection(A1, A0); 
TinyGPSPlus gps;
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
/*---------變數區-----------*/
int pass=0;
bool passGps=false;
int Gpstest=1024;
int GpsTotalData=0;
int AllTotalData=0;
int connectedPhone=0;
int writeAddress=4;
int DataReadAddress=0;
long TotalLight=0;
int count=1;
int mode=2;
int RealTime=0;
//A 30mins mode3
//B 2mins mode2
int GpsWriteAddress=1024;
int GpsreadAddress=1024;
int test=4;
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
int TotalData=0;
bool del=false;
bool unlock=true;
DS1339 RTC = DS1339();


void setup()
{
  Serial.begin(9600);
  serial_connection.begin(9600);
  long readtime[]={5};
  EEPROMread(DataReadAddress,(uint8_t*)readtime,4);
  Serial.println(readtime[0]);
  
  Wire.begin();
  accel.begin(BMA250_range_2g, BMA250_update_time_64ms);//This sets up the BMA250 accelerometer
  TSL2572nit(GAIN_1X);
  
  int readData[]={1};
  
  BLEsetup();
  //calibrate();

  RTC.start();
  for(int i=0;i<510;i++)//256*2=512
      {        
        EEPROMread(test,(uint8_t*)readData,2);
        if(readData[0]!=0)
        {
          /*Serial.print(test+":");
          Serial.println(readData[0]);*/
               test+=2;
        }       
        else{      
               break;
            }
        
      }
      Serial.print("LightandStepData:");
      Serial.println((test-4)/2);
      TotalData=(test-4)/2;
  for(int i=0;i<512;i++)
      {        
        EEPROMread(GpsWriteAddress,(uint8_t*)readData,4);
        if(readData[0]!=0)
        {
          /*Serial.print(test+":");
          Serial.println(readData[0]);*/
               GpsWriteAddress+=4;
        }       
        else{      
               break;
            }
        
      }
      Serial.print("GpsTotal:");
      Serial.println((GpsWriteAddress-1024)/4);
      GpsTotalData=(GpsWriteAddress-1024)/4;
      Serial.print("AllTotalData:");
      AllTotalData=TotalData+GpsTotalData;
      Serial.println(AllTotalData);
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
    ///Serial.print(F("Xavg"));
    //Serial.println(xavg);
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
     else if(ble_rx_buffer[0]=='d')
     {
      del=true;
      unlock=true;
     }
    else if(ble_rx_buffer[0]=='r' && ble_rx_buffer[1]=='t')//now
    {
    //read_time();
    RealTime=1;
    connectedPhone=0;
  delaycontrol=1000;
    }
  else if(ble_rx_buffer[0]=='c' && ble_rx_buffer[1]=='t')//close
    {
      //read_time();
    RealTime=0;
   // connectedPhone=0;//
  delaycontrol=100;
  unlock=true;
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
  { //Serial.print("connectedPhone:");
    Serial.println("Transmitting...");
    int readData[]={5};
    long readtime[]={5};
    if(DataReadAddress==0)
    {
      char sendBuffer[20];
      EEPROMread(DataReadAddress,(uint8_t*)readtime,4);
      Serial.println(readtime[0]);
      Serial.println("我傳了AlltotalData:");
      Serial.print(AllTotalData);
      
      String msg="";
      msg+="T"+String(AllTotalData);
      msg+="+";
      int str_len = msg.length()+1;
      msg.toCharArray(sendBuffer, str_len);
      sendBuffer[str_len] = '\0';
      lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, str_len);
       msg="";
    msg+=String(mode);
    msg+="+";
      msg+=String(readtime[0]);
       sendBuffer[20];
       str_len = msg.length()+1;
      msg.toCharArray(sendBuffer, str_len);
    
      //str_len++;
      sendBuffer[str_len] = '\0'; //Terminate string
    
      lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, str_len);
      ble_rx_buffer_len = 0;
    
      DataReadAddress+=4;
    }
    else
    {
      
      if(pass==4)//if(DataReadAddress<1024)
     {
      EEPROMread(DataReadAddress,(uint8_t*)readData,2);
      if(readData[0]!=0)
      {
      if(DataReadAddress>=4&&DataReadAddress%4==0)    
      Serial.print("Light:");    
      else
      Serial.print("Step:");   
      Serial.println(readData[0]);
      }
      String msg=""; 
      if(DataReadAddress%4!=0&&readData[0]!=0)
    {
    msg+="B"; 
    }
      if(readData[0]==0&&unlock)
    {
      msg+="end";
      delaycontrol=100;
      unlock=false;
    
    }
   else
    {
      if(readData[0]!=0)
      msg+=String(readData[0]);
    }
      char sendBuffer[20];
      int str_len = msg.length()+1;
      msg.toCharArray(sendBuffer, str_len);
    
      //str_len++;
      sendBuffer[str_len] = '\0'; //Terminate string
     // if(unlock)//可能要刪除
      lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, str_len);
      ble_rx_buffer_len = 0;
     // delay(2000);
      DataReadAddress+=2;
    }
          
   
     if(pass<4)
    {
      String msg="";
      char sendBuffer[20];
      double readGps[]={0.0};
      EEPROMread(GpsreadAddress,(uint8_t*)readGps,4);
      Serial.print(readGps[0]);
      if(readGps[0]!=0.0)
       {
          msg+="G";
          
          long gps=readGps[0]*1000000;
          String strGps=String(gps);
          msg+=strGps;
          Serial.print("傳出的GPSlat:");
          Serial.println(strGps);
          //Serial.print(readGps[0],6);
         /* int str_len = msg.length()+1;
          msg.toCharArray(sendBuffer, str_len);
          sendBuffer[str_len] = '\0';
          lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, str_len);*/
          //delay(1000);
         
          GpsreadAddress+=4;  
          EEPROMread(GpsreadAddress,(uint8_t*)readGps,4);
          msg+="+";
          
       
          gps=readGps[0]*1000000;
          strGps=String(gps);
          msg+=strGps;
          Serial.println("傳出的GPSlng:");   
          Serial.println(strGps);
          //Serial.print(readGps[0],6);
          GpsreadAddress+=4;  
          
          
          int str_len = msg.length()+1;
          msg.toCharArray(sendBuffer, str_len);
    
      
          sendBuffer[str_len] = '\0';
         
          lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, str_len);
          pass++;
      }
     
   }
    
   }
    mode=2;
    TotalLight=0;
    steps=1;
    /*if(del)
    {
      count=1;
      int WriteClearAddress=0;
      for(int i=0; i<512; i++)
      {
        int writeClear[]={0};
        EEPROMwrite(WriteClearAddress,(uint8_t*)writeClear,2);
        WriteClearAddress+=2;
      }
      delay(250);
    
      long writeTimeData[]={0};
      RTC.readTime();
      long timedata=int(RTC.getYears()-2000)*100000000
            +int(RTC.getMonths())*1000000
            +long(RTC.getDays())*10000
            +int(RTC.getHours())*100
            +int(RTC.getMinutes());
      writeTimeData[0]=timedata;
      Serial.print("已經刪除DATA");
      EEPROMwrite(0,(uint8_t*)writeTimeData,4);
 
      DataReadAddress=0;
      connectedPhone=0;
      writeAddress=4;
      del=false;
      TotalData=0;
      GpsTotalData=0;
      AllTotalData=0;
       //Serial.print("connectedPhone:");
       //Serial.print(connectedPhone);
    }*/
  
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
    else if(RealTime!=1 &&count%1200==0 &&connectedPhone!=2) //改成1200
    {
      Serial.println(F("2 mode Begin record"));//////////兩分鐘寫到PROM一次
      int readData[]={1};     //why??
      for(int i=0;i<510;i++)
      {        
       EEPROMread(writeAddress,(uint8_t*)readData,2);
        if(readData[0]==0)//兩byte存取
        {
          Serial.print(F("Total_Light:"));
          
          Serial.println(TotalLight);
          Serial.println(writeAddress);
       int rootnum=0;
          rootnum=sqrt(TotalLight);
          Serial.print("rootLight:");
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
      
      Serial.print("TotalData:");
      Serial.println((writeAddress-4)/2);
      TotalData=(writeAddress-4)/2;
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
    if(RealTime!=1 &&count%50==0 &&connectedPhone!=2)
  {
    while(serial_connection.available())//While there are characters to come from the GPS
      {
        gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time
      }
    if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
        {
    

          Serial.println("I am in Gps");

     
          double readGps[]={0.0};
          Serial.println(String(gps.location.lat()));
          Serial.println(String(gps.location.lng())); 
          double GpsData[]={0};
          GpsData[0]={gps.location.lat()};
      
          EEPROMwrite(GpsWriteAddress,(uint8_t*)GpsData,4);
      
          Serial.print(GpsWriteAddress);
          Serial.print(":");
          Serial.println(GpsData[0]);
          EEPROMread(GpsWriteAddress,(uint8_t*)readGps,4);
          Serial.println("寫入後緯度:");
          Serial.println(readGps[0],6);
      
          GpsWriteAddress+=4;
          GpsData[0]={gps.location.lng()};
          Serial.print(GpsWriteAddress);
          Serial.print(":");
          Serial.println(GpsData[0]);
      
          EEPROMwrite(GpsWriteAddress,(uint8_t*)GpsData,4);
          EEPROMread(GpsWriteAddress,(uint8_t*)readGps,4);
          Serial.println("寫入後經度:");
      
          Serial.println(readGps[0],6);
      
          GpsWriteAddress+=4;
      
          GpsTotalData=(GpsWriteAddress-1024)/4;
          AllTotalData=TotalData+GpsTotalData;
    
 
      } 
    }
  }
   
    delay(delaycontrol); //改成100

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
