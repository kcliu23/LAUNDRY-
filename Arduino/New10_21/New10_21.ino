#include <SPI.h>
#include <STBLE.h>
//Debug output adds extra flash and memory requirements!
#ifndef BLE_DEBUG
#define BLE_DEBUG true
#endif
#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif
uint8_t ble_rx_buffer[21];
uint8_t ble_rx_buffer_len = 0;
uint8_t ble_connection_state = false;
#define PIPE_UART_OVER_BTLE_UART_TX_TX 0
#include "DSRTCLib.h"
#include <avr/power.h>
#include <avr/sleep.h>
#include <Wire.h>
#include "BMA250.h"
#define TSL2572_I2CADDR     0x39
#define EEPROM_A0 0
#define EEPROM_A1 0
#define EEPROM_ADDR 0x50|(EEPROM_A1<<1)|(EEPROM_A0<<0)
#define GAIN_1X 0
#define GAIN_8X 1
#define GAIN_16X 2
#define GAIN_120X 3
#define GAIN_DIVIDE_6 true 
BMA250 accel;
int gain_val = 0;
int connectedPhone=0;
int writeAddress=4;
int DataReadAddress=0;
long TotalLight=0;
long count=1;
int mode=2;
int RealTime=0;
//A 30mins mode3
//B 2mins mode2
int test=4;
float totvectdect=0;
float xavg=0;
float yavg=0;
float zavg=0;
float xsum=0;
float ysum=0;
float zsum=0;
int steps=1;
int secStep=0;
int TotalData=0;
bool del=false;
bool unlock=true;
bool close1=false;

void setup() {
  SerialMonitorInterface.begin(9600);
  while (!SerialMonitorInterface); //This line will block until a serial monitor is opened with TinyScreen+!
  BLEsetup();
  
  long readtime[]={5};
  EEPROMread(DataReadAddress,(uint8_t*)readtime,4);
  Serial.println(readtime[0]);
  Wire.begin();
  accel.begin(BMA250_range_2g, BMA250_update_time_64ms);//This sets up the BMA250 accelerometer
  TSL2572nit(GAIN_1X);
  
  int readData[]={1};
  int readData2[]={1};
  
  for(int i=0;i<10200;i++)
  {        
    EEPROMread(test,(uint8_t*)readData,2);
    EEPROMread(test+2,(uint8_t*)readData2,2);
    if(readData[0]!=0||readData2[0]!=0)
      test+=2;       
    else    
      break;     
  }
  Serial.print(F("TotalData:"));
  Serial.println((test-4)/2);
  TotalData=(test-4)/2;
  writeAddress=test;
}

void loop() {
  
  int AmbientLightLux = 0;
  count++;            //1count=0.1秒
  if(count%10==0)     //改成10  //累積到count==10算一次=1秒
    AmbientLightLux= Tsl2572ReadAmbientLight();
  else
    AmbientLightLux=0;
  
  accel.read();
  float totvect=0;
  float xaccl=accel.X;
  float yaccl=accel.Y;
  float zaccl=accel.Z;
  xsum+=xaccl;
  ysum+=yaccl;
  zsum+=zaccl;
  float SVM=0;
  if(count % 10==0)//累積到count==10算一次=1秒
  {
    xavg=xsum/10;       //取平均
    yavg=ysum/10;
    zavg=zsum/10;
    xsum=ysum=zsum=0;
 
  }
   //SVM=sqrt(xaccl*xaccl+yaccl*yaccl+zaccl*zaccl);
   totvect = sqrt(((xaccl - xavg)* (xaccl - xavg))+ ((yaccl - yavg)*(yaccl - yavg)) + ((zaccl - zavg)*(zaccl - zavg)));//晃動計算
  if(count % 5==0)
  {
    totvectdect=totvect;
  }
  
   
  if (abs(totvectdect-totvect) > 13.8&& totvectdect!=0)//計步數
    {

      Serial.print(F("steps:"));
      //Serial.println(totvect);
      steps=steps+1;
      secStep++;
      Serial.println(steps);
      Serial.print(F("totvect:"));
      Serial.println(totvect);
  }

  aci_loop();   //Process any ACI commands or events from the NRF8001- main BLE handler, must run often. Keep main loop short.
  
  if (ble_rx_buffer_len)    //Check if data is available    手機傳電腦，電腦接收手機傳送
  {   
    
    if(ble_rx_buffer[0]=='B')      //2分鐘傳一次(mode2
    {
      Serial.println(F("into B mode"));
      RealTime=0;
      connectedPhone=2;//mode==2
      
    }
    else if(ble_rx_buffer[0]=='d')
    {
      del=true;
      unlock=true;
    }
    else if(ble_rx_buffer[0]=='r' && ble_rx_buffer[1]=='t')     //now
    {
      RealTime=1;
      connectedPhone=0;
      
    }
    else if(ble_rx_buffer[0]=='c' && ble_rx_buffer[1]=='t')     //close
    {
      RealTime=0;
      
      //unlock=true;8.31改
      Serial.println(F("close"));
    }
    else if(ble_rx_buffer[0]=='s' && ble_rx_buffer[1]=='t')     //set time
    {

          set_time(((ble_rx_buffer[2]-48)*10+(ble_rx_buffer[3]-48)),
          ((ble_rx_buffer[4]-48)*10+(ble_rx_buffer[5]-48)),
          ((ble_rx_buffer[6]-48)*10+(ble_rx_buffer[7]-48)),
          ((ble_rx_buffer[8]-48)*10+(ble_rx_buffer[9]-48)),
          ((ble_rx_buffer[10]-48)*10+(ble_rx_buffer[11]-48)));
         
          Serial.print(ble_rx_buffer[2]-48);
          Serial.print(ble_rx_buffer[3]-48);
          Serial.print(ble_rx_buffer[4]-48);
          Serial.print(ble_rx_buffer[5]-48);
          Serial.print(ble_rx_buffer[6]-48);
          Serial.print(ble_rx_buffer[7]-48);
          Serial.print(ble_rx_buffer[8]-48);
          Serial.print(ble_rx_buffer[9]-48);
          Serial.print(ble_rx_buffer[10]-48);
          Serial.print(ble_rx_buffer[11]-48);
       
          close1=true;
    }
     
    ble_rx_buffer_len = 0;      //clear afer reading
    bleDelay(100);//delay(100);
  } 
  else if(connectedPhone==2)    //B mode
  { 
    Serial.println(F("Transmitting..."));
    int readData[]={5};
    int readData2[]={5};
    long readtime[]={5};
    if(DataReadAddress==0)
    {
      char sendBuffer[20];
      EEPROMread(DataReadAddress,(uint8_t*)readtime,4);
      Serial.print(readtime[0]);
      Serial.println(F("Total is coming!"));
      Serial.println(TotalData);
      String msg="";
      msg+="T"+String(TotalData);
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
      sendBuffer[str_len] = '\0'; //Terminate string
    
      lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, str_len);
      ble_rx_buffer_len = 0;
      DataReadAddress+=4;
    }
    else
    {  
      EEPROMread(DataReadAddress,(uint8_t*)readData,2);
      EEPROMread(DataReadAddress+2,(uint8_t*)readData2,2);
      if(readData[0]!=0||readData2[0]!=0)//8.31修改解決光照量=0問題
      {
          if(DataReadAddress>=4&&DataReadAddress%4==0)    
          Serial.print(F("Light:"));    
          else
          Serial.print(F("Step:"));   
          Serial.println(readData[0]);
      }
      String msg=""; 
      if(DataReadAddress%4!=0&&readData[0]!=0)
      {
        msg+="B"; 
      }
      if(readData[0]==0&&unlock&&readData2[0]==0)
      {
        delay(1000);
        Serial.println(F("sent End"));
        msg+="end";
        unlock=false;
      }
      else
      {
        //if(readData[0]!=0)
        msg+=String(readData[0]);
      }
      char sendBuffer[20];
      int str_len = msg.length()+1;
      msg.toCharArray(sendBuffer, str_len);
      sendBuffer[str_len] = '\0'; //Terminate string
      lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, str_len);
      ble_rx_buffer_len = 0;
    
      DataReadAddress+=2;
    }
    mode=2;
    TotalLight=0;
    steps=1;
    if(del)
    {
      count=1;
      int WriteClearAddress=0;
      for(int i=0; i<TotalData+4; i++)
      {
        int writeClear[]={0};
        EEPROMwrite(WriteClearAddress,(uint8_t*)writeClear,2);
        WriteClearAddress+=2;
      }
//9.1修改      bleDelay(250);//delay(250);
      Serial.print(F("delete DATA already"));
      del=false;
    }
    if(close1){
      String msg="";
      msg+="S"; 
      char sendBuffer[20];
      int str_len = msg.length()+1;
      msg.toCharArray(sendBuffer, str_len);
      sendBuffer[str_len] = '\0'; //Terminate string
      lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, str_len);
      close1=false;
      DataReadAddress=0;
      connectedPhone=0;
      writeAddress=4;
      TotalData=0; 
    }
  }
  
  //紀錄
  TotalLight+=AmbientLightLux;          //非同步 一直累加
  if( mode==2)
  {
    if(RealTime==1 && count%1200!=0 )   //改成1200  =2min    //////real time的資料->圖表用
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
    else if(RealTime!=1 &&count%1200==0 &&connectedPhone!=2)    //改成1200
    {
      Serial.println(F("2 mode Begin record"));       //兩分鐘寫到PROM一次
      int readData[]={1};     //why??
      int readData2[]={1};     //why??

          Serial.print(F("Total_Light:"));
          Serial.println(TotalLight);
          Serial.println(writeAddress);
          int rootnum=0;
          rootnum=sqrt(TotalLight);
          Serial.print(F("rootLight:"));
          Serial.println(rootnum);
          int writeData[]={(rootnum)};
          //int writeData[]={(TotalLight/10)};
          EEPROMwrite(writeAddress,(uint8_t*)writeData,2);      //光資料 2個位置
          writeAddress+=2;    
          Serial.println(writeAddress);
          Serial.print(F("Steps:"));
          Serial.println(steps);
      
          int writeStepData[]={(steps/(3.5))+1};
          EEPROMwrite(writeAddress,(uint8_t*)writeStepData,2);    //步數兩個位置
          
          writeAddress+=2;
      
          Serial.print(F("TotalData:"));
          Serial.println((writeAddress-4)/2);
          TotalData=(writeAddress-4)/2;

      TotalLight=0;
      steps=1;
    } 
  }
  bleDelay(100);//delay(delaycontrol);      //改成100 
}

