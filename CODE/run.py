#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import matplotlib.pyplot as plt
import serial
import mysql.connector
from mysql.connector import Error
import datetime
#no module named serial -> pip install pyserial


# In[2]:


PORT = '/dev/cu.DEV1'                   
BAUD_RATES = 9600  
ser = serial.Serial(PORT,BAUD_RATES) 


# In[20]:


def connect_db():
    try:
        connection = mysql.connector.connect(
            host='35.77.173.217', #資料庫的host
#             port='3306', #資料庫的port (通常會是3306)
            database='Laundry', #資料庫名稱
            user='laundry', #資料庫使用者
            password='12345678', #資料庫密碼
        )
        if connection.is_connected():
            cursor = connection.cursor()

    except Error as e:
        print('資料庫連接失敗,'+'\n'+'錯誤代碼為 '+e)

    return connection, cursor

def get_current_time():
    current_time = str(datetime.datetime.now()).split('.')[0]
    return current_time


# In[21]:


CONNECTION, CURSOR = connect_db()


# In[28]:


ser.flushInput()
while True:
    DATA = []
    data_raw = ser.readline()
    data = data_raw.decode()  
    DATA = data.split()
    print(len(DATA))
    # try:
    #     if len(DATA) == 5:
    #         time = get_current_time()+str(DATA[1])
    #         sql = "INSERT INTO `data` (`device`,`time`,`x`,`y`,`z`) VALUES ('"+DATA[0]+"', '"+time+"', '"+DATA[2]+"', '"+DATA[3]+"','"+DATA[4]+"'); "
    #         CURSOR.execute(sql)
    #         CONNECTION.commit()
    # except Error as e:
    #     print(e)


# In[8]:


data_length = 80000#自行修改測量次數
data_arr = ['']*(data_length+1)
ser.flushInput()
for i in range(1,data_length+1):
    data_raw = ser.readline()
    data = data_raw.decode()  
    data_arr[i] = data
    print(data)


# In[ ]:


# Write result to Excel
# import pandas as pd
# df = pd.DataFrame({'Frequency xyz': data_arr})
# writer = pd.ExcelWriter('exp1.xlsx', engine='xlsxwriter')
# df.to_excel(writer, sheet_name='Sheet1')
# writer.save()
#no module named pandas -> pip install pandas
import csv
import numpy
with open(r'0906_kc_nano_result_1.csv', 'wt',newline ='') as file:
    mywriter = csv.writer(file)
    for i in range(1,len(data_arr)):
        dataraw = data_arr[i][:-2]
        datat = dataraw.split()
        print((datat))
        mywriter.writerow(datat)


# In[2]:


PORT_1 = '/dev/cu.DEV1'                   
BAUD_RATES_1 = 9600  
ser1 = serial.Serial(PORT_1,BAUD_RATES_1) 


# In[4]:


PORT_2 = '/dev/cu.DEV2'                  
BAUD_RATES_2 = 9600  
ser2 = serial.Serial(PORT_2,BAUD_RATES_2) 


# In[5]:


PORT_3 = '/dev/cu.DEV3'                   
BAUD_RATES_3 = 9600  
ser3 = serial.Serial(PORT_3,BAUD_RATES_3) 


# In[6]:


data_length = 80000#自行修改測量次數
data_arr1 = ['']*(data_length+1)
data_arr2 = ['']*(data_length+1)
data_arr3 = ['']*(data_length+1)
ser1.flushInput()
ser2.flushInput()
ser3.flushInput()

for i in range(1,data_length+1):
    print("DEV1:",end=" ")
    data_raw1 = ser1.readline()
    data1 = data_raw1.decode()  
    data_arr1[i] = data1
    print(data1)
    print("DEV2:",end=" ")
    data_raw2 = ser2.readline()
    data2 = data_raw2.decode()  
    data_arr2[i] = data2
    print(data2)
    print("DEV3:",end=" ")
    data_raw3 = ser3.readline()
    data3 = data_raw3.decode()  
    data_arr3[i] = data3
    print(data3)


# In[7]:


import csv
import numpy

with open(r'0908_kc_nano_result_1.csv', 'wt',newline ='') as file1:
    mywriter1 = csv.writer(file1)
    for i in range(1,len(data_arr1)):
        dataraw1 = data_arr1[i][:-2]
        datat1 = dataraw1.split()
        print((datat1))
        mywriter1.writerow(datat1)
with open(r'0908_kc_nano_result_2.csv', 'wt',newline ='') as file2:
    mywriter2 = csv.writer(file2)
    for j in range(1,len(data_arr2)):
        dataraw2 = data_arr2[j][:-2]
        datat2 = dataraw2.split()
        print((datat2))
        mywriter2.writerow(datat2)
with open(r'0908_kc_nano_result_3.csv', 'wt',newline ='') as file3:
    mywriter3 = csv.writer(file3)
    for k in range(1,len(data_arr3)):
        dataraw3 = data_arr3[k][:-2]
        datat3 = dataraw3.split()
        print((datat3))
        mywriter3.writerow(datat3)


# In[ ]:



try:[]
    while True:
        while ser.inWaiting():          # 若收到序列資料…
            print("data coming")
            data_raw = ser.readline()  # 讀取一行
            data = data_raw.decode()   # 用預設的UTF-8解碼
            print('接收到的原始資料：', data_raw)
            print('接收到的資料：', data)

except KeyboardInterrupt:
    ser.close()    # 清除序列通訊物件
    print('再見！')


# In[ ]:


import bluetooth

bd_addr = "21:14:10:05:16:32"

port = 1

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((bd_addr, port))

sock.send("1")

while(True):
    data = sock.recv(4096)

    print(data)

sock.close()


# In[ ]:


import serial.tools
import serial
from serial.tools import list_ports

ports = serial.tools.list_ports.comports()
serialinst = serial.Serial()

portlist = []

for port in ports:
    portlist.append(str(port))
    print(str(port))

serialinst.port = '/dev/cu.ldy'
serialinst.baudrate = 9600
serialinst.open()
while True:
    if serialinst.in_waiting:
        packet = serialinst.readline()
        print(packet.decode('utf-8'))


# In[ ]:




