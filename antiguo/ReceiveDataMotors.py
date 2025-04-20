import time
import serial   
import mysql.connector                                    
from datetime import datetime
import time
import keyboard
import math
import re
#-------------------------------------------------------------------------#
# ModbusSerialClient Configurations From Tiva
#-------------------------------------------------------------------------#
ser=serial.Serial(
port='/dev/ttyUSB0',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)


#-------------------------------------------------------------------------#
# Connecting Database MySql
#-------------------------------------------------------------------------#
mydb = mysql.connector.connect(
    host="localhost",
    user="usuario",
    password="Edsonj*1",
    database="RemoteData"
)
mycursor = mydb.cursor()


#-------------------------------------------------------------------------#
# Reading Data
#-------------------------------------------------------------------------#
while True:
	for i in range(1,3):
		x=str(ser.readline())
	#print(x) #temp

	SplitData=x.split(",")
	print(SplitData)



    #------------------------------------------------#
    # Managing Data from Reading  
    #------------------------------------------------#

	'''Convert each data to float type'''
	m1_L0=re.findall(r"-?\d+\.?\d*",SplitData[0])
	m2_L0=re.findall(r"-?\d+\.?\d*",SplitData[1])
	m3_L0=re.findall(r"-?\d+\.?\d*",SplitData[2])

	m1_L1=[float(s) for s in m1_L0]
	m2_L1=[float(s) for s in m2_L0]
	m3_L1=[float(s) for s in m3_L0]


	m1=float(m1_L1[0])
	m2=float(m2_L1[0])
	m3=float(m3_L1[0])

	#print(m3)
	print("Mizq: "+str(m1)+" Mder: "+str(m2)+" Yaw: "+str(m3))


	'''Return the current UTC date and time for each measure'''
	MeasureTime = datetime.utcnow()
	MeasureTime2 = datetime.utcnow()
    
    #------------------------------------------------#
    # Insert Data Into Database
    #------------------------------------------------#
   
	'''Inser data into database table "Motors" '''

	sql = "INSERT INTO Motors (LeftMotor, RightMotor, Yaw, time, time2) VALUES (%s, %s, %s, %s, %s)"
	valv = (m1,m2,m3,MeasureTime,MeasureTime2)
	mycursor.execute(sql,valv)
	mydb.commit()
	print("1 record inserted, ID:", mycursor.lastrowid)
