import time
import serial   
import time
import keyboard
import math
import re
import numpy as np

    
#-------------------------------------------------------------------------#
# ModbusSerialClient Configurations From VN300
#-------------------------------------------------------------------------#
ser=serial.Serial(
port='/dev/ttyUSB1',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)

T2="b'VNISL,-100.281,+000.696,+000.784,+00.00000000,+000.00000000,+00000.000,+000.000,+000.000,+000.000,+00.131,-00.137,-09.663,-00.000472,-00.000372,+00.000202"

def checksum(txt):
    tam = len(txt)
    cksum=0
    txt=txt.encode()
    
    
    for i in range(2,tam):
        cksum = cksum^txt[i]

    return hex(cksum)
    

crc = checksum(T2)
print(crc)
