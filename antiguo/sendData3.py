import time
import serial

def enviar():
    ser.write(tec.encode())
    #TECLA=tec+'\r\n'
    #ser.write(TECLA.encode())
    #print('presiono '+tec)

ser=serial.Serial(
        port='/dev/ttyUSB0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
A=''
S=''
W=''
B=''
P=''
H=''
M=''
N=''
F=''
G=''
H=''
J=''
K=''
L=''
X=''
C=''
V=''
tec=''
while True:
    tec = input("digite la tecla \n")
    if tec == 'P': # closing the port
        enviar()
        printf("out of the loop")
        break
        ser.close
    elif tec == 'B': # break
        enviar()
    elif tec == 'A': # left
        enviar()
    elif tec == 'S': # back
        enviar()
    elif tec == 'D': # right
        enviar()
    elif tec == 'W': # up
        enviar()
    elif tec == 'H': # ZZ1
        enviar()
    elif tec == 'M': # MEDIA VUELTA DER
        enviar()
    elif tec == 'N': # MEDIA VUELTA IZQ
        enviar()
    elif tec == 'F': # ZZ1
        enviar()
    elif tec == 'G': # ZZ2
        enviar()
    elif tec == 'H': # ZZ3
        enviar()
    elif tec == 'J': # ZZ4
        enviar()
    elif tec == 'K': # ZZ5
        enviar()
    elif tec == 'L': # ZZ6
        enviar()
    elif tec == 'X': # K Gaint 0.5
        enviar()
    elif tec == 'C': # K Gaint 1
        enviar()
    elif tec == 'V': # K Gaint 2
        enviar()
    elif tec == '1': # function 1
        enviar()
    elif tec == '2': # function 2
        enviar()
    elif tec == '3': # function 3
        enviar()
    elif tec == '4': # function 4
        enviar()
    elif tec == '5': # function 5
        enviar()

time.sleep(1)
ser.close
