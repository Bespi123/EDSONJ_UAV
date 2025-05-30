import time
import serial
import time
import keyboard
import math
import re
import numpy as np



#-------------------------------------------------------------------------#
# Configuracion Serial arduino
#-------------------------------------------------------------------------#
# consultar dmesg | grep tty
# pl2303 ttyUSB 0 1 o 2
serPL=serial.Serial(
	port='/dev/ttyUSB0',
	baudrate=115200,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
)

#-------------------------------------------------------------------------#
# Configura Serial VN-300
#-------------------------------------------------------------------------#
ser=serial.Serial(
	port='/dev/ttyUSB1',
	baudrate=115200,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
)

#-------------------------------------------------------------------------#
# Funciones
#-------------------------------------------------------------------------#
#---------------------        CHECKSUM       -----------------------------#
def checksum(txt):
    tam = len(txt)
    cksum=0
    txt=txt.encode()
    
    
    for i in range(2,tam):
        cksum = cksum^txt[i]

    return hex(cksum)

#---------------------      SEND SERIAL            ---------------------#
# Funciona enviar serial
    
def enviar(a,b):    # para flotantes
	a = "{:.2f}".format(a)
	a = float(a)
    
	b = "{:.2f}".format(b)
	b = float(b)
    
	a = signoNum(a)
	b = signoNum(b)
    
	c = a + "," + b +"\r\n"
	serPL.write(c.encode())
	#print(c.encode())

def signoNum(a):

    # agregarle el signo
    if (a>0):
        c = str(a)
        b="+"+ c.zfill(6)
        #print(b)
    else:
        c = str(a*-1)
        b = "-"+ c.zfill(6)
    
    return b
    




Y=[0,0]
#--------------      read VN300 + checksum + Hinf      -----------------#
def dataVN300(inUref,inRref):
	


	x=str(ser.readline())

    #------------------------------------------------#
    # Checksum
    #------------------------------------------------#
	try:
		# Discriminar $ y *-> de cadena
		T1= re.split('\$', x)	# TIPO LISTA
		T1=''.join(T1) 		# TIPO STRING

		T2= re.split('\*', T1) 	#TIPO LISTA
		T2=''.join(T2[0])   	#TIPO STRING
		
		# LEER CHECKSUM DE VN300
		T3=re.split('\*', x) #TIPO LISTA
		T3=''.join(T3[1])   #TIPO STRING
		T3="0X"+T3
		T3=T3.rstrip("b'\\r\\n")
	except:
		#print("error try")
		#return(Y[0]+Y[1],Y[0]-Y[1])	# motores guardados
		return(0,0)	# motores guardados
	else:

		# Calculo CRC
		crc = checksum(T2)
		crc = crc.upper()
		
		if(crc!=T3):	# si no se cumple, reinicia el while(1)
			#print("NO Checksum")
			#return(Y[0]+Y[1],Y[0]-Y[1])	# motores guardados
			return(0,0)	# motores guardados
		else:
			
		#print("correcto")


	#------------------------------------------------#
	# Managing Data from Reading  
	#------------------------------------------------#

			SplitData=x.split(",")
			#print(SplitData)

			'''Convert each data to float type'''
			
		
			m2_L0=re.findall(r"-?\d+\.?\d*",SplitData[1])
			m3_L0=re.findall(r"-?\d+\.?\d*",SplitData[2])
			m4_L0=re.findall(r"-?\d+\.?\d*",SplitData[3])
			m5_L0=re.findall(r"-?\d+\.?\d*",SplitData[4])
			m6_L0=re.findall(r"-?\d+\.?\d*",SplitData[5])
			m7_L0=re.findall(r"-?\d+\.?\d*",SplitData[6])
			m8_L0=re.findall(r"-?\d+\.?\d*",SplitData[7])
			m9_L0=re.findall(r"-?\d+\.?\d*",SplitData[8])
			m10_L0=re.findall(r"-?\d+\.?\d*",SplitData[9])
			m11_L0=re.findall(r"-?\d+\.?\d*",SplitData[10])
			m12_L0=re.findall(r"-?\d+\.?\d*",SplitData[11])
			m13_L0=re.findall(r"-?\d+\.?\d*",SplitData[12])
			m14_L0=re.findall(r"-?\d+\.?\d*",SplitData[13])
			m15_L0=re.findall(r"-?\d+\.?\d*",SplitData[14])
			m16_L0=re.findall(r"-?\d+\.?\d*",SplitData[15])
		
		
			m2_L1=[float(s) for s in m2_L0]
			m3_L1=[float(s) for s in m3_L0]
			m4_L1=[float(s) for s in m4_L0]
			m5_L1=[float(s) for s in m5_L0]
			m6_L1=[float(s) for s in m6_L0]
			m7_L1=[float(s) for s in m7_L0]
			m8_L1=[float(s) for s in m8_L0]
			m9_L1=[float(s) for s in m9_L0]
			m10_L1=[float(s) for s in m10_L0]
			m11_L1=[float(s) for s in m11_L0]
			m12_L1=[float(s) for s in m12_L0]
			m13_L1=[float(s) for s in m13_L0]
			m14_L1=[float(s) for s in m14_L0]
			m15_L1=[float(s) for s in m15_L0]
			m16_L1=[float(s) for s in m16_L0]
			

			yaw = float(m2_L1[0])
			pitch = float(m3_L1[0])
			roll = float(m4_L1[0])
			latitude = float(m5_L1[0])
			longitude=float(m6_L1[0])
			altitude=float(m7_L1[0])
			velNedX=float(m8_L1[0])
			velNedY=float(m9_L1[0])
			velNedZ=float(m10_L1[0])
			accBodX=float(m11_L1[0])
			accBodY=float(m12_L1[0])
			accBodZ=float(m13_L1[0])
			angRateX=float(m14_L1[0])
			angRateY=float(m15_L1[0])
			angRateZ=float(m16_L1[0])
			
			#print(str(yaw)+","+str(pitch)+","+str(roll)+","+str(latitude)+","+str(longitude)+","+str(altitude)+","+str(velNedX)+","+str(velNedY)+","+str(velNedZ)+","+str(accBodX)+","+str(accBodY)+","+str(accBodZ)+","+str(angRateX)+","+str(angRateY)+","+str(angRateZ))

			''' Velocidad Body '''
			
			a11 = math.cos (yaw) * math.cos (pitch)
			a12 = (math.cos (yaw) * math.sin (pitch) * math.sin (roll)) - (math.sin (yaw) * math.cos (roll));
			a13 = (math.sin (yaw) * math.sin (roll) + math.cos (yaw) * math.cos (roll) * math.sin (pitch));
			
			b11 = math.sin (yaw) * math.cos (pitch);
			b12 = (math.cos (yaw) * math.cos (roll)) + (math.sin (roll) * math.sin (pitch) * math.sin (yaw));
			b13 = (math.cos (roll) * math.sin (pitch) * math.sin (yaw)) + (-math.cos (yaw) * math.sin (roll));

			c11 = -math.sin (pitch);
			c12 = math.cos (pitch) * math.sin (roll);
			c13 = math.cos (roll) * math.cos (pitch);

			J = np.array([[a11, a12, a13],[b11, b12, b13],[c11, c12, c13]])
			#print(J)

			V = np.array([[velNedX],[velNedY],[velNedZ]]) # (3,1)
			#print(V)

			velBody = np.dot(J,V)
			#print(velBody)

			''' H INFINITO '''

			A = np.array([[0.3166 ,0.000495010414223325,-0.00333937125248452,2.40286944609632,0.00915814246877045,0.117761141167872,0.00251517798550882],[5.40741300780403e-05,0.951196996210378,0.368093871737200,0.000618441743298645,-0.900636777065195,3.57860864560393e-05,-0.00645036014358575],[-4.27350165734011e-05,0.0360043397301523,0.554595644477981,-0.000722158921562911,0.963900814191687,-3.29814404550843e-05,0.000929263035261145],[9.74738130766904e-15  ,-2.22819156643580e-12,1.57013040396079e-11,0.999200319872053,-4.00955379544565e-11,3.58299359766609e-15,-2.08325415421773e-12],[3.11688449289474e-18 ,-7.99549564875759e-16,5.63612847760353e-15,1.65228730201321e-18,0.999800019997986 ,1.23234688015350e-18,-7.47578218180561e-16],[-2.49257515829194,0.0378103562793925,-0.238002452286121,9.10513700978578,0.635720173625357,-0.552693668407488,0.0105269194311767],[0.000912729050256367,-1.68639408429943,10.4202823469188,0.0223536447875703,-27.9518222535072,0.000822827292531607,-0.864097527716423]]) # (7,7)

			B = np.array([[0.0478248439764841,0.000226923346973228,0.0479812984988504,0.000225387015743105],[7.58036698106008e-06,-0.00715146054163218,1.06977350092788e-05,-0.00716947335481180],[-1.18116460804164e-05,0.00881009242714553,-1.35363125298376e-05,0.00881887300104030],[0.0399840063974410,2.73110240984954e-14,0.0399840063974410,2.72375971802069e-14],[-1.06353410465807e-19,0.0199980001999800,-2.14063440481788e-20,0.0199980001999800],[0.182542457319418,0.00627813318560780,0.182246609054090,0.00627920252621309],[0.000469858641990445,-0.287109621838555,0.000454949852263351,-0.287030830780295]]) # (7,4)

			C = np.array([[-190.168868235004,2.88470846597925,-18.1581914743234,694.668561754478,48.5017214146264,34.1264686476395,0.803142223241827],[0.0278549764551581,-51.4655,318.009730651167,0.682196155660706,-853.043244905504,0.0251113239506201,4.14599212174637]]) # (2,7)

			D = np.array([[13.9269190731477,0.478984747386228,13.9043475853434,0.479066331803849],[0.0143393062883452,-8.76210936215517,0.0138843147585955,-8.75970478976636]])  # (2,4)
			
			# Valores iniciales
			X = np.array([[0],[0],[0],[0],[0],[0],[0]])   #(7,1)

			# Matriz U
			Ur = inUref # Referencia Surge  m/s
			Rr = inRref # Referencia AngularRate rad/s


			
			U = np.array([[Ur],[Rr],[velBody[0]],[angRateZ]])  # (4,1)
			
			# Producto estados
			Y = np.dot(C,X) + np.dot(D,U)  # (2,1)
			X = np.dot(A,X) + np.dot(B,U)

			# Salida controlador
			#MotorIzq=Y[0]+Y[1]
			#MotorDer=Y[0]-Y[1]
			#print("nc:"+str(Y[0])+"nd:"+str(Y[1]))
			#print("Motor Izq:"+str(MotorIzq)+" Motor Der:"+str(MotorDer))
			M_i=float(Y[0]+Y[1])
			M_d=float(Y[0]-Y[1])

			return(M_i,M_d)

#--------------      read VN300 + ZIGZAG      -----------------#
def zigzag(grados):
	


	x=str(ser.readline())

    #------------------------------------------------#
    # Checksum
    #------------------------------------------------#
	try:
		# Discriminar $ y *-> de cadena
		T1= re.split('\$', x)	# TIPO LISTA
		T1=''.join(T1) 		# TIPO STRING

		T2= re.split('\*', T1) 	#TIPO LISTA
		T2=''.join(T2[0])   	#TIPO STRING
		
		# LEER CHECKSUM DE VN300
		T3=re.split('\*', x) #TIPO LISTA
		T3=''.join(T3[1])   #TIPO STRING
		T3="0X"+T3
		T3=T3.rstrip("b'\\r\\n")
	except:
		#print("error try")
		#return(Y[0]+Y[1],Y[0]-Y[1])	# motores guardados
		return(0,0)	# motores guardados
	else:

		# Calculo CRC
		crc = checksum(T2)
		crc = crc.upper()
		
		if(crc!=T3):	# si no se cumple, reinicia el while(1)
			#print("NO Checksum")
			#return(Y[0]+Y[1],Y[0]-Y[1])	# motores guardados
			return(0,0)	# motores guardados
		else:
			
		#print("correcto")


	#------------------------------------------------#
	# Managing Data from Reading  
	#------------------------------------------------#

			SplitData=x.split(",")
			#print(SplitData)

			'''Convert each data to float type'''
			
		
			m2_L0=re.findall(r"-?\d+\.?\d*",SplitData[1])
			m3_L0=re.findall(r"-?\d+\.?\d*",SplitData[2])
			m4_L0=re.findall(r"-?\d+\.?\d*",SplitData[3])
			m5_L0=re.findall(r"-?\d+\.?\d*",SplitData[4])
			m6_L0=re.findall(r"-?\d+\.?\d*",SplitData[5])
			m7_L0=re.findall(r"-?\d+\.?\d*",SplitData[6])
			m8_L0=re.findall(r"-?\d+\.?\d*",SplitData[7])
			m9_L0=re.findall(r"-?\d+\.?\d*",SplitData[8])
			m10_L0=re.findall(r"-?\d+\.?\d*",SplitData[9])
			m11_L0=re.findall(r"-?\d+\.?\d*",SplitData[10])
			m12_L0=re.findall(r"-?\d+\.?\d*",SplitData[11])
			m13_L0=re.findall(r"-?\d+\.?\d*",SplitData[12])
			m14_L0=re.findall(r"-?\d+\.?\d*",SplitData[13])
			m15_L0=re.findall(r"-?\d+\.?\d*",SplitData[14])
			m16_L0=re.findall(r"-?\d+\.?\d*",SplitData[15])
		
		
			m2_L1=[float(s) for s in m2_L0]
			m3_L1=[float(s) for s in m3_L0]
			m4_L1=[float(s) for s in m4_L0]
			m5_L1=[float(s) for s in m5_L0]
			m6_L1=[float(s) for s in m6_L0]
			m7_L1=[float(s) for s in m7_L0]
			m8_L1=[float(s) for s in m8_L0]
			m9_L1=[float(s) for s in m9_L0]
			m10_L1=[float(s) for s in m10_L0]
			m11_L1=[float(s) for s in m11_L0]
			m12_L1=[float(s) for s in m12_L0]
			m13_L1=[float(s) for s in m13_L0]
			m14_L1=[float(s) for s in m14_L0]
			m15_L1=[float(s) for s in m15_L0]
			m16_L1=[float(s) for s in m16_L0]
			

			yaw = float(m2_L1[0])
			pitch = float(m3_L1[0])
			roll = float(m4_L1[0])
			latitude = float(m5_L1[0])
			longitude=float(m6_L1[0])
			altitude=float(m7_L1[0])
			velNedX=float(m8_L1[0])
			velNedY=float(m9_L1[0])
			velNedZ=float(m10_L1[0])
			accBodX=float(m11_L1[0])
			accBodY=float(m12_L1[0])
			accBodZ=float(m13_L1[0])
			angRateX=float(m14_L1[0])
			angRateY=float(m15_L1[0])
			angRateZ=float(m16_L1[0])
			'''
			if((-179+grados)<yaw and yaw<(179-grados)): # caso 1
				
			elif (-179<yaw and yaw<(-179+grados)): # caso 2

			elif ((179-grados)<yaw and yaw<179): # caso 3
			'''
#-------------------------------------------------------------------------#
# Inicio de bucle
#-------------------------------------------------------------------------#

while(1):
	f=open("log.txt","r")
	key=f.readlines()
	key=key[-1]	# string de 1 caracter con \n
	
	f.close()
	#print(type(key)) # tecla detectada
	

	# modo manual
	if key == "B\n": # break
		M_i,M_d = 0,0
		enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))

	elif key == "W\n": # up
		M_i,M_d = 60,60
		enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))

	elif key == "S\n": # back
		M_i,M_d = -120,-120
		enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))

	elif key == "A\n": # left
		M_i,M_d = 0,60
		enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))
	
	elif key == "D\n": # right
		M_i,M_d = 60,0
		enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))

	elif key == "M\n": # Horario
		M_i,M_d = 60,-60
		enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))
 
	elif key == "N\n": # Antihorario
		M_i,M_d = -60,60
		enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))


	# maniobras zigzag
	elif key == "F\n": # ZZ 10
		print("")		
	elif key == "G\n": # ZZ 15
		print("")
	elif key == "H\n": # ZZ 20
		print("")
	elif key == "J\n": # ZZ 25
		print("")
	elif key == "K\n": # ZZ 30
		print("")	

	# maniobras control
	elif key == "Y\n": # K=0.5
		M_i,M_d=dataVN300(1,0) # Surge ref, AngRate ref
		K=0.5		
		enviar(M_i*K,M_d*K)
		print("Motor Izq:"+str(M_i*K)+"  Motor Der:"+str(M_d*K))

	elif key == "U\n": # K=0.75
		M_i,M_d=dataVN300(1,0) # Surge ref, AngRate ref
		K=0.75	
		enviar(M_i*K,M_d*K)
		print("Motor Izq:"+str(M_i*K)+"  Motor Der:"+str(M_d*K))

	elif key == "I\n": # K=1
		M_i,M_d=dataVN300(1,0) # Surge ref, AngRate ref
		K=1	
		enviar(M_i*K,M_d*K)
		print("Motor Izq:"+str(M_i*K)+"  Motor Der:"+str(M_d*K))
		#print(type(M_i))

	elif key == "O\n": # K=1.25
		M_i,M_d=dataVN300(1,0) # Surge ref, AngRate ref
		K=1.25	
		enviar(M_i*K,M_d*K)
		print("Motor Izq:"+str(M_i*K)+"  Motor Der:"+str(M_d*K))
	
	elif key == "P\n": # K=1.5
		M_i,M_d=dataVN300(1,0) # Surge ref, AngRate ref
		K=1.5	
		enviar(M_i*K,M_d*K)
		print("Motor Izq:"+str(M_i*K)+"  Motor Der:"+str(M_d*K))

	
	time.sleep(0.025) # cada 40 Hz (modificable)
