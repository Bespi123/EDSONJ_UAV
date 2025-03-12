# Hecho por: Jorch Mendoza Ch.
import funciones

import time
import mysql.connector                                    
from datetime import datetime



#-------------------------------------------------------------------------#
# Connecting Database MySql
#-------------------------------------------------------------------------#
mydb = mysql.connector.connect(
    host="localhost",
    user="edsonj",
    password="password",
    database="RemoteData"
)
mycursor = mydb.cursor()


flg = True
keyRight = False
keyLeft = False
yawref = 0
M_i,M_d=0,0
#-------------------------------------------------------------------------#
# Inicio de bucle
#-------------------------------------------------------------------------#

while(1):
	f=open("log.txt","r")
	key=f.readlines()
	key=key[-1]	# string de 1 caracter con \n
	
	f.close()
	#print(type(key)) # tecla detectada
	
	try:	
		yaw,pitch,roll,latitude,\
		longitude,altitude,velNedX,velNedY,\
		velNedZ,accBodX,accBodY,accBodZ,\
		angRateX,angRateY,angRateZ = funciones.getDataVN300()
	except:
		continue

	#--------------------------------------------------------------------#
	# Modo manual
	#--------------------------------------------------------------------#
	
	if key == "B\n": # break
		M_i,M_d = 0,0
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))

	elif key == "W\n": # up
		M_i,M_d = 60,60
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))

	elif key == "S\n": # back
		M_i,M_d = -120,-120
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))

	elif key == "A\n": # left
		M_i,M_d = 0,60
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))
	
	elif key == "D\n": # right
		M_i,M_d = 60,0
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))

	elif key == "M\n": # Horario
		M_i,M_d = 60,-60
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))
 
	elif key == "N\n": # Antihorario
		M_i,M_d = -60,60
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))


	#--------------------------------------------------------------------#
	# Maniobras zigzag
	#--------------------------------------------------------------------#
	
	elif key == "F\n": # ZZ 10
		alpha=10 # 

		if flg:  # solo se ejecuta 1 vez
			flg,yawref,keyLeft,keyRight = funciones.onOncetime(keyRight,keyLeft,yaw) 
			caso = funciones.detectCase(yawref,alpha)
		#print("caso: "+str(caso)+" keyR: "+str(keyRight)+" keyL: "+str(keyLeft))

		M_i,M_d,keyLeft,keyRight = funciones.zigzagMov(keyRight,keyLeft,caso,yaw,alpha,yawref)
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d)+"\r\n")
		#print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d)+"  yawref:"+str(yawref)+"  yaw:"+str(yaw)+"  alpha:"+str(alpha))

	elif key == "G\n": # ZZ 15
		alpha=15 # 

		if flg:  # solo se ejecuta 1 vez
			flg,yawref,keyLeft,keyRight = funciones.onOncetime(keyRight,keyLeft,yaw) 
			caso = funciones.detectCase(yawref,alpha)

		M_i,M_d,keyLeft,keyRight = funciones.zigzagMov(keyRight,keyLeft,caso,yaw,alpha,yawref)
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))


	elif key == "H\n": # ZZ 20
		alpha=20 # 

		if flg:  # solo se ejecuta 1 vez
			flg,yawref,keyLeft,keyRight = funciones.onOncetime(keyRight,keyLeft,yaw) 
			caso = funciones.detectCase(yawref,alpha)

		M_i,M_d,keyLeft,keyRight = funciones.zigzagMov(keyRight,keyLeft,caso,yaw,alpha,yawref)
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))


	elif key == "J\n": # ZZ 25
		alpha=25 # 

		if flg:  # solo se ejecuta 1 vez
			flg,yawref,keyLeft,keyRight = funciones.onOncetime(keyRight,keyLeft,yaw) 
			caso = funciones.detectCase(yawref,alpha)

		M_i,M_d,keyLeft,keyRight = funciones.zigzagMov(keyRight,keyLeft,caso,yaw,alpha,yawref)
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))


	elif key == "K\n": # ZZ 30
		alpha=30 # 

		if flg:  # solo se ejecuta 1 vez
			flg,yawref,keyLeft,keyRight = funciones.onOncetime(keyRight,keyLeft,yaw) 
			caso = funciones.detectCase(yawref,alpha)

		M_i,M_d,keyLeft,keyRight = funciones.zigzagMov(keyRight,keyLeft,caso,yaw,alpha,yawref)
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))


	#--------------------------------------------------------------------#
	# Control H Infinito
	#--------------------------------------------------------------------#
	
	elif key == "Y\n": # K=0.5
		M_i,M_d = funciones.Hinf(1,0,yaw,pitch,roll,velNedX,velNedY,velNedZ,angRateZ) # Surge ref, AngRate ref
		K=0.5		
		M_i=M_i*K
		M_d=M_d*K
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))

	elif key == "U\n": # K=0.75
		M_i,M_d = funciones.Hinf(1,0,yaw,pitch,roll,velNedX,velNedY,velNedZ,angRateZ) # Surge ref, AngRate ref
		K=0.75	
		M_i=M_i*K
		M_d=M_d*K
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))

	elif key == "I\n": # K=1
		M_i,M_d = funciones.Hinf(1,0,yaw,pitch,roll,velNedX,velNedY,velNedZ,angRateZ) # Surge ref, AngRate ref
		K=1	
		M_i=M_i*K
		M_d=M_d*K
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))
		#print(type(M_i))

	elif key == "O\n": # K=1.25
		M_i,M_d = funciones.Hinf(1,0,yaw,pitch,roll,velNedX,velNedY,velNedZ,angRateZ) # Surge ref, AngRate ref
		K=1.25	
		M_i=M_i*K
		M_d=M_d*K
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))
	
	elif key == "P\n": # K=1.5
		M_i,M_d = funciones.Hinf(1,0,yaw,pitch,roll,velNedX,velNedY,velNedZ,angRateZ) # Surge ref, AngRate ref
		K=1.5	
		M_i=M_i*K
		M_d=M_d*K
		funciones.enviar(M_i,M_d)
		print("Motor Izq:"+str(M_i)+"  Motor Der:"+str(M_d))


	if (key == "F\n" or key == "G\n" or key == "H\n" or key == "J\n" or key == "K\n" ):
		pass
	else:
		flg,keyRight,keyLeft,yawref = funciones.endZigzag()
		#print("ejecucion funcion endzigzag")

	
	#-------------------------------------------------------------------------#
	# Insert Data Into Database
    #-------------------------------------------------------------------------#
	#Return the current UTC date and time for each measure
		
	MeasureTime = datetime.utcnow()

	    
	# Insert data into database table "Vectors" 
	sql = "INSERT INTO dataEdsonJ1 (yaw, pitch, roll, latitude, \
	longitude, altitude, velNEDx, velNEDy, velNEDz, accBodX,\
	accBodY, accBodZ, angRateX, angRateY, angRateZ, motorIzq,\
	motorDer, Command, Time) \
	VALUES (%s, %s, %s, %s, %s, %s,\
	%s, %s, %s, %s, %s, %s, %s, %s,\
	%s, %s, %s, %s, %s)"
	valv = (yaw,pitch,roll,latitude,longitude,altitude,velNedX,velNedY,velNedZ,accBodX,accBodY,accBodZ,angRateX,angRateY,angRateZ,M_i,M_d,key,MeasureTime)

	mycursor.execute(sql, valv)
	mydb.commit()
	#print("1 record inserted, ID:", mycursor.lastrowid)
	time.sleep(0.035) # cada 20 Hz (modificable)
