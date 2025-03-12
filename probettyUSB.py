import funciones
import time

# PRIMERA PRUEBA: probar recepcion de VN300
try:
    yaw,pitch,roll,latitude,\
	longitude,altitude,velNedX,velNedY,\
	velNedZ,accBodX,accBodY,accBodZ,\
	angRateX,angRateY,angRateZ = funciones.getDataVN300()
    print(str(yaw)+","+str(pitch)+","+str(roll)+","+str(latitude)+","+str(longitude)+","+str(altitude)+","+str(velNedX)+","+str(velNedY)+","+str(velNedZ)+","+str(accBodX)+","+str(accBodY)+","+str(accBodZ)+","+str(angRateX)+","+str(angRateY)+","+str(angRateZ))    
    print(" Primera prueba correcta.")
    
except:
    print("Primera prueba error")

# SEGUNDA PRUEBA: probar envio de datos Seriales por el puerto
try:
    M_i,M_d = 0,0
    funciones.enviar(M_i,M_d)

    print("Segunda prueba correcta")
except:
    print("Segunda prueba error")

