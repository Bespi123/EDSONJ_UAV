# Hecho por: Jorch Mendoza Ch.
from pickle import TRUE

print(" ..:: Bienvenido ::.. ")
while (TRUE):
	try:
		f=open("log.txt","a")
		i=input("Introduzca comando de operacion: ")
		f.write("%s\n" % i)
		f.close()
	except KeyboardInterrupt:
		i='B'
		f.write("%s\n" % i)
		f.close()
		break
