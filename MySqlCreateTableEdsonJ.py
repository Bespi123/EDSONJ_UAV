# ------------------------#
# ----- Create Table------#
# ------------------------#

import mysql.connector
mydb = mysql.connector.connect(
    host="localhost",
    user="edsonj",
    password="password",
    database="RemoteData"
)
mycursor = mydb.cursor()

'''Motors Table'''
mycursor.execute("CREATE TABLE dataEdsonJ1 (id INT AUTO_INCREMENT PRIMARY KEY, yaw FLOAT(25),\
    pitch FLOAT(25), roll FLOAT(25), latitude FLOAT(25), longitude FLOAT(25), \
    altitude FLOAT(25), velNEDx FLOAT(25), velNEDy FLOAT(25), velNEDz FLOAT(25),\
    accBodX FLOAT(25), accBodY FLOAT(25), accBodZ FLOAT(25), angRateX FLOAT(25),\
    angRateY FLOAT(25), angRateZ FLOAT(25), motorIzq FLOAT(25) ,motorDer FLOAT(25),\
    Command VARCHAR(25), time DATETIME(3) )")
#                                                                                                                                                                                                       yaw,pitch,roll,latitude,longitude,altitude,velNedX,velNedY,velNedZ,accBodX,accBodY,accBodZ,angRateX,angRateY,angRateZ
#mycursor.execute("ALTER TABLE EdsonJ ADD COLUMN id INT AUTO_INCREMENT PRIMARY KEY")
