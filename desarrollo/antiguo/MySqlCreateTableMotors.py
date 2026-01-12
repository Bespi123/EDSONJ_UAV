# ------------------------#
# ----- Create Table------#
# ------------------------#

import mysql.connector
mydb = mysql.connector.connect(
    host="localhost",
    user="usuario",
    password="Edsonj*1",
    database="RemoteData"
)
mycursor = mydb.cursor()

'''Motors Table'''
mycursor.execute("CREATE TABLE Motors (id INT AUTO_INCREMENT PRIMARY KEY, LeftMotor FLOAT(25), RightMotor FLOAT(25), Yaw FLOAT(25), time DATETIME NOT NULL, time2 DATETIME(3))")
#mycursor.execute("ALTER TABLE Motors ADD COLUMN id INT AUTO_INCREMENT PRIMARY KEY")
