#-------------------------------------------------------------------------#
# Create Table
#-------------------------------------------------------------------------#
import mysql.connector
mydb = mysql.connector.connect(
    host="localhost",
    user="usuario",
    password="Edsonj*1",
    database="RemoteData"
)
mycursor = mydb.cursor()

'''Vectors Table'''
mycursor.execute("CREATE TABLE Vectors (id INT AUTO_INCREMENT PRIMARY KEY, yaw FLOAT(25), pitch FLOAT(25), roll FLOAT(25), latitude FLOAT(25), longitude FLOAT(25), altitude FLOAT(25), velocityx FLOAT(25), velocityy FLOAT(25), velocityz FLOAT(25), accelx FLOAT(25), accely FLOAT(25), accelz FLOAT(25), angularratex FLOAT(25), angularratey FLOAT(25), angularratez FLOAT(25), time DATETIME NOT NULL, time2 DATETIME(3) )")
#mycursor.execute("ALTER TABLE Vectors ADD COLUMN id INT AUTO_INCREMENT PRIMARY KEY")
