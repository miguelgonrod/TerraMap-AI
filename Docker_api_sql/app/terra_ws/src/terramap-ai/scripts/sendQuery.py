#!/usr/bin/env python3

import rospy
from query_msg.msg import queryData
import rospkg

import mysql.connector
import json
import time    

class sql:
    def __init__(self):
        self.mydb = None
        self.mycursor = None
        self.table = None
        
    def setCredentials(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("terramap-ai")
        file_name = "Add the sql credential paths in a json format"
        file_path = f"{package_path}/{file_name}"
        
        with open(file_path) as json_file:
            data = json.load(json_file)
            self.mydb = mysql.connector.connect(
                host=data['host'],
                user=data['user'],
                password=data['password'],
                database=data['database'],
                port=data['port']
            )
            self.mycursor = self.mydb.cursor()
            self.table = data['table']
            
        json_file.close()
        
    def upload(self, username, mapName, description, downloadIdpgm, downloadIdyaml):
        select = "SELECT * FROM " + self.table + " WHERE map_name = %s"
        mapData = (mapName,)
        self.mycursor.execute(select, mapData)
        myresult = self.mycursor.fetchall()
        
        if not myresult:
            try:
                sql = "INSERT INTO " + self.table + " (username, map_name, description, download_id_pgm, download_id_yaml, created_at, updated_at) VALUES (%s, %s, %s, %s, %s, %s, %s)"
                date = time.strftime('%Y-%m-%d %H:%M:%S')
                val = (username, mapName, description, downloadIdpgm, downloadIdyaml, date, date)
                self.mycursor.execute(sql, val)
                self.mydb.commit()
                print("Mapa almacenado")
            except mysql.connector.Error as err:
                rospy.logerr(f"Error al insertar en la base de datos: {err}")

        else:
            try:
                sql = "UPDATE " + self.table + " SET updated_at = %s WHERE map_name = %s"
                date = time.strftime('%Y-%m-%d %H:%M:%S')
                val = (date)
                self.mycursor.execute(sql, val)
                self.mydb.commit()
                print("Mapa modificado")
            except mysql.connector.Error as err:
                rospy.logerr(f"Error al modificar el registro en la base de datos: {err}")

class subscriber:
    def __init__(self):
        self.sqlNode = sql()
        self.sqlNode.setCredentials()
        
        rospy.init_node('sqlNode', anonymous=True)
        rospy.Subscriber('sqlQuery', queryData, self.callback)
        rospy.spin()
        
    def callback(self, userInfo):
        self.sqlNode.upload(userInfo.username.data, userInfo.mapName.data, userInfo.description.data, userInfo.downloadIdpgm.data, userInfo.downloadIdyaml.data)
        rospy.loginfo(rospy.get_caller_id() + 'Map receeived')
        
    
if __name__ == '__main__':
    mapSubscriber = subscriber()

