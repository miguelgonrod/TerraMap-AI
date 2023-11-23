#!/usr/bin/env python3

import rospy
from query_msg.msg import queryData

class publisher:
    def __init__(self):
        self.pub = rospy.Publisher('notifyMap', queryData, queue_size=10)
        self.mapString = queryData()
    
    def publish(self, username="", filename="", description="", downloadIdpgm="", downloadIdyaml="", caso = 0):
        self.mapString.username.data = username
        self.mapString.mapName.data = filename
        self.mapString.description.data = description
        self.mapString.downloadIdpgm.data = downloadIdpgm
        self.mapString.downloadIdyaml.data = downloadIdyaml
        print("Ejecucion caso: " + str(caso))
        try:
            self.pub.publish(self.mapString)
            print("Nodo publicado con exito")
        
        except Exception as e:
            print(f'Error al subir el archivo: {e}')

if __name__ == '__main__':
    publicador = publisher()
    publicador.publish("miguel", "mapaPrueba", "descripcion de prueba", "", "", 1)
    publicador.publish("miguel", "hola", "descripcion de prueba", "", "", 2)
    publicador.publish("miguel", "mapaPrueba", "descripcion de prueba", "", "", 3)