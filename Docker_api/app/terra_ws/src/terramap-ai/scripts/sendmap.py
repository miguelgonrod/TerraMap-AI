#!/usr/bin/env python3

import rospy
from query_msg.msg import queryData
import rospkg

import os
from google.oauth2 import service_account
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload

rospack = rospkg.RosPack()
package_path = rospack.get_path("terramap-ai")

class drive:
    def __init__(self):
        self.file_name = "Add the json of the drive v3 api keys"
        self.file_path = f"{package_path}/{self.file_name}"
        self.archivo_local = 'test.txt'
        self.id_carpeta_destino = 'Add destination folder id'
        self.credenciales = service_account.Credentials.from_service_account_file(
            self.file_path,
            scopes=['https://www.googleapis.com/auth/drive']
        )
        
        self.drive_service = build('drive', 'v3', credentials=self.credenciales)
        
        self.archivo_drive = {
            'name': os.path.basename(self.archivo_local),
            'parents': [self.id_carpeta_destino]
        }
        
    def setFile(self, file):
        self.archivo_local = file
        self.archivo_drive = {
            'name': os.path.basename(self.archivo_local),
            'parents': [self.id_carpeta_destino]
        }
        
    def upload(self):
        media = MediaFileUpload(self.archivo_local, resumable=True)

        try:
            archivo_subido = self.drive_service.files().create(
                body=self.archivo_drive,
                media_body=media,
                fields='id'
            ).execute()
            
            print(f'Archivo subido con éxito. ID: {archivo_subido["id"]}')
            return archivo_subido["id"]
            
        except Exception as e:
            print(f'Error al subir el archivo: {e}')
            return 0

class publisher:
    def __init__(self):
        self.pub = rospy.Publisher('sqlQuery', queryData, queue_size=10)
        self.mapString = queryData()
    
    def publish(self, username, filename, description, downloadIdpgm, downloadIdyaml):
        self.mapString.username.data = username
        self.mapString.mapName.data = filename
        self.mapString.description.data = description
        self.mapString.downloadIdpgm.data = downloadIdpgm
        self.mapString.downloadIdyaml.data = downloadIdyaml
        self.pub.publish(self.mapString)
        print("Nodo publicado con exito")

class subscriber:
    def __init__(self):
        rospy.init_node('drive', anonymous=True)
        rospy.Subscriber("notifyMap", queryData, self.callback)
        self.publicador = publisher()
        rospy.spin()

    def callback(self, data):
        archivo = drive()
        archivo.setFile("/maps/" + data.mapName.data + ".yaml", )
        idYaml = archivo.upload()
        archivo.setFile("/maps/" + data.mapName.data + ".pgm")
        idPgm = archivo.upload()
        
        if idYaml != "" and idPgm != "":
            self.publicador.publish(data.username.data, data.mapName.data, data.description.data, str(idPgm), str(idYaml))


if __name__ == '__main__':
    suscriptor = subscriber()
