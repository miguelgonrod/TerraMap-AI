#!/usr/bin/env python3

import rospy
from query_msg.msg import queryData
import sys


class publisher:
    def __init__(self):
        self.pub = rospy.Publisher('notifyMap', queryData, queue_size=10)
        rospy.init_node('notifier', anonymous=True)
        self.mapString = queryData()
        self.mapString.username.data = rospy.get_param('~username')
        self.mapString.mapName.data = rospy.get_param('~filename')
        self.mapString.description.data = rospy.get_param('~description')
        self.mapString.downloadIdpgm.data = ""
        self.mapString.downloadIdyaml.data = ""
        self.pub.publish(self.mapString)
 
if __name__ == '__main__':
    try:
        publicador = publisher()
    except rospy.ROSInterruptException:
        pass