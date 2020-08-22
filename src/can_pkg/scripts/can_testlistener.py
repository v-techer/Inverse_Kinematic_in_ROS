#!/usr/bin/env python
# Software License Agreement (BSD License)

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import re
import sys
from can_pkg.srv import *
from std_msgs.msg import String

def usage():
	return "%s  [x,y]"%sys.argv[0]

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)


def SendCANMsgToServer(inf,data,a_id):
	print "Sending CAN Message to %s"%inf
	service_name = 'sent_to_can' + re.sub('/','',inf)
	rospy.wait_for_service(service_name)
	try:
		send_gateway = rospy.ServiceProxy(service_name,SendCAN)
		baud = 250000
		gateway = send_gateway(inf,baud,data,a_id,inf)
		print "%s"%gateway.Resp
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


def SubscribeCANMsg():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('chatter', String, callback)
	rospy.spin()

if __name__ == '__main__':
	if len(sys.argv) == 3:
		intf = int(sys.argv[1])
		data = int(sys.argv[2]) 		  
	else:
		print usage()
		sys.exit(1)

	intf = "can1"
	data = "test"
	a_id = 0x1a
	SendCANMsgToServer(intf,data,a_id)
	#SubscribeCANMsg()

