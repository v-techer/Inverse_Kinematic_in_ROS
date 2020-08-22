#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from uart_pkg.srv import *
import re

def usage():
	return "%s  [x,y]"%sys.argv[0]

def SendUARTMsgToServer(port,baud,data):
	service_name = 'sent_to_uart' + re.sub('/','',port)
	rospy.wait_for_service(service_name)
	try:
		send_gateway = rospy.ServiceProxy(service_name,SendUART)
		gateway = send_gateway(port,baud,data)
		print "%s"%gateway.Resp
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "i heard %s",data.data)

def SubscribeUARTMsg():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("chatter",String,callback)
	rospy.spin()

if __name__ == "__main__":
	if len(sys.argv) == 4:
		port = int(sys.argv[1])
		baud = int(sys.argv[2]) 
		data = int(sys.argv[3]) 		  
	else:
		print usage()
		sys.exit(1)
	
	port = "/dev/ttyTHS2"
	baud = 9600
	data = "test"
	print "Sending UART Message to %s"%port
	SendUARTMsgToServer(port, baud,data)
	#SubscribeUARTMsg()



