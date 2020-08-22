#!/usr/bin/env python

import rospy
from std_msgs.msg import String 
from uart_pkg.srv import *
import serial as ser
import re

def usage():
	return "\nError!\n\nUsage\n"

def SendGateUART_Msg(req,uart):
	uart.write(req.data)
	rospy.loginfo("Sending %s to %s.", req.data , req.port, )	
	return SendUARTResponse(1)

def uart_exit_node():
	rospy.loginfo("UART Node is shutting down.")

def uart_node(port,baud):
	# init node
	uart = ser.Serial(port,baudrate=baud,timeout=3)
	server_name = 'uart_server' + re.sub('/','',port)
	rospy.init_node(server_name)
	rate = rospy.Rate(10) # 10hz

	# server
	callback_lambda = lambda data: SendGateUART_Msg(data,uart)
	service_name = 'sent_to_uart' + re.sub('/','',port)
	s = rospy.Service(service_name, SendUART, callback_lambda)

	# publisher
	pub = rospy.Publisher('chatter', String, queue_size=10)
   	    	
	rospy.loginfo("UART Node on port %s is ready.",port)

	while not rospy.is_shutdown():
		rcv = uart.read(size=32)
		drcv = rcv.decode()
		if len(drcv) != 0:
			rospy.loginfo(drcv)		
			pub.publish(drcv)
		rate.sleep()
		

if __name__ == "__main__":
	if len(sys.argv) == 3:
		port = int(sys.argv[1])
		baud = int(sys.argv[2])
	else:
		print usage()
		sys.exit(1)
	port = "/dev/ttyTHS2"
	baud = 9600
	rospy.on_shutdown(uart_exit_node)
	uart_node(port,baud)
