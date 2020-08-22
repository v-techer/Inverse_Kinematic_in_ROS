#!/usr/bin/env python

import rospy
import subprocess
import netifaces
import re
import can
import sys
from can_pkg.srv import *
from std_msgs.msg import String

def can_exit_node():
	rospy.loginfo("CAN Node is shutting down.")

def check_interfaces(intf,bitrate):
	'''  '''
	lsmod_proc = subprocess.Popen(['ifconfig'], stdout=subprocess.PIPE)
	grep_proc = subprocess.Popen(['grep', intf], stdin=lsmod_proc.stdout)
	grep_proc.communicate()  # Block until finished
	print('Interface {} {} loaded'.format(intf, "is" if (grep_proc.returncode == 0) else "isn't"))
	if grep_proc.returncode != 0:
		try:
			proc = subprocess.check_output("sudo ip link set "+intf+" down",shell=True)
			proc = subprocess.check_output("sudo ip link set "+intf+" type can bitrate " + str(bitrate),shell=True)
			proc = subprocess.check_output("sudo ip link set "+intf+" up",shell=True)
			print('Interface {} is loaded'.format(intf))			
		except subprocess.calledprocesserror as grepexc:
			print('error code',grepexc.returncode, grepexc.output)
			return False
	return True

def module_loaded(module_name):
    """Checks if module is loaded"""
    lsmod_proc = subprocess.Popen(['lsmod'], stdout=subprocess.PIPE)
    grep_proc = subprocess.Popen(['grep', module_name], stdin=lsmod_proc.stdout)
    grep_proc.communicate()  # Block until finished
    return grep_proc.returncode == 0


def check_modules():
	''' lsmod mttcan | can; can_raw; mttcan '''

	for module_name in ['mttcan', 'can', 'can_raw','can_dev']:
		loaded = module_loaded(module_name)
		print('Module {} {} loaded'.format(module_name, "is" if loaded else "isn't"))
		if loaded == False: 
			try:
				proc = subprocess.check_output("sudo modprobe "+module_name,shell=True)
				print('Module {} is loaded'.format(module_name))			
			except subprocess.CallProcessError as grepexc:
				print('error code',grepexc.returncode, grepexc.output)
				return False
	return True



def SendGateCAN_Msg(req,bus):
	msg = can.Message(arbitration_id=req.a_id,data=req.data)
	bus.send(msg)
	rospy.loginfo("Sending %s to %s.", req.data , req.port)	
	return SendCANResponse(1)

def can_node(intf,baud):
	# node	
	bus = can.interface.Bus(bustype='socketcan', channel=intf, bitrate=baud)
	node_name = 'can_gateway' + intf	
	rospy.init_node(node_name, anonymous=True)	
	rate = rospy.Rate(10)

	# server
	callback_lambda = lambda data: SendGateCAN_Msg(data,bus)
	service_name = 'sent_to_can' + intf
	s = rospy.Service(service_name, SendCAN, callback_lambda)

	# publisher
	pub = rospy.Publisher('chatter', String, queue_size=10)
	
	rospy.loginfo("CAN Node on interface %s is ready.",intf)

	while not rospy.is_shutdown():
		message = bus.recv()
		rospy.loginfo(message)
		msg_id =  hex(message.arbitration_id)
		msg_data = ""	
		for data in message.data:
			msg_data += str(hex(data))[2:4] 
		pub.publish(hex(message.arbitration_id)+ " " + msg_data)
		rate.sleep()


if __name__ == '__main__':

	if len(sys.argv) == 3:
		intf = int(sys.argv[1])
		baud = int(sys.argv[2])
	else:
		sys.exit(1)	

	# check drivers 
	if check_modules():
		print("Driver are loaded.")
	else:
		print("Driver could not been loaded.")
		exit()

	# check interfaces	
	intf = "can1"
	baud = 250000	
	if check_interfaces(intf,baud):
		print("Interfaces are loaded.")
	else:
		print("Interfaces could not been loaded.")
		exit()

	rospy.on_shutdown(can_exit_node)
	can_node(intf,baud)

