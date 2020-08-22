#!/usr/bin/env python
# -*- coding: utf-8 -*-

print(
"""
Jetson API for sending and receiving data
---------------------------------------
European Rover Challenge
---------------------------------------
Author: Marcel Burda
Date: 13.07.2019
---------------------------------------
Version 1.0 -> copied UDP class out of Rover Ground Station project
                and created new Data Class
---------------------------------------
C TYPES (needed for un-/packing data):
b -> s int 8        B -> u int 8        e -> float 2 bytes
h -> s int 16       H -> u int 16       f -> float 4 bytes
i -> s int 32       I -> u int 32       d -> double 8 bytes
q -> s int 64       Q -> u int 64
see https://docs.python.org/3.6/library/struct.html for more
"""
)

##Imports
#Data transmitting or transforming related imports
import socket  # udp services
import time  # time functions like e.g. sleep
import threading  # multi-threading and mutex
import struct  # packing and unpacking byte objects in specified c types
from collections import deque
import numpy as np
import cmath

#Message related imports
import data_transmision_definition
#from horizon_ethernet_interface.msg import BoschSensorMessage
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
#from tf.transformations import *

##Definitions
#Initialize queue
queue = deque([])
#Received fused position
rcv_fused = False
#names and adresses
ipaddressIOcore = ''
ipAddressGroundstation = ''
ipAddressRobotArm = ''


class UdpClass:

	def __init__(self, target_address, portSend, portRecv, data_object, printing=True):
		# set host address and port
		#self.host = socket.gethostbyname(socket.gethostname())
		self.host = '192.168.0.21'
		self.portSend = portSend
		self.portRecv = portRecv
		# set global data object
		self.data = data_object  # TODO
		# create socket object with UDP services
		self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		# bind the socket object to host address and port
		self.udp_sock.bind((self.host, self.portRecv))
		# do we want to print ?
		self.printing = printing
		# print success
		if self.printing:
			print("Communication: "
					"UDP Socket successfully created ( host: '" + self.host + "', port: " + str(self.portRecv) + " )")
		# set and print target address
		self.target_address = target_address
		if self.printing:
			print("Communication: Target IP address is '" + target_address + "'")
		# initialize thread objects
		self.t1_receive = list()
		self.t2_send_cyclic = list()
		self.t2_active_threads = dict()
		# initialize mutex object (acquire: lock, release: unlock)
		self.mutex = threading.Lock()
		#flag to kill the thread
		self.forceShutdown = False

	def __receive_thread(self):

		while not self.forceShutdown:
			try:
				# Setting timeout, else the thread will be stuck at recv from if nothing is received
				self.udp_sock.settimeout(3.0)
				# receiving data
				raw_data = self.udp_sock.recvfrom(128)[0]  # 128: multiple of 2 & greater than greatest possible data length
			except socket.error as msg:
				if str(msg) == 'timed out':
					pass
				else:
					print("WARNING: __receive_thread -> " + str(msg))
			else:
				# store id
				rx_id = struct.unpack('B', raw_data[0:1])[0]
				# for loop stores the raw byte data into the global data structures
				ctr = 0
				try:
					recv_data = struct.unpack(''.join(self.data.RobotArm_ARM_GS_types), raw_data)
					#push received data set into queue
					queue.append(recv_data)
				except struct.error as msg:
					if self.printing:
						print("Warning: __receive_thread -> " + str(msg))
				else:
					self.mutex.acquire()  # enter critical section
					for key in self.data.id2keys(rx_id):
						self.data.id2keys(rx_id)[key] = recv_data[ctr]
						ctr += 1
					self.mutex.release()  # leave critical section

	def run_receive_thread(self):

		# check if thread is already active, this would shoot trouble if started multiply times
		if isinstance(self.t1_receive, list):
			# create thread object
			self.t1_receive = threading.Thread(target=self.__receive_thread, args=[])
			# start created thread
			self.t1_receive.start()
			if self.printing:
				print('Communication: Started receive thread')

	def __pack_tx_data(self, tx_id):

		# init byte object
		data_in_byte_object = bytes(0)
		# for loop, which pulls all values out of dictionary and pack them
		# into one big byte object, after that, the data is ready to get send
		if len(self.data.id2types(tx_id)) is len(self.data.id2keys(tx_id)):  # check if types match with data set
			ctr = 0
			for v in self.data.id2lists(tx_id)[0]:
				# the '>' specifies either its MSB or LSB TODO: check with your system
				if ctr == 0:
					data_in_byte_object = struct.pack(''+data.id2types(tx_id)[ctr], v)
				else:
					data_in_byte_object += struct.pack(''+data.id2types(tx_id)[ctr], v)
				ctr += 1
		else:
			if self.printing:
				print("WARNING: Communication -> __pack_tx_data -> types do not match with data in data object")
		data_in_byte_object = bytes(data_in_byte_object)
		return data_in_byte_object

	def send_message(self, tx_id):

		# store ID and Data in a byte object
		msg = self.__pack_tx_data(tx_id)
		# send message if length of byte object is greater 0
		if len(msg) > 0:
			self.udp_sock.sendto(msg, (self.target_address, self.portSend))
			# nice print out
			if self.printing:
				print(" Data sent -> " + str(msg))
		else:
			if self.printing:
				print('WARNING: Communication -> send_message -> message length is 0')

	def __send_cyclic_thread(self, tx_id, interval_time):

		while self.t2_active_threads[tx_id] and not self.forceShutdown:
			self.send_message(tx_id)
			time.sleep(interval_time)

	def run_send_cyclic_thread(self, tx_id, interval_time):

		if tx_id not in self.t2_active_threads:
			self.t2_active_threads[tx_id] = False

		if self.t2_active_threads[tx_id] is False:
			self.t2_active_threads[tx_id] = True
			self.t2_send_cyclic = threading.Thread(target=self.__send_cyclic_thread, args=[tx_id, interval_time])
			self.t2_send_cyclic.start()
		else:
			self.t2_active_threads[tx_id] = False


## Initialise everything necessary
# Comment: Perhaps this is indeed unnecessary since we first have to initialise the variables globally
# then we call the init method and do this again with the right values
def init():
	##For Jetson board, receiving of Bosch sensor values
	global ipaddressIOcore, ipAddressGroundstation, ipAddressRobotArm, data, udp
	ipaddressIOcore = '192.168.0.40'
	ipAddressGroundstation = '192.168.0.21'
	ipAddressRobotArm = '192.168.0.25'

	#Start receive thread
	data = DataClass()
	udp = UdpClass(ipAddressRobotArm, 2019, 2019, data, printing=True)	#original 2019
	udp.run_receive_thread()

	#Start send thread
	udp.run_send_cyclic_thread(data.ID_ARM, 0.01)
	udp.run_send_cyclic_thread(data.ID_BOSCHEMU,0.01)

if __name__ == '__main__':

	init()
