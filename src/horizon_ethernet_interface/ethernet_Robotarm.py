#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket  # udp services
import time  # time functions like e.g. sleep
import struct  # packing and unpacking byte objects in specified c types
import threading  # multi-threading and mutex
from collections import deque

import rospy

from config import *

rospy.init_node(ROS_NODE_NAME, anonymous=True)
print("im alive")

import udp_tx_rx_datastructure as participant


## ros related defines
base = participant.DataClass()


# create global socket object with UDP services
UDP_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)




class TransferClass:

	def __init__(self, printing=True):
		global UDP_SOCKET
		# set global data object
		self.participant = list()
		# do we want to print ?
		self.printing = printing
		# tf2_rinitialize thread objects
		self.t1_receive = list()
		self.t2_send_cyclic = list()
		self.t2_active_threads = dict()
		# initialize mutex object (acquire: lock, release: unlock)
		self.mutex = threading.Lock()
		#flag to kill the thread
		self.forceShutdown = False

	def __receive_thread(self):
		global UDP_SOCKET
		while not self.forceShutdown:
			try:
				# Setting timeout, else the thread will be stuck at recv from if nothing is received
				UDP_SOCKET.settimeout(3.0)
				# receiving data
				raw_data, address = UDP_SOCKET.recvfrom(2048)  # 128: multiple of 2 & greater than greatest possible data length
				pass
			except socket.error as msg:
				if str(msg) == 'timed out':
					pass
				else:
					print("WARNING: __receive_thread -> " + str(msg))
			else:
				try:
					for user in self.participant:
						if address[0] == user.getAddress()["ip"]:
							self.mutex.acquire()  # enter critical section
							# safe the data in own data class
							user.parsData(raw_data)

							self.mutex.release()  # leave critical section

				except struct.error as msg:
					if self.printing:
						print("Warning: __receive_thread -> " + str(msg))

					

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
		data_in_byte_object = bytes()
		# for loop, which pulls all values out of dictionary and pack them
		# into one big byte object, after that, the data is ready to get send
		if len(self.participant.id2types(tx_id, 'TX')) is len(self.participant.id2keys(tx_id, 'TX')):  # check if types match with data set

			data_in_byte_object = self.data.get_byte_string()
		else:
			if self.printing:
				print("WARNING: Communication -> __pack_tx_data -> types do not match with data in data object")

		data_in_byte_object = bytes(data_in_byte_object)
		return data_in_byte_object

	def send_message(self, user):
		global UDP_SOCKET
		# store ID and Data in a byte object
		msg = user.get_byte_string()
		# send message if length of byte object is greater 0
		if len(msg) > 0:
			UDP_SOCKET.sendto(msg, (user.address["ip"], user.address["port"]))
			# nice print out
			# if self.printing:
			# 	print(" Data sent -> " + str(self.data.id2keys(user.ID, 'TX')))
		else:
			if self.printing:
				print('WARNING: Communication -> send_message -> message length is 0')

	def __send_cyclic_thread(self, user, interval_time):

		while self.t2_active_threads[user] and not self.forceShutdown:
			self.send_message(user)
			time.sleep(interval_time)

	def run_send_cyclic_thread(self, user, interval_time):
		pass
		if user not in self.t2_active_threads:
			self.t2_active_threads[user] = False

		if self.t2_active_threads[user] is False:
			self.t2_active_threads[user] = True
			self.t2_send_cyclic = threading.Thread(target=self.__send_cyclic_thread, args=[user, interval_time])
			self.t2_send_cyclic.start()
		else:
			self.t2_active_threads[user] = False

	def add_participant(self, participant):
		self.participant.append(participant)

	def get_participant(self, participant_type):
		for user in self.participant:
			if  isinstance(user, participant_type):
				return user

def init():
	##For Jetson board, receiving of Bosch sensor values
	global data, ipaddressIOcore, ipAddressGroundstation, ipAddressRobotArm, topicPublisher, topicSubscriber, data, udo, juergens, save_pos_array, save_dir_array, save_complete_array, udp, GROUNDSTATION, IO_CORE, ARM_CORE
	
	base.initRos()

	# bind the socket object to host address and port
	if HOST:
		UDP_SOCKET.bind((HOST["ip"], HOST["port"]))
	else:
		print("no Host or no Port specified")
		return False
	#Start receive thread
	try:
		#Start receive thread
		transfer = TransferClass(printing=True)
		transfer.run_receive_thread()

		transfer.add_participant(participant.ArmCore())
		transfer.add_participant(participant.Groundstation())
		
		transfer.run_send_cyclic_thread(transfer.get_participant(participant.ArmCore), 0.2)
		transfer.run_send_cyclic_thread(transfer.get_participant(participant.Groundstation), 0.5)
		return True
	except:
		return False

def main():
	global data
	
	while True:
		try:
			pass
		except rospy.ROSInterruptException:
			print("some ROS problems")

if __name__ == '__main__':
	# if initialisation was successful go on with main function
	if init():
		main()
	else:
		print("some problems with connection. Check connection!")


# def rosReceivedDataFromIKsolverSystemCallback():
#     sendDataToArmCore()

# def rosReceivedDataFromIKsolverGroundstationCallback():
#     sendDataToGroundstation()
#     pass

# def rosSendDataToIKsolverGroundstation():
#     pass

# def rosSendDataToIKSolverSystem():
#     pass

# def udpReceive():
#     if receivedFromGroundstation():
#         rosSendDataToIKsolverGroundstation()
#     if receivedFromArmeCore():
#         rosSendDataToIKSolverSystem()
