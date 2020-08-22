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
import tf
from collections import deque
import numpy as np

#Ros related import
import rospy

#Message related imports
from horizon_ethernet_interface.msg import BoschSensorMessage
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

##Definitions
#Initialize queue
queue = deque([])
#Initialize save object
udo = (0.0,0.0,0.0)
juergens = (0.0,0.0,0.0)
save_pos_array = (0.0,0.0,0.0)
save_dir_array = (0.0,0.0,0.0)
save_complete_array = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
#Received fused position
rcv_fused = False
#names and adresses
ipaddressIOcore = ''
ipAddressGroundstation = ''
topicPublisher = ''
topicSubscriber = ''
nameNode = ''


class DataClass:

	def __init__(self):
		self.ID_BOSCHEMU = 11
		self.BoschEMU_EMU_GS_types = [	'B',  # dataID
										'f',  # xEuler
										'f',  # yEuler
										'f',  # zEuler
										'f',  # xLinearAcc
										'f',  # yLinearAcc
										'f']  # zLinearAcc
		self.BoschEMU_EMU_GS_list = [	self.ID_BOSCHEMU,  # dataID
										0.0,  # xEuler
										0.0,  # yEuler
										0.0,  # zEuler
										0.0,  # xLinearAcc
										0.0,  # yLinearAcc
										0.0]  # zLinearAcc
		self.BoschEMU_EMU_GS_keys = {	"dataID": 0,
										"xEuler": 1,
										"yEuler": 2,
										"zEuler": 3,
										"xLinearAcc": 4,
										"yLinearAcc": 5,
										"zLinearAcc": 6}
		self.BoschEMU_EMU_GS = [	self.BoschEMU_EMU_GS_list,
									self.BoschEMU_EMU_GS_types,
									self.BoschEMU_EMU_GS_keys]

		self.ID_JETSONFUSED = 12
		self.Jetson_FusedPosition_GS_types = [	'B',  # dataID
												'B',  # dummy0
												'B',  # dummy1
												'B',  # dummy2
												'f',  # xPos
												'f',  # yPos
												'f',  # zPos
												'f',  # xQuaternion
												'f',  # yQuaternion
												'f',  # zQuaternion
												'f']  # wQuaternion
		self.Jetson_FusedPosition_GS_list = [	self.ID_JETSONFUSED,  # dataID
												0,    # dummy0
												0,    # dummy1
												0,    # dummy2
												0.0,  # xPos
												0.0,  # yPos
												0.0,  # zPos
												0.0,  # xQuaternion
												0.0,  # yQuaternion
												0.0,  # zQuaternion
												0.0]  # wQuaternion
		self.Jetson_FusedPosition_GS_keys = {	"dataID": 0,
												"dummy0": 1,
												"dummy1": 2,
												"dummy2": 3,
												"xPosition": 4,
												"yPosition": 5,
												"zPosition": 6,
												"xQuaternion": 7,
												"yQuaternion": 8,
												"zQuaternion": 9,
												"wQuaternion": 10}
		self.Jetson_FusedPosition_GS = [	self.Jetson_FusedPosition_GS_list,
											self.Jetson_FusedPosition_GS_types,
											self.Jetson_FusedPosition_GS_keys]

	def id2types(self, id_):
		# get types of a package by passing corresponding ID
		switcher = {	self.ID_BOSCHEMU: self.BoschEMU_EMU_GS_types,
						self.ID_JETSONFUSED: self.Jetson_FusedPosition_GS_types}
		return switcher.get(id_, "WARNING: id2types -> invalid ID")

	def id2keys(self, id_):
		# get data of a package by passing corresponding ID
		switcher = {	self.ID_BOSCHEMU: self.BoschEMU_EMU_GS_keys,
						self.ID_JETSONFUSED: self.Jetson_FusedPosition_GS_keys}
		return switcher.get(id_, "WARNING: id2keys -> invalid ID")

	def id2lists(self, id_):
		#get the array of the lists of the correspending type
		switcher = {	self.ID_BOSCHEMU: self.BoschEMU_EMU_GS,
						self.ID_JETSONFUSED: self.Jetson_FusedPosition_GS}
		return switcher.get(id_, "WARNING: id2lists -> invalid ID")


class UdpClass:

	def __init__(self, target_address, portSend, portRecv, data_object, printing=True):
		# set host address and port
		#self.host = socket.gethostbyname(socket.gethostname())
		self.host = '192.168.0.50'
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
					recv_data = struct.unpack(''.join(self.data.BoschEMU_EMU_GS_types), raw_data)
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
				print(" Data sent -> " + str(self.data.id2keys(tx_id)))
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


##Callback method for fused position messages
def callbackFusedPosition(data):
	#gets it as geometry_msgs/PoseWithCovariance -> Header, pose.Pose (3D position and rotation as Quaternion) => has to be calculated at ground station
	global save_pos_array
	global save_dir_array
	global save_complete_array
	#get the camera frame to base_link transform and applies it to calculate the rover position out of the zed camera center position
	#transformed_pose = tf.transformPose('base_link',data)
	transformed_pose = data
	save_pos_array = (transformed_pose.pose.pose.position.x,transformed_pose.pose.pose.position.y,transformed_pose.pose.pose.position.z)
	save_dir_array = (transformed_pose.pose.pose.orientation.x,transformed_pose.pose.pose.orientation.y,transformed_pose.pose.pose.orientation.z,transformed_pose.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(save_dir_array)
	save_complete_array = (12, 0, 0, 0, save_pos_array[0], save_pos_array[1], save_pos_array[2], euler[2], 0.0, 0.0, 0.0)
	global rcv_fused
	rcv_fused = True

##Calculate the quaternions from the given euler angles
def eulerToQuaternion(data):
	res = Quaternion()
	y = data[0]
	p = data[1]
	r = data[2]
	
	sy = np.sin(y/2)
	cy = np.cos(y/2)
	#sp = np.sin(p/2)
	#cp = np.cos(p/2)
	#sr = np.sin(r/2)
	#cr = np.cos(r/2)
	#res.x = sr * cp * cy - cr * sp * sy
	#res.y = cr * sp * cy + sr * cp * sy
	#res.z = cr * cp * sy - sr * sp * cy
	#res.w = cr * cp * cy + sr * sp * sy
	
	##Onlx rotation around the z axis is considered, other stuff didn't work out
	res.x = 0
	res.y = 0
	res.z = cy * 0.5
	res.w = sy * 0.5
	
	return res

## Initialise everything necessary
# Comment: Perhaps this is indeed unnecessary since we first have to initialise the variables globally
# then we call the init method and do this again with the right values
def init():
	##For Jetson board, receiving of Bosch sensor values
	global ipaddressIOcore, ipAddressGroundstation, topicPublisher, topicSubscriber, nameNode, data, udo, juergens, save_pos_array, save_dir_array, save_complete_array, udp
	ipaddressIOcore = '192.168.0.40'
	ipAddressGroundstation = '192.168.0.21'
	topicPublisher = '/horizon/bosch_imu'
	#topicSubscriber = '/horizon/position_fused'
	topicSubscriber = '/zed/zed_node/pose_with_covariance'
	nameNode = 'HorizonSensorPublisher'
	
	#TF Transformer
	t = tf.TransformerROS(True, rospy.Duration(10.0))

	#Start receive thread
	data = DataClass()
	udp = UdpClass(ipAddressGroundstation, 1024, 2019, data, printing=False)
	udp.run_receive_thread()

	udo = np.zeros(3, dtype=np.float32)
	juergens = np.zeros(3, dtype=np.float32)
	save_pos_array = np.zeros(3, dtype=np.float32)
	save_dir_array = np.zeros(4, dtype=np.float32)
	save_complete_array = (0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

	#Start send thread
	udp.run_send_cyclic_thread(data.ID_JETSONFUSED, 0.2)

if __name__ == '__main__':

	init()

	try:
		print("trying")
		rospy.init_node(nameNode, anonymous=True)
		pubSensor = rospy.Publisher(topicPublisher, Imu, queue_size = 10)
		subFusedPos = rospy.Subscriber(topicSubscriber, PoseWithCovarianceStamped, callbackFusedPosition)
		rate = rospy.Rate(100)

		#endless for loop for resolving the queued messages
		while not rospy.is_shutdown() and not udp.forceShutdown:
			while len(queue)>0:
				i = queue.popleft()
				if udp.printing:
					print("message received, working on it")
					print(i)
				udo = np.zeros(3, dtype=np.float32)
				juergens = np.zeros(3, dtype=np.float32)
				udo = i[1:4]
				juergens = i[4:7]

				#Sensor msgs/Imu
				msg = Imu()

				msg.header.stamp = rospy.Time.now()
				msg.header.frame_id = 'imu'

				msg.orientation = eulerToQuaternion(udo)
				msg.orientation_covariance = [	1e-9, 0.0, 0.0, 
												0.0, 1e-9, 0.0, 
												0.0, 0.0, 1e-9]

				msg.angular_velocity = Vector3(0.0,0.0,0.0)
				msg.angular_velocity_covariance = [	0.0, 0.0, 0.0,
													0.0, 0.0, 0.0,
													0.0, 0.0, 0.0]

				msg.linear_acceleration = Vector3()
				msg.linear_acceleration.x = juergens[0]
				msg.linear_acceleration.y = juergens[1]
				msg.linear_acceleration.z = juergens[2]
				msg.linear_acceleration_covariance = [	1e-9, 0.0, 0.0,
														0.0, 1e-9, 0.0,
														0.0, 0.0, 1e-9]

				#BoschSensormsg
				#msg = BoschSensorMessage()
				#msg.pos = udo
				#msg.dir = juergens
				#Send message
				pubSensor.publish(msg)
			#push received position message into datastructure
			if rcv_fused:
				ctr = 0
				udp.mutex.acquire()
				for key in data.id2keys(12):
					data.id2lists(12)[0][data.id2keys(12)[key]] = save_complete_array[data.id2keys(12)[key]]
					ctr += 1
				udp.mutex.release()
				rcv_fused = False
			rate.sleep()

		udp.forceShutdown = True
		udp.t1_receive.join()
	except rospy.ROSInterruptException:
		pass
	except:
		print('Unexpected error')
		raise
