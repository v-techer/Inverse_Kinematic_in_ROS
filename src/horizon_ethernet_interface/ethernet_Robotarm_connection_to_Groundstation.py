
import socket  # udp services
import time  # time functions like e.g. sleep
import threading  # multi-threading and mutex
import struct  # packing and unpacking byte objects in specified c types
from collections import deque

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16

from config import *
import udp_tx_rx_datastructure as data


## ros related defines
rosNodeName = 'connection_pipeline'

##Definitions
#Initialize queue
queue = deque([])

# create global socket object with UDP services
UDP_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)




class UdpClass:

	def __init__(self, target_address, data_object, printing=True):
		global UDP_SOCKET
		# set host address and port
		self.target_address = target_address
		# set global data object
		self.data = data_object  # TODO
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
				raw_data = UDP_SOCKET.recvfrom(2048)[0]  # 128: multiple of 2 & greater than greatest possible data length
				pass
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
					recv_data = struct.unpack(''.join(self.data.id2types(rx_id, 'RX')), raw_data)
					#push received data set into queue
					queue.append(recv_data)
				except struct.error as msg:
					if self.printing:
						print("Warning: __receive_thread -> " + str(msg))
				else:
					self.mutex.acquire()  # enter critical section
					for key in self.data.id2keys(rx_id):
						self.data.id2keys(rx_id)[key] = recv_data[ctr]

						print("{}: {}".format(key, self.data.id2keys(rx_id)[key]))
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
		global UDP_SOCKET
		# store ID and Data in a byte object
		msg = self.__pack_tx_data(tx_id)
		# send message if length of byte object is greater 0
		if len(msg) > 0:
			UDP_SOCKET.sendto(msg, (self.target_address["ip"], self.target_address["port"]))
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

def init():
	##For Jetson board, receiving of Bosch sensor values
	global data, ipaddressIOcore, ipAddressGroundstation, ipAddressRobotArm, topicPublisher, topicSubscriber, data, udo, juergens, save_pos_array, save_dir_array, save_complete_array, udp, GROUNDSTATION, IO_CORE, ARM_CORE
	
	# bind the socket object to host address and port
	if HOST:
		UDP_SOCKET.bind((HOST["ip"], HOST["port"]))
	else:
		print("no Host or no Port specified")
		return False
	#Start receive thread
	try:
		#Start receive thread
		udpGroundstation = UdpClass(GROUNDSTATION, data.Groundstation(), printing=True)
		udpArmCore = UdpClass(ARM_CORE, data.ArmCore(), printing=True)
		#udpArmIOcore = UdpClass(IO_CORE, data.IOcore(), printing=True)
		udpGroundstation.run_receive_thread()
		udpArmCore.run_receive_thread()
		#udpArmIOcore.run_receive_thread()
		return True
	except:
		return False

def talker():
    pub = rospy.Publisher('groundstation_data', String, queue_size=10)
    rospy.init_node(rosNodeName, anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


def main():
	global data
	
	while True:
		try:
			talker()
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
