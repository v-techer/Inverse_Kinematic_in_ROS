

from config import *
import rospy
from collections import deque
import struct  # packing and unpacking byte objects in specified c types


#Message related imports
from horizon_ethernet_interface.msg import GroundstationToRobotarm
from horizon_ethernet_interface.msg import ArmCoreToRobotarm


rospy.init_node(ROS_NODE_NAME, anonymous=True)

class DataClass:
		
	def __init__(self):
		# IDs:
		self.ID_ERROR = ID_ERROR
		self.ID_BMS = ID_BMS
		self.ID_WEIGHTCELL = ID_WEIGHTCELL
		self.ID_SECURITY = ID_SECURITY
		self.ID_SAFETY = ID_SAFETY
		self.ID_ENVIRONMENTSENSOR = ID_ENVIRONMENTSENSOR
		self.ID_DRILL = ID_DRILL
		self.ID_ARM = ID_ARM
		self.ID_POWERTRAIN = ID_POWERTRAIN
		self.ID_SYSTEMSTATE = ID_SYSTEMSTATE
		self.ID_BOSCHEMU = ID_BOSCHEMU
		self.ID_JETSONFUSED = ID_JETSONFUSED
		self.ID_ARM_CORE = ID_ARM_CORE

	def getAddress(self, raw_data):
		pass

	def parsData(self, raw_data):
		pass

	def id2types(self, id_, direction_):
		 pass

	def id2keys(self, id_):
		 pass

	def id2lists(self, id_):
		 pass
	
	def initRos(self):
		pass

class Groundstation(DataClass):
	def __init__(self):
		self.address = GROUNDSTATION
		DataClass.__init__(self)

		self.groundstation_to_ik_solver = rospy.Publisher('groundstation_to_ik_solver', GroundstationToRobotarm, queue_size=1)

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

		self.RobotArm_GS_ARM_types = [	'B',
										'B',
										'B',
										'B',
										'h',
										'h',
										'h',
										'h',
										'h',
										'h',
										'h',
										'B',
										'B',
										'B',
										'B',
										'B',
										'B']
		self.RobotArm_GS_ARM_list = [	self.ID_ARM,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,]
		self.RobotArm_GS_ARM_keys = { 	"dataID": 0,
										"mode" : 1,
										"teachedPos" : 2,
										"activAxis": 3,
										"axisVelocity" : 4,
										"target_x" : 5,
										"target_y" : 6,
										"target_z": 7,
										"target_roll": 8,
										"target_pitch": 9,
										"target_yaw": 10,
										"endEffectorState" : 11,
										"movementStarted": 12,	
										"collision": 13,
										"dummy0": 14,
										"dummy1": 15,
										"dummy2":16}
		self.RobotArm_GS_ARM = [	self.RobotArm_GS_ARM_list,
									self.RobotArm_GS_ARM_types,
									self.RobotArm_GS_ARM_keys]
		self.RobotArm_ARM_GS_types = [	'B',
										'B',
										'B',
										'B',
										'h',
										'h',
										'h',
										'h',
										'h',
										'h',
										'h',
										'h',
										'h',
										'h',
										'h',
										'h']
		self.RobotArm_ARM_GS_list = [	self.ID_ARM,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0,
										0]
		self.RobotArm_ARM_GS_keys = { 	'dataID' : 0,
										'status' : 1,
										'mode' : 2,
										'gripperStatate' : 3,
										'target_x' : 4,
										'target_y' : 5,
										'target_z' : 6,
										'target_roll' : 7,
										'target_pitch' : 8,
										'target_yaw' : 9,
										'actualJointAngle1' : 10,
										'actualJointAngle2' : 11,
										'actualJointAngle3' : 12,
										'actualJointAngle4' : 13,
										'actualJointAngle5' : 14,
										'actualJointAngle6' : 15}
		self.RobotArm_ARM_GS = [	self.RobotArm_ARM_GS_list,
									self.RobotArm_ARM_GS_types,
									self.RobotArm_ARM_GS_keys]
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
	def getAddress(self):
		return self.address

	def parsData(self, raw_data):
	
		# store id
		rx_id = struct.unpack('B', raw_data[0:1])[0]

		recv_data = struct.unpack(''.join(self.id2types(rx_id, 'RX')), raw_data)

		self.setValuesInDataList(recv_data, rx_id, 'RX')

		self.publishData(rx_id)

	def publishData(self, rx_id):
		if rx_id == self.ID_ARM:
			# create message
			msg = GroundstationToRobotarm()
			temp_dict = self.id2lists(rx_id, 'RX')
			msg.dataID = temp_dict[self.getIndexByKey(rx_id, 'RX', 'dataID')]
			msg.mode = temp_dict[self.getIndexByKey(rx_id, 'RX', 'mode')]
			msg.teached_pos = temp_dict[self.getIndexByKey(rx_id, 'RX', 'teachedPos')]
			msg.active_axis = temp_dict[self.getIndexByKey(rx_id, 'RX', 'activAxis')]
			msg.axis_velocity = temp_dict[self.getIndexByKey(rx_id, 'RX', 'axisVelocity')]
			msg.target_x = temp_dict[self.getIndexByKey(rx_id, 'RX', 'target_x')]
			msg.target_y = temp_dict[self.getIndexByKey(rx_id, 'RX', 'target_y')]
			msg.target_z = temp_dict[self.getIndexByKey(rx_id, 'RX', 'target_z')]
			msg.target_roll = temp_dict[self.getIndexByKey(rx_id, 'RX', 'target_roll')]
			msg.target_pitch = temp_dict[self.getIndexByKey(rx_id, 'RX', 'target_pitch')]
			msg.target_yaw = temp_dict[self.getIndexByKey(rx_id, 'RX', 'target_yaw')]
			msg.gripper_status = temp_dict[self.getIndexByKey(rx_id, 'RX', 'endEffectorState')]
			msg.movementStarted = temp_dict[self.getIndexByKey(rx_id, 'RX', 'movementStarted')]
			msg.collision = temp_dict[self.getIndexByKey(rx_id, 'RX', 'collision')]
			msg.dummy0 = temp_dict[self.getIndexByKey(rx_id, 'RX', 'dummy0')]
			msg.dummy1 = temp_dict[self.getIndexByKey(rx_id, 'RX', 'dummy1')]
			msg.dummy2 = temp_dict[self.getIndexByKey(rx_id, 'RX', 'dummy2')]
			rospy.loginfo(msg)
			
			# publish message
			self.groundstation_to_ik_solver.publish(msg)

	def setValuesInDataList(self, recv_data, id_, direction_):
		if direction_ is 'RX':
			if id_ is ID_ARM:
				self.RobotArm_GS_ARM_list = recv_data

	def getIndexByKey(self, id_, direction_, key_):
		if direction_ is 'RX':
			if id_ is ID_ARM:
				return self.RobotArm_GS_ARM_keys[key_]
				

	def id2types(self, id_, direction_):

		if direction_ is 'RX':
			# get types of a package by passing corresponding ID
			switcher = {	self.ID_ARM: self.RobotArm_GS_ARM_types
							}
		elif direction_ is 'TX':
			# get types of a package by passing corresponding ID
			switcher = {	self.ID_BOSCHEMU: self.BoschEMU_EMU_GS_types,
							self.ID_JETSONFUSED: self.Jetson_FusedPosition_GS_types,
							self.ID_ARM: self.RobotArm_ARM_GS_types
							}

		return switcher.get(id_, "WARNING: id2types -> invalid ID")

	def id2keys(self, id_, direction_):

		if direction_ is 'RX':
			# get data of a package by passing corresponding ID
			switcher = {	self.ID_ARM: self.RobotArm_GS_ARM_keys
							}
		elif direction_ is 'TX':
			# get types of a package by passing corresponding ID
			switcher = {	self.ID_BOSCHEMU: self.BoschEMU_EMU_GS_types,
							self.ID_JETSONFUSED: self.Jetson_FusedPosition_GS_types,
							self.ID_ARM: self.RobotArm_ARM_GS_types
							}
		return switcher.get(id_, "WARNING: id2keys -> invalid ID")

	def id2lists(self, id_, direction_):
		if direction_ is 'RX':
			#get the array of the lists of the correspending type
			switcher = {	self.ID_ARM: self.RobotArm_GS_ARM_list
							}
		elif direction_ is 'TX':
			#get the array of the lists of the correspending type
			switcher = {	self.ID_BOSCHEMU: self.BoschEMU_EMU_GS_list,
							self.ID_JETSONFUSED: self.Jetson_FusedPosition_GS_list,
							self.ID_ARM: self.RobotArm_ARM_GS_list
							}
							
		return switcher.get(id_, "WARNING: id2keys -> invalid ID")

class ArmCore(DataClass):
	def __init__(self):
		self.address = ARM_CORE
		DataClass.__init__(self)

		self.armCore_to_ik_solver = rospy.Publisher('armCore_to_ik_solver', ArmCoreToRobotarm, queue_size=1)
		self.RobotArm_ARM_TO_ARM_CORE_types = [	'B',
												'B',
												'B',
												'B',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i']
		self.RobotArm_ARM_TO_ARM_CORE_list = [	self.ID_ARM,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0]
		self.RobotArm_ARM_TO_ARM_CORE_keys = { 	"dataID": 0,
												"operationEnable": 1,
												"mode" : 2,
												"Endeffector_open" : 3,
												"JointAngle1" : 4,
												"JointAngle2": 5,
												"JointAngle3" : 6,
												"JointAngle4" : 7,
												"JointAngle5" : 8,
												"JointAngle6": 9,
												"targetJointVelocity1": 10,
												"targetJointVelocity2": 11,
												"targetJointVelocity3": 12,
												"targetJointVelocity4" : 13,
												"targetJointVelocity5" : 14,
												"targetJointVelocity6" : 15,
												"targetJointAcceleration1" : 16,
												"targetJointAcceleration2" : 17,
												"targetJointAcceleration3" : 18,
												"targetJointAcceleration4" : 19,
												"targetJointAcceleration5" : 20,
												"targetJointAcceleration6" : 21}
		self.RobotArm_ARM_TO_ARM_CORE = [	self.RobotArm_ARM_TO_ARM_CORE_list,
											self.RobotArm_ARM_TO_ARM_CORE_types,
											self.RobotArm_ARM_TO_ARM_CORE_keys]
		self.RobotArm_ARM_CORE_TO_ARM_types = [	'B',
												'B',
												'B',
												'B',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'i',
												'B',
												'B',
												'B',
												'B',
												'B',
												'B',
												'B',
												'B']
		self.RobotArm_ARM_CORE_TO_ARM_list = [	self.ID_ARM_CORE,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0,
												0]
		self.RobotArm_ARM_CORE_TO_ARM_keys = { 	"dataID": 0,
												"operationEnabled": 1,
												"actualMode" : 2,
												"Endeffector_IsOpen" : 3,
												"acutalJointAngle1" : 4,
												"acutalJointAngle2" : 5,
												"acutalJointAngle3" : 6,
												"acutalJointAngle4" : 7,
												"acutalJointAngle5" : 8,
												"acutalJointAngle6" : 9,
												"actualJointVelocity1": 10,
												"actualJointVelocity2" : 11,
												"actualJointVelocity3" : 12,
												"actualJointVelocity4" : 13,
												"actualJointVelocity5" : 14,
												"actualJointVelocity6" : 15,
												"HomeOffset1" : 16,
												"HomeOffset2" : 17,
												"HomeOffset3" : 18,
												"HomeOffset4" : 19,
												"HomeOffset5" : 20,
												"HomeOffset6" : 21,
												"targetJointAngle1" : 22,
												"targetJointAngle2" : 23,
												"targetJointAngle3" : 24,
												"targetJointAngle4" : 25,
												"targetJointAngle5" : 26,
												"targetJointAngle6" : 27,
												"targetJointVelocity1" : 28,
												"targetJointVelocity2" : 29,
												"targetJointVelocity3" : 30,
												"targetJointVelocity4" : 31,
												"targetJointVelocity5" : 32,
												"targetJointVelocity6" : 33,
												"targetJointAcceleration1" : 34,
												"targetJointAcceleration2" : 35,
												"targetJointAcceleration3" : 36,
												"targetJointAcceleration4" : 37,
												"targetJointAcceleration5" : 38,
												"targetJointAcceleration6" : 39,
												"reachedJointPosition1" : 40,
												"reachedJointPosition2" : 41,
												"reachedJointPosition3" : 42,
												"reachedJointPosition4" : 43,
												"reachedJointPosition5" : 44,
												"reachedJointPosition6" : 45,
												"dummy1":46,
												"dummy2":47}
		self.RobotArm_ARM_CORE_TO_ARM = [	self.RobotArm_ARM_CORE_TO_ARM_list,
											self.RobotArm_ARM_CORE_TO_ARM_types,
											self.RobotArm_ARM_CORE_TO_ARM_keys]

	def rosSend(self):
		pass

	def parsData(self, raw_data):
	
		# store id
		rx_id = struct.unpack('B', raw_data[0:1])[0]

		recv_data = struct.unpack(''.join(self.id2types(rx_id, 'RX')), raw_data)

		self.setValuesInDataList(recv_data, rx_id, 'RX')

		self.publishData(rx_id)

	def publishData(self, rx_id):
		if rx_id == self.ID_ARM:
			# create message
			msg = ArmCoreToRobotarm()
			temp_dict = self.id2lists(rx_id, 'RX')

			msg.dataID = self.getIndexByKey(rx_id, 'RX', 'dataID')
			msg.operationEnabled = self.getIndexByKey(rx_id, 'RX', 'operationEnabled')
			msg.actualMode = self.getIndexByKey(rx_id, 'RX', 'actualMode')
			msg.Endeffector_IsOpen = self.getIndexByKey(rx_id, 'RX', 'Endeffector_IsOpen')
			msg.actualPositions = [	self.getIndexByKey(rx_id, 'RX', 'acutalJointAngle1'), self.getIndexByKey(rx_id, 'RX', 'acutalJointAngle2'),
									self.getIndexByKey(rx_id, 'RX', 'acutalJointAngle3'), self.getIndexByKey(rx_id, 'RX', 'acutalJointAngle4'),
									self.getIndexByKey(rx_id, 'RX', 'acutalJointAngle5'), self.getIndexByKey(rx_id, 'RX', 'acutalJointAngle6'),]
			msg.actualVelocities = [ self.getIndexByKey(rx_id, 'RX', 'actualJointVelocity1'), self.getIndexByKey(rx_id, 'RX', 'actualJointVelocity2'),
									 self.getIndexByKey(rx_id, 'RX', 'actualJointVelocity3'), self.getIndexByKey(rx_id, 'RX', 'actualJointVelocity4'),
									 self.getIndexByKey(rx_id, 'RX', 'actualJointVelocity5'), self.getIndexByKey(rx_id, 'RX', 'actualJointVelocity6'),]
			msg.HomeOffset = [	self.getIndexByKey(rx_id, 'RX', 'HomeOffset1'), self.getIndexByKey(rx_id, 'RX', 'HomeOffset2'),
								self.getIndexByKey(rx_id, 'RX', 'HomeOffset3'), self.getIndexByKey(rx_id, 'RX', 'HomeOffset4'),
								self.getIndexByKey(rx_id, 'RX', 'HomeOffset5'), self.getIndexByKey(rx_id, 'RX', 'HomeOffset6'),]
			msg.targetPositions = [ self.getIndexByKey(rx_id, 'RX', 'targetJointAngle1'), self.getIndexByKey(rx_id, 'RX', 'targetJointAngle2'),
									self.getIndexByKey(rx_id, 'RX', 'targetJointAngle3'), self.getIndexByKey(rx_id, 'RX', 'targetJointAngle4'),
									self.getIndexByKey(rx_id, 'RX', 'targetJointAngle5'), self.getIndexByKey(rx_id, 'RX', 'targetJointAngle6'),]
			msg.targetVelocities = [ self.getIndexByKey(rx_id, 'RX', 'targetJointVelocity1'), self.getIndexByKey(rx_id, 'RX', 'targetJointVelocity2'),
									 self.getIndexByKey(rx_id, 'RX', 'targetJointVelocity3'), self.getIndexByKey(rx_id, 'RX', 'targetJointVelocity4'),
									 self.getIndexByKey(rx_id, 'RX', 'targetJointVelocity5'), self.getIndexByKey(rx_id, 'RX', 'targetJointVelocity6'),]
			msg.targetAcceleration  = [ self.getIndexByKey(rx_id, 'RX', 'targetJointAcceleration1'), self.getIndexByKey(rx_id, 'RX', 'targetJointAcceleration2'),
									 	self.getIndexByKey(rx_id, 'RX', 'targetJointAcceleration3'), self.getIndexByKey(rx_id, 'RX', 'targetJointAcceleration4'),
									 	self.getIndexByKey(rx_id, 'RX', 'targetJointAcceleration5'), self.getIndexByKey(rx_id, 'RX', 'targetJointAcceleration6'),]
			msg.PositionReached = [ self.getIndexByKey(rx_id, 'RX', 'reachedJointPosition1'), self.getIndexByKey(rx_id, 'RX', 'reachedJointPosition2'),
									self.getIndexByKey(rx_id, 'RX', 'reachedJointPosition3'), self.getIndexByKey(rx_id, 'RX', 'reachedJointPosition4'),
									self.getIndexByKey(rx_id, 'RX', 'reachedJointPosition5'), self.getIndexByKey(rx_id, 'RX', 'reachedJointPosition6'),]
			msg.dummy = [0, 0]
			rospy.loginfo(msg)
			
			# publish message
			self.armCore_to_ik_solver.publish(msg)

	def setValuesInDataList(self, recv_data, id_, direction_):
		if direction_ is 'RX':
			if id_ is ID_ARM_CORE:
				self.RobotArm_ARM_CORE_TO_ARM_list = recv_data

	def getIndexByKey(self, id_, direction_, key_):
		if direction_ is 'RX':
			if id_ is ID_ARM_CORE:
				return self.RobotArm_ARM_CORE_TO_ARM_keys[key_]

	def id2types(self, id_, direction_):

		if direction_ is 'RX':
			# get types of a package by passing corresponding ID
			switcher = {	self.ID_ARM_CORE: self.RobotArm_ARM_CORE_TO_ARM_types
							}
		elif direction_ is 'TX':
			# get types of a package by passing corresponding ID
			switcher = {	self.ID_ARM_CORE: self.RobotArm_ARM_TO_ARM_CORE_types
							}
		return switcher.get(id_, "WARNING: id2types -> invalid ID")
		
	def id2keys(self, id_, direction_):
		if direction_ is 'RX':
		# get data of a package by passing corresponding ID
			switcher = {	self.ID_ARM_CORE: self.RobotArm_ARM_CORE_TO_ARM_keys
							}
		elif direction_ is 'TX':
			# get data of a package by passing corresponding ID
			switcher = {	self.ID_ARM_CORE: self.RobotArm_ARM_TO_ARM_CORE_keys
							}
		return switcher.get(id_, "WARNING: id2keys -> invalid ID")

	def id2lists(self, id_):
		#get the array of the lists of the correspending type
		switcher = {	self.ID_ARM_CORE: self.RobotArm_ARM_TO_ARM_CORE,
						self.ID_ARM_CORE: self.RobotArm_ARM_CORE_TO_ARM
						}
		return switcher.get(id_, "WARNING: id2lists -> invalid ID")


class IOcore(DataClass):

	def __init__(self):
		self.arddress = IO_CORE
		self.queue = deque([])
		DataClass.__init__(self)

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
	
	def parsData(self, raw_data):
		pass
	# 	# store id
	# 	rx_id = struct.unpack('B', raw_data[0:1])[0]

	# 	recv_data = struct.unpack(''.join(self.data.id2types(rx_id, 'RX'))

	# 	# in some cases you want to store all received data. So store the data in a queue
	# 	queueData(receive_data)

	# 	for key, index in enumerate(id2keys(rx_id, 'RX')):

	# 		id2keys(rx_id, 'RX')[key] = recv_data[index]

	# 		print("{}: {}".format(key, id2keys(rx_id, 'RX')[key]))


	# def queueData(self, receive_data):
	# 	self.queue.append(recv_data)

	def getAddress(self):
		return self.address

	def id2types(self, id_, direction_):

		if direction_ is 'RX':
			pass
		elif direction_ is 'TX':
			# get types of a package by passing corresponding ID
			switcher = {	self.ID_BOSCHEMU: self.BoschEMU_EMU_GS_types
							}
		return switcher.get(id_, "WARNING: id2types -> invalid ID")

	def id2keys(self, id_):
		# get data of a package by passing corresponding ID
		switcher = {	self.ID_BOSCHEMU: self.BoschEMU_EMU_GS_keys
						}
		return switcher.get(id_, "WARNING: id2keys -> invalid ID")

	def id2lists(self, id_):
		#get the array of the lists of the correspending type
		switcher = {	self.ID_BOSCHEMU: self.BoschEMU_EMU_GS
						}
		return switcher.get(id_, "WARNING: id2lists -> invalid ID")


if __name__ == '__main__':
	pass