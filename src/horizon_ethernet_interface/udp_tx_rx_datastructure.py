

from config import *
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from collections import deque
import struct  # packing and unpacking byte objects in specified c types


#Message related imports
from horizon_ethernet_interface.msg import GroundstationToRobotarm
from horizon_ethernet_interface.msg import ArmCoreToRobotarm
from horizon_ethernet_interface.msg import RobotarmToArmCore
from horizon_ethernet_interface.msg import RobotarmToGroundstation


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

	def getAddress(self):
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

	def getIndexByKey(self, id_, direction_, key_):
		pass
	
	def getValueByIndex(self, index_):
		pass

class Groundstation(DataClass):
	def __init__(self):
		self.address = GROUNDSTATION
		DataClass.__init__(self)

		self.groundstation_to_ik_solver = rospy.Publisher('groundstation_to_ik_solver', GroundstationToRobotarm, queue_size=1)
		rospy.Subscriber('ik_solver_to_groundstation', RobotarmToGroundstation, self.callbackRobotState)

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
										"collisionDetection": 13,
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

	def callbackRobotState(self, msg):
		self.RobotArm_ARM_GS_list[1] = msg.status
		self.RobotArm_ARM_GS_list[2] = msg.mode
		self.RobotArm_ARM_GS_list[3] = msg.gripperStatate
		self.RobotArm_ARM_GS_list[4] = msg.targetCoordinate_target_x
		self.RobotArm_ARM_GS_list[5] = msg.targetCoordinate_target_y
		self.RobotArm_ARM_GS_list[6] = msg.targetCoordinate_target_z
		self.RobotArm_ARM_GS_list[7] = msg.targetCoordinate_target_roll
		self.RobotArm_ARM_GS_list[8] = msg.targetCoordinate_target_pitch
		self.RobotArm_ARM_GS_list[9] = msg.targetCoordinate_target_yaw
		self.RobotArm_ARM_GS_list[10] = msg.actualJointAngle1
		self.RobotArm_ARM_GS_list[11] = msg.actualJointAngle2
		self.RobotArm_ARM_GS_list[12] = msg.actualJointAngle3
		self.RobotArm_ARM_GS_list[13] = msg.actualJointAngle4
		self.RobotArm_ARM_GS_list[14] = msg.actualJointAngle5
		self.RobotArm_ARM_GS_list[15] = msg.actualJointAngle6

		rospy.loginfo(msg)

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
			msg.collisionDetection = temp_dict[self.getIndexByKey(rx_id, 'RX', 'collisionDetection')]
			msg.dummy0 = temp_dict[self.getIndexByKey(rx_id, 'RX', 'dummy0')]
			msg.dummy1 = temp_dict[self.getIndexByKey(rx_id, 'RX', 'dummy1')]
			msg.dummy2 = temp_dict[self.getIndexByKey(rx_id, 'RX', 'dummy2')]

			#rospy.loginfo(msg)
			
			# publish message
			self.groundstation_to_ik_solver.publish(msg)

	def get_byte_string(self):
	
		data_in_byte_object = bytes()

		for i, key in enumerate(self.RobotArm_ARM_GS_keys.values()):
			
			value = self.getValueByIndex(i, 'TX')

			data_in_byte_object += struct.pack(''+self.id2types(self.ID_ARM, 'TX')[i], value)

		return data_in_byte_object
	
	def getValueByIndex(self, index_, direction_):
		if direction_ is 'TX':
			return self.RobotArm_ARM_GS_list[index_]

	def setValuesInDataList(self, recv_data, id_, direction_):
		if direction_ is 'RX':
			if id_ is ID_ARM:
				self.RobotArm_GS_ARM_list = recv_data
		if direction_ is 'TX':
			if id_ is ID_ARM:
				self.RobotArm_ARM_GS_list = recv_data

	def getIndexByKey(self, id_, direction_, key_):
		if direction_ is 'RX':
			if id_ is ID_ARM:
				return self.RobotArm_GS_ARM_keys[key_]
		if direction_ is 'TX':
			if id_ is ID_ARM:
				return self.RobotArm_ARM_GS_keys[key_]
				

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

		self.arm_core_to_ik_solver = rospy.Publisher('arm_core_to_ik_solver', ArmCoreToRobotarm, queue_size=1)
		rospy.Subscriber('ik_solver_to_arm_core', RobotarmToArmCore, self.callbackSystemState)
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
		self.RobotArm_ARM_TO_ARM_CORE_list = [	self.ID_ARM_CORE,
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
												"robotArmPositionReached" :46,
												"dummy1":47}
		self.RobotArm_ARM_CORE_TO_ARM = [	self.RobotArm_ARM_CORE_TO_ARM_list,
											self.RobotArm_ARM_CORE_TO_ARM_types,
											self.RobotArm_ARM_CORE_TO_ARM_keys]

	def callbackSystemState(self, msg):
		self.RobotArm_ARM_TO_ARM_CORE_list[1] = msg.operationEnable
		self.RobotArm_ARM_TO_ARM_CORE_list[2] = msg.targetlArmCoreMode
		self.RobotArm_ARM_TO_ARM_CORE_list[3] = msg.Endeffector_open
		self.RobotArm_ARM_TO_ARM_CORE_list[4] = msg.targetPositions[0]
		self.RobotArm_ARM_TO_ARM_CORE_list[5] = msg.targetPositions[1]
		self.RobotArm_ARM_TO_ARM_CORE_list[6] = msg.targetPositions[2]
		self.RobotArm_ARM_TO_ARM_CORE_list[7] = msg.targetPositions[3]
		self.RobotArm_ARM_TO_ARM_CORE_list[8] = msg.targetPositions[4]
		self.RobotArm_ARM_TO_ARM_CORE_list[9] = msg.targetPositions[5]
		self.RobotArm_ARM_TO_ARM_CORE_list[10] = msg.targetVelocities[0]
		self.RobotArm_ARM_TO_ARM_CORE_list[11] = msg.targetVelocities[1]
		self.RobotArm_ARM_TO_ARM_CORE_list[12] = msg.targetVelocities[2]
		self.RobotArm_ARM_TO_ARM_CORE_list[13] = msg.targetVelocities[3]
		self.RobotArm_ARM_TO_ARM_CORE_list[14] = msg.targetVelocities[4]
		self.RobotArm_ARM_TO_ARM_CORE_list[15] = msg.targetVelocities[5]
		self.RobotArm_ARM_TO_ARM_CORE_list[16] = msg.targetAcceleration[0]
		self.RobotArm_ARM_TO_ARM_CORE_list[17] = msg.targetAcceleration[1]
		self.RobotArm_ARM_TO_ARM_CORE_list[18] = msg.targetAcceleration[2]
		self.RobotArm_ARM_TO_ARM_CORE_list[19] = msg.targetAcceleration[3]
		self.RobotArm_ARM_TO_ARM_CORE_list[20] = msg.targetAcceleration[4]
		self.RobotArm_ARM_TO_ARM_CORE_list[21] = msg.targetAcceleration[5]

		rospy.loginfo(msg)

	def getAddress(self):
		return self.address

	def rosSend(self):
		pass

	def parsData(self, raw_data):
	
		# store id
		rx_id = struct.unpack('B', raw_data[0:1])[0]

		recv_data = struct.unpack(''.join(self.id2types(rx_id, 'RX')), raw_data)

		self.setValuesInDataList(recv_data, rx_id, 'RX')

		self.publishData(rx_id)

	def publishData(self, rx_id):
		pass
		if rx_id == self.ID_ARM_CORE:
			# create message
			msg = ArmCoreToRobotarm()
			temp_dict = self.id2lists(rx_id, 'RX')

			msg.dataID = self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'dataID'), 'RX')
			msg.operationEnabled = self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'operationEnabled'), 'RX')
			msg.actualMode = self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'actualMode'), 'RX')
			msg.Endeffector_IsOpen = self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'Endeffector_IsOpen'), 'RX')
			msg.actualPositions = [	self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'acutalJointAngle1'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'acutalJointAngle2'), 'RX'),
									self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'acutalJointAngle3'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'acutalJointAngle4'), 'RX'),
									self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'acutalJointAngle5'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'acutalJointAngle6'), 'RX')]
			msg.actualVelocities = [ self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'actualJointVelocity1'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'actualJointVelocity2'), 'RX'),
									 self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'actualJointVelocity3'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'actualJointVelocity4'), 'RX'),
									 self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'actualJointVelocity5'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'actualJointVelocity6'), 'RX')]
			msg.HomeOffset = [	self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'HomeOffset1'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'HomeOffset2'), 'RX'),
								self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'HomeOffset3'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'HomeOffset4'), 'RX'),
								self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'HomeOffset5'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'HomeOffset6'), 'RX')]
			msg.targetPositions = [ self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointAngle1'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointAngle2'), 'RX'),
									self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointAngle3'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointAngle4'), 'RX'),
									self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointAngle5'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointAngle6'), 'RX')]
			msg.targetVelocities = [ self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointVelocity1'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointVelocity2'), 'RX'),
									 self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointVelocity3'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointVelocity4'), 'RX'),
									 self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointVelocity5'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointVelocity6'), 'RX')]
			msg.targetAcceleration  = [ self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointAcceleration1'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointAcceleration2'), 'RX'),
									 	self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointAcceleration3'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointAcceleration4'), 'RX'),
									 	self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointAcceleration5'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'targetJointAcceleration6'), 'RX')]
			msg.PositionReached = [ self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'reachedJointPosition1'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'reachedJointPosition2'), 'RX'),
									self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'reachedJointPosition3'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'reachedJointPosition4'), 'RX'),
									self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'reachedJointPosition5'), 'RX'), self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'reachedJointPosition6'), 'RX')]
			msg.RobotArmPositionReached = self.getValueByIndex(self.getIndexByKey(rx_id, 'RX', 'robotArmPositionReached'), 'RX')
			msg.dummy = [0]
			
			#rospy.loginfo(msg)
			
			# publish message
			self.arm_core_to_ik_solver.publish(msg)

	def get_byte_string(self):
		
		data_in_byte_object = bytes()

		for i, key in enumerate(self.RobotArm_ARM_TO_ARM_CORE_keys.values()):

			value = self.getValueByIndex(i, 'TX')

			data_in_byte_object += struct.pack(''+self.id2types(self.ID_ARM_CORE, 'TX')[i], value)

		return data_in_byte_object
	
	def getValueByIndex(self, index_, direction_):
		if direction_ is 'RX':
			return self.RobotArm_ARM_CORE_TO_ARM_list[index_]
			
		elif direction_ is 'TX':
			return self.RobotArm_ARM_TO_ARM_CORE_list[index_]

	def setValuesInDataList(self, recv_data, id_, direction_):
		if direction_ is 'RX':
			if id_ is ID_ARM_CORE:
				self.RobotArm_ARM_CORE_TO_ARM_list = recv_data

	def getIndexByKey(self, id_, direction_, key_):
		if direction_ is 'RX':
			if id_ is ID_ARM_CORE:
				return self.RobotArm_ARM_CORE_TO_ARM_keys[key_]
		
		if direction_ is 'TX':
			if id_ is ID_ARM_CORE:
				return self.RobotArm_ARM_TO_ARM_CORE_keys[key_]

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

	def id2lists(self, id_, direction_):
		if direction_ is 'RX':
			#get the array of the lists of the correspending type
			switcher = {	self.ID_ARM_CORE: self.RobotArm_ARM_CORE_TO_ARM
							}
		elif direction_ is 'TX':
			switcher = {	self.ID_ARM_CORE: self.RobotArm_ARM_TO_ARM_CORE
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

	def getValueByIndex(self, index_, direction_):
		if direction_ is 'TX':
			return self.RobotArm_ARM_TO_ARM_CORE_list[index_]

	def getIndexByKey(self, id_, direction_, key_):
		if direction_ is 'RX':
			if id_ is ID_ARM_CORE:
				return self.RobotArm_ARM_CORE_TO_ARM_keys[key_]
		
		if direction_ is 'TX':
			if id_ is ID_ARM_CORE:
				return self.RobotArm_ARM_TO_ARM_CORE_keys[key_]

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