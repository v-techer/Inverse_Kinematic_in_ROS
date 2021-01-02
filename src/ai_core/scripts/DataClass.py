class DataClass:
  '''Klasse zur Speicherung von ID und Aufbau von den verschiedenen Nachrichtenarten'''
  #Übernommen aus den vorherigen Implementierungen der Ethernet-Schnittstelle


  # IDs, anhand die Nachrichten verarbeitet und weitergeleitet werden
  ID_ERROR = 0
  ID_BMS = 2
  ID_WEIGHTCELL = 3
  ID_SECURITY = 4
  ID_SAFETY = 5
  ID_ENVIRONMENTSENSOR = 6
  ID_DRILL = 7
  ID_ARM = 8
  ID_POWERTRAIN = 9
  ID_SYSTEMSTATE = 10
  ID_BOSCHEMU = 11
  ID_JETSONFUSED = 12
  ID_ARM_CORE = 13

  BoschEMU_EMU_GS_types = [	'B',  # dataID
                              'f',  # xEuler
                              'f',  # yEuler
                              'f',  # zEuler
                              'f',  # xLinearAcc
                              'f',  # yLinearAcc
                              'f']  # zLinearAcc

  BoschEMU_EMU_GS_keys = {	"dataID": 0,
                              "xEuler": 1,
                              "yEuler": 2,
                              "zEuler": 3,
                              "xLinearAcc": 4,
                              "yLinearAcc": 5,
                              "zLinearAcc": 6}

  #Wird im ROS-Topic RobotarmToArmCore verwendet
  ARM_TO_ARM_CORE_types = [	'B',
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

  ARM_TO_ARM_CORE_keys = { 	"dataID": 0,
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

  #Wird im ROS-Topic ArmCoreToRobotarm verwendet
  ARM_CORE_TO_ARM_types = [	'B',
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

  ARM_CORE_TO_ARM_keys = { 	"dataID": 0,
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

  #Wird im ROS-Topic GroundstationToRobotarm verwendet
  GS_TO_ARM_types = [	'B',
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

  GS_TO_ARM_keys = { 	"dataID": 0,
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

  #Wird vom ROS-Topic RobotarmToGroundstation verwendet
  ARM_TO_GS_types = [	'B',
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

  ARM_TO_GS_keys = { 	'dataID' : 0,
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

  def __init__(self):
  #List wird in dieser Implementierung nicht benötigt, da die Werte in der Konvertierungsfunktion direkt weitergegeben werden

    self.BoschEMU_EMU_GS_list = [	self.ID_BOSCHEMU,  # dataID
                    0.0,  # xEuler
                    0.0,  # yEuler
                    0.0,  # zEuler
                    0.0,  # xLinearAcc
                    0.0,  # yLinearAcc
                    0.0]  # zLinearAcc
    
    self.BoschEMU_EMU_GS = [	self.BoschEMU_EMU_GS_list,
                  self.BoschEMU_EMU_GS_types,
                  self.BoschEMU_EMU_GS_keys]
      
		

  def id2types(self, id_):
	# get types of a package by passing corresponding ID
	  switcher = {self.ID_BOSCHEMU: self.BoschEMU_EMU_GS_types, self.ID_JETSONFUSED: self.Jetson_FusedPosition_GS_types}
	  return switcher.get(id_, "WARNING: id2types -> invalid ID")

  def id2keys(self, id_):
	# get data of a package by passing corresponding ID
	  switcher = {	self.ID_BOSCHEMU: self.BoschEMU_EMU_GS_keys, self.ID_JETSONFUSED: self.Jetson_FusedPosition_GS_keys}
	  return switcher.get(id_, "WARNING: id2keys -> invalid ID")

  def id2lists(self, id_):
	#get the array of the lists of the correspending type
	  switcher = {	self.ID_BOSCHEMU: self.BoschEMU_EMU_GS, self.ID_JETSONFUSED: self.Jetson_FusedPosition_GS}
	  return switcher.get(id_, "WARNING: id2lists -> invalid ID")