from ArmCoreToRobotarm.msg import ArmCoreToRobotarm
from GroundstationToRobotarm.msg import GroundstationToRobotarm
from eth_schnittstelle import *

#Separate Datei mit den selben Konvertierungsfunktionen wie im Hauptskript. 

def convertfromros(data, kanal):
	#für switch-case evtl bessere Lösung suchen, z.B. mit dictionaries
	if kanal == "RobotarmToGroundstation":

		#um alle Attribute einer Klasse in einem String-Array zu kriegen: [a for a in dir(obj) if not a.startswith('__')]
		bytemsg = bytes()
		#bytesdone = 0
		#durchiterieren durch jedes Feld der ROS-msg und das Feld dann in ein byte-objekt umwandeln und der bytemsg dranhängen
		for i in range(0, len(DataClass.ARM_TO_GS_TYPES)):
			
			item = DataClass.ARM_TO_GS_KEYS.values()[i]
			#bytelen = len(getattr(data,item))
			#bytemsg.append(struct.pack(DataClass.ARM_TO_GS_TYPES[i],data[bytesdone:(bytesdone+bytelen)])
			#bytesdone = bytelen + 1
			if i == 0:
				bytemsg = struct.pack(''+DataClass.ARM_TO_GS_TYPES[i], item)
			else:
				bytemsg += struct.pack(''+DataClass.ARM_TO_GS_TYPES[i], item)

		for inst in UDPConn.InstanceList:
			if inst.ip == "" and inst.port == "":
				inst.senddata(bytemsg)

	elif kanal == "RobotarmToArmCore":
		bytemsg = bytes()
		for i in range(0, len(DataClass.ARM_TO_ARM_CORE_TYPES)):
			
			item = DataClass.ARM_TO_ARM_CORE_KEYS.values()[i]
			if i == 0:
				bytemsg = struct.pack(''+DataClass.ARM_TO_ARM_CORE_TYPES[i], item)
			else:
				bytemsg += struct.pack(''+DataClass.ARM_TO_ARM_CORE_TYPES[i], item)

		for inst in UDPConn.InstanceList:
			if inst.ip == "" and inst.port == "":
				inst.senddata(bytemsg)


def convertfromudp(data,addr):
	id = struct.unpack('B', data[0:1])[0]
	if id == DataClass.ID_ARM_CORE:
		
		#Wenn Erstellung der ROS-msg so nicht klappt, dann für jedes einzelne Feld den Wert zuweisen
		newmsg = ArmCoreToRobotarm(list(struct.unpack(''.join(DataClass.ARM_CORE_TO_ARM_TYPES), data)))

		for inst in ROSConn.InstanceList:
			if inst.kanal == "ArmCoreToRobotarm":
				inst.senddata(newmsg)

	elif id == DataClass.ID_ARM:

		newmsg = ArmCoreToRobotarm(list(struct.unpack(''.join(DataClass.GS_TO_ARM_TYPES), data)))

		for inst in ROSConn.InstanceList:
			if inst.kanal == "GroundstationToRobotarm":
				inst.senddata(newmsg)

#die Konvertierungsfunktionen später dann besser in separaten Threads laufen lassen
def init_convertthreads():
	convertthread1 = threading.Thread(target=convertfromros,daemon=True, name="rosconvert")
	convertthread1.start()
	convertthread2 = threading.Thread(target=convertfromudp,daemon=True, name="udpconvert")
	convertthread2.start()
	