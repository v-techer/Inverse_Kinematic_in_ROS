#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket  # udp services
import time  # time functions like e.g. sleep
import struct  # packing and unpacking byte objects in specified c types
import threading  # multi-threading and mutex
from collections import deque
import rospy         
from ai_core.DataClass import DataClass          

from ai_core.msg import ArmCoreToRobotarm
from ai_core.msg import GroundstationToRobotarm

#zu beachten:
#IO-Core ist noch nicht mit implementiert
#IDs in convertfromudp sind noch nicht geprüft


#--------------------------------------------------------------------------------------------------------------------------
#TODO: Initialisieren von Parametern, z.B. für alle verwendeten IPs und Ports konst. Variablen erstellen und diese im Code verwenden

#rosqueue = deque()
#udpqueue = deque()
#rossema = threading.Semaphore()
#udpsema = threading.Semaphore()

#--------------------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------------------
#Die Funktionen für die Datenknovertierung

def convertfromros(data, kanal):
	'''Wandelt eine ROS-Nachricht in eine bytes-Objekt um zum Versenden mit UDP'''
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

		#Hier IP und Port der Groundstation eintragen
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

		#Hier IP und Port des Arm Cores eintragen
		for inst in UDPConn.InstanceList:
			if inst.ip == "" and inst.port == "":
				inst.senddata(bytemsg)


def convertfromudp(data,addr):
	'''Wandelt einen UDP-Dataframe in eine ROS-Nachricht anhand der ID im 1.Datenbyte um'''
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

#Initialisiert separate Threads für die Konvertierungsfunktionen. Wird bis jetzt nicht verwendet.
def init_convertthreads():
	convertthread1 = threading.Thread(target=convertfromros, name="rosconvert")
	convertthread1.daemon = True
	convertthread1.start()
	convertthread2 = threading.Thread(target=convertfromudp, name="udpconvert")
	convertthread2.daemon = True
	convertthread2.start()

#---------------------------------------------------------------------------------------------------------------------------------------

class UDPConn:

	InstanceList = []
	serverip = "192.168.0.50"
	#Hier IP und Port des AI-Cores eintragen
	#serverip = "127.0.0.1"
	serverport = 2020
	serversocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
	setupflag = False
	serverthread = None

	def __init__(self, ip, port):
		#ip und port, wohin konvertierte nachrichten weitergeleitet werden
		self.ip = ip 
		self.port = port
		self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

	@classmethod
	def init_server(cls):
		'''initialisiert einen separaten Thread zum Empfangen von UDP-Nachrichten'''
		if cls.setupflag == False:
			cls.serversocket.bind((cls.serverip,cls.serverport))
			cls.serverthread = threading.Thread(target=cls.server_receivemsg, name="serverthread")
			cls.serverthread.daemon = True
			cls.serverthread.start()
			cls.setupflag = True

	@classmethod
	def server_receivemsg(cls):
		'''Callback-Methode für den Empfangsthread von UDP-Nachrichten'''
		while True:
			data, addr = cls.serversocket.recvfrom(1024)
			convertfromudp(data,addr)
			#Übergabe am besten mit Queue

	def sendmsg(self, data):
		self.sock.sendto(data,(self.ip,self.port))

	@classmethod
	def addtoinstancelist(cls, udpinstance):
		'''fügt erstellte Klasseninstanzen in eine Liste hinzu, um später eine Übersicht zu haben'''
		if isinstance(udpinstance,UDPConn):
			cls.InstanceList.append(udpinstance)
		else:
			print("Fehler in addToInstanceList-Methode")



class ROSConn:
	'''Speichert Informationen zu einem Topic, mit dem der AI-Core verbunden ist und liefert Methoden zum Senden und Empfangen von diesem Topic'''		
	InstanceList = []
	
	def __init__(self, kanal, msgtyp):
		self.kanal = kanal
		self.msgtyp = msgtyp
		self.pub = rospy.Publisher(kanal, msgtyp, queue_size=20)
		rospy.Subscriber(kanal, msgtyp, self.roscallback)

	def roscallback(self,data):
		"""Callback-Methode, die die ROSConn-Instanz bei empfangener Nachricht aufruft"""
		#print("von ", self.kanal, " empfangen")
		#da von diesen Topics die Schnittstelle nichts empfangen muss
		if self.kanal != "GroundstationToRobotarm" and self.kanal != "ArmCoreToRobotarm":
			convertfromros(data,self.kanal)
			#Übergabe am besten mit Queue

	def senddata(self, data):
		"""publisht eine Nachricht auf den jeweiligen Topic der Klasseninstanz"""
		if self.msgtyp == type(data):
			self.pub.publish(data)
			#print(self.kanal, "sendet")
		else:
			print("Fehler beim Senden: Nachricht für Topic", self.kanal, "hat falschen msg-typ.")

	@classmethod
	def addtoinstancelist(cls, rosinstance):
		'''die gleiche Methode wie in UDPConn'''
		if isinstance(rosinstance,cls):
			cls.InstanceList.append(rosinstance)
		else:
			print("Fehler in addToInstanceList-Methode")
	

if __name__ == '__main__':

	UDPConn.init_server()
	rospy.init_node("ai_core", anonymous=True) 

	#-------------------------------------------------------------------------------------------------------------------------
	# Hier festlegen, mit welchen Topics/UDP-Clients kommuniziert werden soll. 
	# Für jedes ROS-Topic und UDP-Verbindung eine Klasseninstanz erstellen und in die Instanzliste hinzufügen

	test = DataClass.ID_SAFETY
	to_ik_solver1 = ROSConn("groundstation_to_ik_solver", GroundstationToRobotarm)
	ROSConn.addtoinstancelist(to_ik_solver1)
	to_ik_solver2 = ROSConn("arm_core_to_ik_solver", ArmCoreToRobotarm)
	ROSConn.addtoinstancelist(to_ik_solver2)
	from_ik_solver1 = ROSConn("ik_solver_to_groundstation", GroundstationToRobotarm)
	ROSConn.addtoinstancelist(from_ik_solver1)
	from_ik_solver2 = ROSConn("ik_solver_to_arm_core", ArmCoreToRobotarm)
	ROSConn.addtoinstancelist(from_ik_solver2)

	groundstation = UDPConn("192.168.0.21",2020)
	UDPConn.addtoinstancelist(groundstation)
	arm_core = UDPConn("192.168.0.25",2020)
	UDPConn.addtoinstancelist(arm_core)
	print("Kontrolle")

	#---------------------------------------------------------------------------------------------------------------------------

	rospy.spin()
