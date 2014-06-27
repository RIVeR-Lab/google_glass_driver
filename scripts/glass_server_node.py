#!/usr/bin/env python

import rospy
from bluetooth import *
import select
from std_msgs.msg import String
from glass_server.srv import RobotConfiguration
from glass_server.msg import TextMessage

class GlassServer():

	def __init__(self):
		#Global Variables
		self._server_sock = BluetoothSocket( RFCOMM )
		self._server_sock.bind(("",PORT_ANY))
		self._server_sock.listen(1)

		self._client_sock = None
		self._client_info = None

		self._uuid = "00001101-0000-1000-8000-00805F9B34FB"

		self._send_queue = []
		self._config_success = False

		self._robotDict = {}

		self._config_srv = rospy.Service("robot_configuration", RobotConfiguration, self.robotConfig)

		#Subscribers
		self._text_sub = rospy.Subscriber("/glass_server/text_message", TextMessage, self.textMessageCallback)

		#Publishers
		self._voice_pub = rospy.Publisher("/recognizer/output", String)

		self.run()

	def run(self):
		port = self._server_sock.getsockname()[1]

		advertise_service( self._server_sock, "GlassServer",
		   service_id = self._uuid,
		   service_classes = [ self._uuid, SERIAL_PORT_CLASS ],
		   profiles = [ SERIAL_PORT_PROFILE ], 
#		   protocols = [ OBEX_UUID ] 
					)

		rospy.loginfo("Waiting for connection on RFCOMM channel %d" % port)

		self._client_sock, self._client_info = self._server_sock.accept()
		self._client_sock.setblocking(0)

		rospy.loginfo("Accepted connection from "  + str(self._client_info))

		try:
			while not rospy.is_shutdown():
				data = ""

				ready = select.select([self._client_sock], [], [], 0.2)
				if ready[0]:
					data = self._client_sock.recv(1024)
				if len(data) > 0:
					rospy.loginfo("received: \"%s\"" % data)

				if data == "end connection\n": 
					break
				elif data == "Confirm connection\n":
					self.writeToGlass("Connection confirmed\n")
				elif data == "configuration complete\n":
					self._client_sock.send("Copy: %s\n" % data)
					self._config_success = True
				elif not data == "":
					self._client_sock.send("Copy: %s\n" % data)
					msg = String()
					msg.data = data
					self._voice_pub.publish(msg)

				if not self._send_queue == []:
					rospy.loginfo("Sending queue: " + str(self._send_queue))
					for string in self._send_queue:
						self._client_sock.send(string)
					self._send_queue = []
					

			rospy.loginfo("Disconnecting")
			self._client_sock.close()
			self._server_sock.close()
			rospy.loginfo("all done")

		except IOError, e:
			rospy.loginfo("Error")
			print e


	def writeToGlass(self, string_data):
		rospy.loginfo("Adding " + string_data + " to queue " + str(self._send_queue))
		self._send_queue.append(string_data)

	def robotConfig(self, msg):
		rospy.loginfo("Recieved configuration for " + msg.name)
		stringToSend = "robot_configuration" + "_DELIM_" + msg.name \
											+ "_DELIM_" + msg.info \
											+ "_DELIM_" + msg.vocab \
											+ "_END"
		self.writeToGlass(stringToSend)
		#self.writeToGlass("robot_configuration")
		#self.writeToGlass(msg.name)
		#self.writeToGlass(msg.info)
		#self.writeToGlass(msg.vocab)
		self._robotDict[msg.name] = rospy.Publisher("/" + msg.name.lower().replace(" ", "_") + "/voice_command", String)
		rospy.loginfo("Configuration written to Glass. Waiting for response")

		while(not self._config_success):
			pass
		self._config_success = False

		return True

	def textMessageCallback(self, msg):
		self.writeToGlass("text_message")
		self.writeToGlass(msg.sender)
		self.writeToGlass(msg.text)
		self.writeToGlass(msg.priority)

	def imgMessageCallback(self, sender, msg):
		self.writeToGlass("Image")
		self.writeToGlass(msg.sender)
		self.writeToGlass(msg.text)
		self.writeToGlass(msg.base64Image)
		self.writeToGlass(msg.priority)



if __name__ == '__main__':
	try:
		rospy.init_node('glass_server')
		rospy.loginfo("Node Initialized")
		instance = GlassServer()
	except rospy.ROSInterruptException:
		pass