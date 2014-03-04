#!/usr/bin/env python

import rospy
from bluetooth import *
from std_msgs.msg import String

class GlassServer():

	def __init__(self):
		#Global Variables
		self._server_sock = BluetoothSocket( RFCOMM )
		self._server_sock.bind(("",PORT_ANY))
		self._server_sock.listen(1)

		self._client_sock = None
		self._client_info = None

		self._uuid = "00001101-0000-1000-8000-00805F9B34FB"

		self._string_buffer = ""
		self._string_buffer_flag = False

		self._robotDict = {"Anna" : ["Status: Normal", \
									"Velocity: 0 m/s",\
									"Command: Do Nothing",\
									"BCI: Disengaged"
									],\
									"Oryx" : ["Status: Off", \
									"Velocity: 0 m/s",\
									"Command: Do Nothing",\
									"BCI: None"
									]
							}

		#Subscribers

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
		rospy.loginfo("Accepted connection from "  + str(self._client_info))

		try:
			while not rospy.is_shutdown():
				data = self._client_sock.recv(1024)
				if len(data) > 0:
					rospy.loginfo("received: \"%s\"" % data)

				if data == "end connection\n": 
					break
				elif data == "Confirm connection\n":
					self._client_sock.send("Connection confirmed\n")
				else:
					self._client_sock.send("Copy: %s\n" % data)
					msg = String()
					msg.data = data
					self._voice_pub.publish(msg)

				if self._string_buffer_flag == True:
					self._client_sock.send(self._string_buffer)
					self._string_buffer_flag = False

			rospy.loginfo("Disconnecting")
			self._client_sock.close()
			self._server_sock.close()
			rospy.loginfo("all done")

		except IOError:
			rospy.loginfo("Error")
			traceback.print_exc()


	def writeToGlass(self, string_data):
		self._string_buffer = string_data
		self._string_buffer_flag = True

	def writeRobotInfo(self):
		rospy.loginfo("Writing robot information to Glass")

		self.writeToGlass("Sending Robot Information:\n")

		for key, value in self._robotDict.iteritems():
			self._writeToGlass(key)

			for info in value:
				self._writeToGlass(info)
				self._writeToGlass("_Next\n")

		self._writeToGlass("_End\n")
		rospy.loginfo("Finished writing robot information to Glass")



if __name__ == '__main__':
	try:
		rospy.init_node('glass_server')
		rospy.loginfo("Node Initialized")
		instance = GlassServer()
	except rospy.ROSInterruptException:
		pass