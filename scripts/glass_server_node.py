#!/usr/bin/env python

import rospy
from bluetooth import *
import select
import base64
from std_msgs.msg import String as StringMsg
from glass_server.srv import RobotConfiguration
from glass_server.msg import TextMessage
from glass_server.msg import ImageMessage
from PIL import Image
import cStringIO as StringIO
from threading import Lock, Event
import itertools
import logging

class GlassServer():

    def __init__(self):
        #Global Variables
        self._server_sock = BluetoothSocket( RFCOMM )
        self._server_sock.bind(("",PORT_ANY))
        self._server_sock.listen(1)

        self._client_sock = None
        self._client_info = None

        self._uuid = "00001101-0000-1000-8000-00805F9B34FB"

        self._send_queue = ""
        self._config_success = False

        self._robotDict = {}

        self._lock = Lock()
        self._talkback_event = Event()

        self._config_srv = rospy.Service("robot_configuration", RobotConfiguration, self.robotConfig)

        #Subscribers
        self._text_sub = rospy.Subscriber("/glass_server/text_messages", TextMessage, self.textMessageCallback)
        self._image_sub = rospy.Subscriber("/glass_server/image_messages", ImageMessage, self.ImageMessageCallback)

        #Publishers
        self._voice_pub = rospy.Publisher("/recognizer/output", StringMsg)

        self.run()

    def run(self):
        port = self._server_sock.getsockname()[1]

        advertise_service( self._server_sock, "GlassServer",
           service_id = self._uuid,
           service_classes = [ self._uuid, SERIAL_PORT_CLASS ],
           profiles = [ SERIAL_PORT_PROFILE ], 
#          protocols = [ OBEX_UUID ] 
                    )

        rospy.loginfo("Waiting for connection on RFCOMM channel %d" % port)

        self._client_sock, self._client_info = self._server_sock.accept()
        self._client_sock.setblocking(1)

        rospy.loginfo("Accepted connection from "  + str(self._client_info))

        try:
            while not rospy.is_shutdown():
                data = ""

                #ready = select.select([self._client_sock], [], [], 0)
                #if ready[0]:
                data = self._client_sock.recv(1024)

                if len(data) > 0:
                    rospy.loginfo("received: \"%s\"" % data)

                    if data == "end connection\n": 
                        break
                    elif data == "Confirm connection\n":
                        self._lock.acquire()
                        self._client_sock.send("Connection confirmed\n")
                        self._lock.release()
                    elif data == "configuration complete\n":
                    #   self._client_sock.send("Copy: %s\n" % data)
                        self._config_success = True
                    elif data == "Delivery failed":
                        pass
                    elif data == "_NEXT_\n":
                        self._talkback_event.set()
                    elif not data == "":
                    #   rospy.loginfo("Sending copy")
                    #   self._client_sock.send("Copy: %s" % data)
                        msg = StringMsg()
                        msg.data = data
                        self._voice_pub.publish(msg)
                        self.sendCommandToRobot(data)
                    

            rospy.loginfo("Disconnecting")
            self._client_sock.close()
            self._server_sock.close()
            rospy.loginfo("all done")

        except IOError, e:
            rospy.loginfo("Error")
            print e


    def writeToGlass(self, string_data):
        rospy.loginfo("Sending: " + string_data)
        success = False
        try:
            self._lock.acquire()
            #headerSize = 30 #bytes
            packetSize = 10000 #bytes
            numPackets = len(string_data) / packetSize + (1 if (len(string_data) % packetSize > 0) else 0)

            rospy.loginfo("Setting up header")
            first_msg = "<msg>" + str(numPackets) + "_DELIM_" + str(packetSize) + "</msg>"
            #slack = headerSize - len(first_msg)
            #first_msg += " " * slack
            rospy.loginfo("Sending header")
            self._client_sock.send(first_msg)

            rospy.loginfo("Creating groups")
            groups = self.grouper(string_data, packetSize)
            rospy.loginfo(groups)
            rospy.loginfo("Creating and sending packets")
            for i in range(len(groups)):
                msg = "<pkt><index>" + str(i+1) + "</index>" + groups[i] + "</pkt>"
                pkt_slack = packetSize - len(msg)
                msg += " " * pkt_slack
                rospy.loginfo("Sending")
                self._client_sock.send(msg)
                rospy.loginfo("Sent " + str(i+1) + " packets! Size=" + str(len(msg)))
                self._talkback_event.wait()
                self._talkback_event.clear()

            rospy.loginfo("Message sent")
            success = True
        except Exception, ex:# OSError:
            logging.exception("Error on send!")
            rospy.loginfo("Error on send!")
        finally:
            rospy.loginfo("Releasing write lock")
            self._lock.release()

    def robotConfig(self, msg):
        rospy.loginfo("Recieved configuration for " + msg.name)
        stringToSend = "robot_configuration" + "_DELIM_" + msg.name \
                                            + "_DELIM_" + msg.info \
                                            + "_DELIM_" + msg.vocab
        self.writeToGlass(stringToSend)
        self._robotDict[msg.name] = rospy.Publisher("/" + msg.name.lower().replace(" ", "_") + "/voice_command", StringMsg)
        rospy.loginfo("Configuration written to Glass. Waiting for response")

        while(not self._config_success):
            pass
        self._config_success = False

        rospy.loginfo("Sending service response")
        return True

    def sendCommandToRobot(self, data):
        splitData = data.split(":")
        targetRobot = splitData[0]
        command = splitData[1][1:]

        publisher = self._robotDict[targetRobot]
        msg = StringMsg()
        msg.data = command
        publisher.publish(msg)

    def textMessageCallback(self, msg):
        rospy.loginfo("Recieved text message from " + msg.sender)
        stringToSend = "text_message" + "_DELIM_" + msg.sender \
                                            + "_DELIM_" + msg.text \
                                            + "_DELIM_" + str(msg.priority)
        self.writeToGlass(stringToSend)

    def ImageMessageCallback(self, msg):
        rospy.loginfo("Received image message from " + msg.sender)

        image_str = msg.base64_image
        image_raw = base64.b64decode(image_str)
        stream = StringIO.StringIO(image_raw)
        image = Image.open(stream)      
        width, height = image.size
        if width > height:
            newW = 100
            newH = newW * height / width
        else:
            newH = 100
            newW = newH * width / height
        newSize = newW, newH
        image = image.resize(newSize)

        image_stream = StringIO.StringIO()
        image.save(image_stream, format="JPEG")
        image_str = base64.b64encode(image_stream.read())

        stringToSend = "image_message" + "_DELIM_" + msg.sender \
                                            + "_DELIM_" + msg.text \
                                            + "_DELIM_" + msg.base64_image \
                                            + "_DELIM_" + str(msg.priority)
        self.writeToGlass(stringToSend)

    def grouper(self, seq, size):
        return [seq[pos:pos + size] for pos in range(0, len(seq), size)]


if __name__ == '__main__':
    try:
        rospy.init_node('glass_server')
        rospy.loginfo("Node Initialized")
        instance = GlassServer()
    except rospy.ROSInterruptException:
        pass