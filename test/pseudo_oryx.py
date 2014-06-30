#!/usr/bin/env python

import roslib; roslib.load_manifest('voice_control')
import rospy
from std_msgs.msg import String
from glass_server.msg import TextMessage
from glass_server.srv import RobotConfiguration
import os


class PseudoOryx:
    def __init__(self):

        #Subscribers
        self._command_listener = rospy.Subscriber("/oryx/voice_command", String, self.voiceCommandCallback)

        #Publishers
        self._text_message_pub = rospy.Publisher("/glass_server/text_messages", TextMessage)

        rospy.wait_for_service('robot_configuration')
        success = False
        try:
            self._robot_configuration_srv = rospy.ServiceProxy('robot_configuration', RobotConfiguration)
            name = "Oryx"
            info = "Planetary exploration mobility platform equiped with three cameras and a scoop"
            this_dir = os.path.dirname(__file__)
            filename = os.path.join(this_dir, 'oryx_vocab.xml')
            vocab_file = open(filename, "r")
            vocab = vocab_file.read()
            rospy.loginfo("Sending configuration")
            success = self._robot_configuration_srv(name, info, vocab)
            vocab_file.close()
            rospy.loginfo("Configuration sent")
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        if not success:
            rospy.loginfo("Configuration unsuccessful")
            exit()
        rospy.loginfo("Configuration successful")

        self.sendTextMessage("Oryx is online\nSystems are ready\nAwaiting commands")


    def voiceCommandCallback(self, msg):
        rospy.loginfo("Pseudo Oryx received command: " + msg.data)
        self.sendTextMessage("I received your command: " + msg.data)

    def sendTextMessage(self, text):
        rospy.loginfo("Sending text message")
        msg = TextMessage()
        msg.sender = "Oryx"
        msg.text = text
        msg.priority = 0

        self._text_message_pub.publish(msg)


if __name__=="__main__":
    try:
        rospy.init_node('pseudo_oryx')
        PseudoOryx()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Oryx going offline")