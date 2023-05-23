#!/usr/bin/env python

import py_trees
import rospy
from std_msgs.msg import String

class Speaking(py_trees.behaviour.Behaviour):

    def __init__(self, message):
        super(Speaking,self).__init__(name = "User Is Speaking")
        self.logger.debug("  %s [Speaking::__init__()]" % self.__class__.__name__)
        self.state = py_trees.Blackboard()
        self.msg = message

    def initialise(self):
        self.logger.debug("  %s [Speaking::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("  %s [Speaking::update()]" % self.__class__.__name__)
        rospy.init_node('pepper_speaker')
        publisher = rospy.Publisher('/speech', String, queue_size=10)
        rospy.sleep(2.0)
        publisher.publish(self.msg)
        self.feedback_message = "Processing finished"
        self.state.set("speaking_is_running",True,overwrite=True)
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        self.logger.debug("  %s [Speaking::terminate()]" % self.__class__.__name__)
        #self.state.set("speaking_is_running",False,overwrite=True)


class Move(py_trees.behaviour.Behaviour):
    pass