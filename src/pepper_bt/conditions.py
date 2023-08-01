#!/usr/bin/env python

import typing
import py_trees
from py_trees import common
from pepper_bt.topics import Listening
import pepper_bt.blackboard_util as bl_util


class PermitRobotSpeak(py_trees.behaviour.Behaviour):

    def __init__(self):
        super(PermitRobotSpeak,self).__init__(name = "Permit Robot Speak")
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.blackboard = py_trees.Blackboard()
        self.blackboard.set(bl_util.speaking_is_running, value=False, overwrite=True)
 
    def update(self) :
        self.logger.debug("[%s::update()] - speaking_is_running: %r" % (self.__class__.__name__ , self.blackboard.get("speaking_is_running")))
        if self.blackboard.get(bl_util.speaking_is_running) == True:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        

class UserEngaged(py_trees.behaviour.Behaviour):

    def __init__(self , name = "User Engaged"):
        super(UserEngaged,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.blackboard = py_trees.Blackboard()
        # After FSM, the engaged status in BT should be changed from None to True
        if self.blackboard.get(bl_util.user_engaged) is None:
            self.blackboard.set(bl_util.user_engaged, value=True, overwrite=True)



    def update(self) :
        self.logger.debug("[%s::update()] - user engaged: %r" % (self.__class__.__name__ , self.blackboard.get(bl_util.user_engaged)))
        if self.blackboard.get(bl_util.user_engaged):
            if self.blackboard.get(bl_util.selected_painting) != None:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        else:
            # Todo: handle back to FSM if user not engaged
            # Todo: callback
            return py_trees.common.Status.FAILURE



class RobotSpeaking(py_trees.behaviour.Behaviour):

    def __init__(self):
        super(RobotSpeaking,self).__init__(name = "RobotSpeaking")
        self.logger.debug("  %s [Listening::__init__()]" % self.__class__.__name__)
        self.listener = Listening()

    def initialise(self):
        self.logger.debug("  %s [Listening::initialise()]" % self.__class__.__name__)

    def update(self):
        self.logger.debug("  %s [Listening::update()]" % self.__class__.__name__)
        self.listener.listen()
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        self.logger.debug("  %s [Listening::terminate()]" % self.__class__.__name__)
    

 

