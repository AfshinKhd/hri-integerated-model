#!/usr/bin/env python

import typing
import py_trees
from py_trees import common
from pepper_bt.topics import Listening
from pepper_bt.blackboard_util import BlackboardItems

# Todo: Delete
# class PermitRobotSpeak(py_trees.behaviour.Behaviour):

#     def __init__(self):
#         super(PermitRobotSpeak,self).__init__(name = "Permit Robot Speak")
#         self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
#         self.blackboard = py_trees.Blackboard()
#         self.blackboard.set(bl_util.speaking_is_running, value=False, overwrite=True)
 
#     def update(self) :
#         self.logger.debug("[%s::update()] - speaking_is_running: %r" % (self.__class__.__name__ , self.blackboard.get("speaking_is_running")))
#         if self.blackboard.get(bl_util.speaking_is_running) == True:
#             return py_trees.common.Status.SUCCESS
#         else:
#             return py_trees.common.Status.FAILURE
        

class UserEngaged(py_trees.behaviour.Behaviour):

    def __init__(self , name = "User Engaged"):
        super(UserEngaged,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.blackboard = py_trees.Blackboard()
        # After FSM, the engaged status in BT should be changed from None to True
        if self.blackboard.get(BlackboardItems.USER_ENGAGED.value) is None:
            self.blackboard.set(BlackboardItems.USER_ENGAGED.value, value=True, overwrite=True)



    def update(self) :
        self.logger.debug("[%s::update()] - user engaged: %r - painting: %s" % (self.__class__.__name__ , self.blackboard.get(BlackboardItems.USER_ENGAGED.value) ,self.blackboard.get(BlackboardItems.SELECTED_PAINTING.value)))
        if self.blackboard.get(BlackboardItems.USER_ENGAGED.value):
            if self.blackboard.get(BlackboardItems.SELECTED_PAINTING.value) != None:
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


class UserTurn(py_trees.behaviour.Behaviour):

    def __init__(self,pepper, knowedge_manager, name="User is Allowed Turn"):
        super(UserTurn,self).__init__(name = name)
        self.logger.debug("  %s [Listening::__init__()]" % self.__class__.__name__)
        self.knowedge_manager = knowedge_manager
        
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("  %s [Listening::initialise()]" % self.__class__.__name__)

    def update(self):
        self.logger.debug("  %s [Listening::update()]" % self.__class__.__name__)
        # This means only user selected the painting and there is no dialogs yet
        print("User Turrrrrrrrrrrrn: " , str(self.knowedge_manager))
        if len(self.knowedge_manager) == 0:
            return py_trees.common.Status.SUCCESS
            #self.blackboard.set(BlackboardItems.USERTURN, value=True, overwrite=True )
        else:
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        self.logger.debug("  %s [Listening::terminate()]" % self.__class__.__name__)
    

 

