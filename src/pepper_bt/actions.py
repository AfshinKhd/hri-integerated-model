#!/usr/bin/env python

import py_trees
from pepper_bt.services import Pepper
import pepper_bt.configs as cfg
import pepper_bt.blackboard_util as bl_util
import pepper_bt.constant as const
from pepper_bt.topics import Speaking as SyncSpeak
 
class ShowPainting(py_trees.behaviour.Behaviour):

    def __init__(self,imageUrl):
        super(ShowPainting,self).__init__(name = "Show Painting on Tablet")
        self.img_url = imageUrl
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.blackboard = py_trees.Blackboard()
        self.pepper_robot = Pepper(cfg.IP_ADDRESS, cfg.PORT)
 
    def update(self) :
        self.logger.debug("[%s::update]" % self.__class__.__name__)

        self.pepper_robot.tablet_show_image(self.img_url)
        self.blackboard.set(bl_util.tablet_is_showing_paint,value=True,overwrite=True)
        self.blackboard.set(bl_util.tablet_is_showing_paint, value=False, overwrite=True)

        return py_trees.common.Status.SUCCESS
    

class RobotStartSpeaking(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, message):
        super(RobotStartSpeaking,self).__init__(name = "Robot Is Speaking")
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.blackboard = py_trees.Blackboard()
        self.msg = message

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)

        # Gesture 
        self.pepper.present_gesture()
        # Speech
        self.blackboard.set(bl_util.speaking_is_running,True,overwrite=True)
        self.pepper.say(self.msg)
        self.blackboard.set(bl_util.speaking_is_running,False,overwrite=True)

        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class EngageUser(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, name="Engage User"):
        super(EngageUser,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)

        # Speech
        speech = SyncSpeak()
        speech.say(const.PRESENTATION_HELLO)
        # Gesture
        self.pepper.present_gesture()
     
        if self.pepper.tablet_show_web():           
            # Waiting for action of user
            app = self.pepper.tablet_touch_handling()
            selected_painting = self.pepper._touch_dwon_feedback(app)
            print("selected painting : " ,selected_painting)
            if selected_painting is None :
                return py_trees.common.Status.FAILURE
            else:
                self.blackboard.set(bl_util.selected_painting, selected_painting, overwrite=True)
                return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        
    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class RobotTakesTurn(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, name="Robot Takes Turn"):
        super(RobotTakesTurn,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)

        self.pepper.set_speech_speed()
        self.pepper.say("Selected Painting is " + self.blackboard.get(bl_util.selected_painting))
        self.pepper.reset_speach_speed()

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)