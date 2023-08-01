#!/usr/bin/env python

import py_trees
from pepper_bt.services import Pepper
import pepper_bt.configs as cfg
import pepper_bt.blackboard_util as bl_util
import pepper_bt.constant as const


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
    

class Speaking(py_trees.behaviour.Behaviour):

    def __init__(self, message):
        super(Speaking,self).__init__(name = "Robot Is Speaking")
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = Pepper(cfg.IP_ADDRESS, cfg.PORT)
        self.blackboard = py_trees.Blackboard()
        self.msg = message

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)

        self.blackboard.set(bl_util.speaking_is_running,True,overwrite=True)
        self.pepper.say(self.msg)
        self.blackboard.set(bl_util.speaking_is_running,False,overwrite=True)

        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class EngageUser(py_trees.behaviour.Behaviour):

    def __init__(self, name="Engage User"):
        super(EngageUser,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = Pepper(cfg.IP_ADDRESS, cfg.PORT)
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)

        self.pepper.say(const.PRESENTATION_HELLO)

        self.pepper.presentation_gesture()

        if self.pepper.tablet_show_web():
            self.blackboard.set(bl_util.paint_is_chosen,True,overwrite=True)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)