#!/usr/bin/env python

import py_trees
from pepper_bt.services import Pepper
import pepper_bt.configs as cfg
from pepper_bt.blackboard_util import BlackboardItems, Backchannel
import pepper_bt.constant as const
from pepper_bt.topics import Speaking as SyncSpeak
from pepper_bt.knowledge_manager import KnowledgeManager, UtteranceType,KnowledgeManagerHelper
import time
 
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
        self.blackboard.set(BlackboardItems.TABLET_IS_SHOWING_PAINT,value=True,overwrite=True)
        self.blackboard.set(BlackboardItems.TABLET_IS_SHOWING_PAINT, value=False, overwrite=True)

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
        self.blackboard.set(BlackboardItems.SPEAKING_IS_RUNNING,True,overwrite=True)
        self.pepper.say(self.msg)
        self.blackboard.set(BlackboardItems.SPEAKING_IS_RUNNING,False,overwrite=True)

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
                self.blackboard.set(BlackboardItems.SELECTED_PAINTING, selected_painting, overwrite=True)
                return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        
    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class RobotTakesTurn(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowedge_manager,name="Robot Takes Turn"):
        super(RobotTakesTurn,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.knowedge_manager = knowedge_manager
        print("knowedge_manager is \n",str(knowedge_manager))
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)

        self.knowedge_manager.add(UtteranceType.ROBOT, dialog=Backchannel.CONFIRM, backchannel=True)
        self.pepper.set_speech_speed(75)
        self.pepper.say(Backchannel.CONFIRM)
        self.pepper.reset_speach_speed()

        time.sleep(3)

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class ProcessUserInput(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowedge_manager, name="Process User Input"):
        super(ProcessUserInput,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.knowedge_manager = knowedge_manager
        print("knowedge_manager : \n", knowedge_manager)
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)
        
        # if len(self.knowedge_manager) == 0:
        #     dialog = "Selected Painting is " + self.blackboard.get(BlackboardItems.SELECTED_PAINTING)
        #     self.knowedge_manager.add(UtteranceType.INIT, dialog, backchannel=False)

        # Todo: In NLP it works

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class ReactUserInput(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowedge_manager, name="Process User Input"):
        super(ReactUserInput,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.knowedge_manager = knowedge_manager
        print("knowedge_manager : \n", knowedge_manager)
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)
        
        helper = KnowledgeManagerHelper(self.knowedge_manager)

        if helper.is_initial():
            dialog = "Selected Painting is " + self.blackboard.get(BlackboardItems.SELECTED_PAINTING)
            self.knowedge_manager.add(UtteranceType.INIT, dialog, backchannel=False)


        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class EnsureUserAttention(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, name="Ensure User Attention"):
        super(EnsureUserAttention,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)
        
        self.pepper.speech_gesture()
        time.sleep(.5)

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)