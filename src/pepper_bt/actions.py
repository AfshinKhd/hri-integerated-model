#!/usr/bin/env python

import py_trees
from pepper_bt.services import Pepper
import pepper_bt.configs as cfg
from pepper_bt.util import *
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
    

class RobotStartsSpeaking(py_trees.behaviour.Behaviour):

    def __init__(self, pepper,knowledge_manager, name="Robot Starts Speaking"):
        super(RobotStartsSpeaking,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.knowledge_manager = knowledge_manager
        self.blackboard = py_trees.Blackboard()


    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)

        # Gesture 
        #self.pepper.present_gesture()
        # utterance
        # Todo : check robot utterance
        item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        self.blackboard.set(BlackboardItems.SPEAKING_IS_RUNNING.value,True,overwrite=True)
        self.pepper.set_speech_speed(95)
        self.pepper.say(KnowledgeManager.get_item_utterance(item))
        self.pepper.reset_speach_speed()
        self.blackboard.set(BlackboardItems.SPEAKING_IS_RUNNING.value,False,overwrite=True)  
        
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

        # utterance
        utterance = SyncSpeak()
        utterance.say(const.PRESENTATION_HELLO)
        # Gesture
        self.pepper.present_gesture()
        
        if self.pepper.tablet_show_web():           
            # Waiting for action of user
            app = self.pepper.tablet_touch_handling()
            coordinate = self.pepper._touch_down_feedback(app)

            # Todo: dynamic numbers
            if coordinate['x'] < 690 and coordinate['x'] > 80:
                selected_painting = const.judgment_of_cambyses_painting['name']
            elif coordinate['x'] > 690 and coordinate['x']< 1680:
                selected_painting = const.scream_painting['name']
            else:
                print("coordinate is wrong!")

            if selected_painting is None :
                return py_trees.common.Status.FAILURE
            else:
                self.blackboard.set(BlackboardItems.SELECTED_PAINTING.value, selected_painting, overwrite=True)
                return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        
    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class RobotTakesTurn(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name="Robot Takes Turn"):
        super(RobotTakesTurn,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.knowledge_manager = knowledge_manager
        print("knowledge_manager is \n",str(knowledge_manager))
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)

        item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        helper = KnowledgeManagerHelper(self.knowledge_manager)


        if helper.is_rate_response():
            return py_trees.common.Status.FAILURE
        
        if self.knowledge_manager.is_robot_utterance(item):
            if helper.is_init_response(self.knowledge_manager.get_tag(item)):
                dialog = Backchannel.CONFIRM.value
                self.knowledge_manager.add_item(UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(item), backchannel=True)
                self._say(dialog)
                return py_trees.common.Status.SUCCESS               
            elif helper.is_further_response(self.knowledge_manager.get_tag(item)):
             
                if helper.get_user_response_2_further(self.knowledge_manager.get_tag(item)) == 'yes' and not self.blackboard.get(BlackboardItems.LAST_FURTHER.value):
                    dialog = Backchannel.CONFIRM.value
                    self.knowledge_manager.add_item(UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(item), backchannel=True)
                    self._say(dialog)
                    self.blackboard.set(BlackboardItems.LAST_FURTHER.value, value=True, overwrite=True)
                    return py_trees.common.Status.SUCCESS
                else:
                    dialog = const.GET_USER_FEEDBACK
                    self.knowledge_manager.add_item(UtteranceType.RATE, dialog, get_next_tag(self.knowledge_manager.get_tag(item), True), backchannel=False)
            # Todo else of elif ??
            self._say(dialog)
            return py_trees.common.Status.FAILURE
           
        else:
            dialog = Backchannel.CONFIRM.value
            self.knowledge_manager.add_item(UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(item), backchannel=True)
            self._say(dialog)
            return py_trees.common.Status.SUCCESS

        
    def _say(self, dialog):
        self.pepper.set_speech_speed(80)
        self.pepper.say(dialog)
        self.pepper.reset_speach_speed()
        time.sleep(1)
    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class ProcessUserInput(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name="Process User Input"):
        super(ProcessUserInput,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.knowledge_manager = knowledge_manager
        print("knowledge_manager : \n", knowledge_manager)
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)
        
        item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        if self.knowledge_manager.is_further_utterance(item):
            # Todo, say yes or no:
            dialog = const.YES_NO
            self.knowledge_manager.add_item(UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(item), backchannel=False)
            self._say(dialog)
            # 5 second wait for answer
            self.pepper.get_user_speech()
            time.sleep(2)
            # Todo: static code
            _user_fb = "yes"
            self.knowledge_manager.add_item(UtteranceType.USER, _user_fb, self.knowledge_manager.get_tag(item), backchannel=False)
            self.knowledge_manager.add_item(UtteranceType.ROBOT, "Your Answer is yes", self.knowledge_manager.get_tag(item), backchannel=False)
        # Todo elif final step say thank put the rate in the file + utterance 
        elif self.knowledge_manager.is_rate_utterance(item):
            # Todo: show a web
            self.pepper.tablet_show_rate()
            app = self.pepper.tablet_touch_handling()
            coordinate = self.pepper._touch_down_feedback(app)
            print(coordinate)
             # Todo rate handling 
            self.knowledge_manager.add_item(UtteranceType.USER, "4", self.knowledge_manager.get_tag(item), backchannel=False)
            self.knowledge_manager.add_item(UtteranceType.ROBOT, const.THANKS, self.knowledge_manager.get_tag(item), backchannel=False)
        else:
            dialog = "Selected Painting is " + ("" if self.blackboard.get(BlackboardItems.SELECTED_PAINTING.value) == None else self.blackboard.get(BlackboardItems.SELECTED_PAINTING.value))
            _tag = get_next_tag("")
            self.knowledge_manager.add_item(UtteranceType.INIT, self.blackboard.get(BlackboardItems.SELECTED_PAINTING.value), _tag, backchannel=False)
            self.knowledge_manager.add_item(UtteranceType.ROBOT, dialog, _tag, backchannel=False)

        print(str(self.knowledge_manager))

        return py_trees.common.Status.SUCCESS
    
    def _say(self, dialog):
        self.pepper.set_speech_speed(80)
        self.pepper.say(dialog)
        self.pepper.reset_speach_speed()


    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class ReactUserInput(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name="Process User Input"):
        super(ReactUserInput,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.knowledge_manager = knowledge_manager
        print("knowledge_manager : \n", knowledge_manager)
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)
        
        

        item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        helper = KnowledgeManagerHelper(self.knowledge_manager)

        if helper.is_further_response(self.knowledge_manager.get_tag(item)):
            if self.knowledge_manager.get_init_utterance() == const.judgment_of_cambyses_painting['name']:
                dialog = dialog = const.JUDGEMNT_OF_CAMBYSES_PAINTING_FURTHER
            elif self.knowledge_manager.get_init_utterance() == const.scream_painting['name']:
                dialog = const.SCREAM_PAINTING_FURTHER
            else:
                print("Cannot find the painting")
                return py_trees.common.Status.FAILURE
            
        else:
            if self.knowledge_manager.get_init_utterance() == const.judgment_of_cambyses_painting['name']:
                dialog = const.JUDGEMNT_OF_CAMBYSES_PAINTING_DESCRIBTION
            elif self.knowledge_manager.get_init_utterance() == const.scream_painting['name']:
                dialog = const.SCREAM_PAINTING_DESCRIBTION
            else:
                print("Cannot find the painting")
                return py_trees.common.Status.FAILURE

        self.knowledge_manager.add_item(UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(item), backchannel=False)
        print(str(self.knowledge_manager))


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
        #time.sleep(.5)

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class EnsurePositiveUnderstanding(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name="Ensure Positive Understanding"):
        super(EnsurePositiveUnderstanding,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.knowledge_manager = knowledge_manager
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)
        
       

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)



class GivieAttentionEvidance(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name="Give Evidence of Attention, etc"):
        super(GivieAttentionEvidance,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.knowledge_manager = knowledge_manager
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)
        

        item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        self.pepper.set_speech_speed(75)
        self.pepper.say(KnowledgeManager.get_item_utterance(item))
        self.pepper.reset_speach_speed()

        self.blackboard.set(BlackboardItems.ROBOT_IS_SPEAKING.value, value=True, overwrite=True )
        time.sleep(2)

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class NoOneInitiative(py_trees.behaviour.Behaviour):

    def __init__(self, callback, pepper, knowledge_manager, name="No One has Initiative"):
        super(NoOneInitiative,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.knowledge_manager = knowledge_manager
        self.callback = callable
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)   

        self.callback(self.knowledge_manager.get_list())

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class TryOtherLine(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name="No One has Initiative"):
        super(TryOtherLine,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.knowledge_manager = knowledge_manager
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)   

        if self.blackboard.get(BlackboardItems.LAST_FURTHER.value):
                return py_trees.common.Status.SUCCESS

        item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        dialog = const.FURTHER_INFORMATION
        self.knowledge_manager.add_item(UtteranceType.FURTHER, dialog, get_next_tag(self.knowledge_manager.get_tag(item)), backchannel=False)
        self._say(dialog)

        return py_trees.common.Status.SUCCESS

    def _say(self, dialog):
        self.pepper.set_speech_speed(80)
        self.pepper.say(dialog)
        self.pepper.reset_speach_speed()
    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)

        