#!/usr/bin/env python

import py_trees
from pepper_bt.services import Pepper
import pepper_bt.configs as cfg
from pepper_bt.util import *
import pepper_bt.constant as const
from pepper_bt.topics import Speaking as SyncSpeak
from pepper_bt.knowledge_manager import KnowledgeManager, UtteranceType,KnowledgeManagerHelper
import time
 
    
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
        top_stack_item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        self.blackboard.set(BlackboardItems.SPEAKING_IS_RUNNING.value,True,overwrite=True)
        self.pepper.set_speech_speed(95)
        self.pepper.say(KnowledgeManager.get_item_utterance(top_stack_item))
        self.pepper.reset_speach_speed()
        self.blackboard.set(BlackboardItems.SPEAKING_IS_RUNNING.value,False,overwrite=True)  
        
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class EngageUser(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name="Engage User"):
        super(EngageUser,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.knowledge_manager = knowledge_manager
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)

        # Utterance
        utterance = SyncSpeak()
        utterance.say(const.PRESENTATION_HELLO)
        # Gesture: run annimation: Async
        self.pepper.present_gesture()
        
        if self.pepper.tablet_show_web():           
            # Waiting for action of user
            coordinate = self.pepper._touch_down_feedback(lower_x=80, upper_x=1680, lower_y=0 , upper_y=1020)

            # Todo: dynamic numbers
            if coordinate['x'] < 690 :
                selected_painting = const.judgment_of_cambyses_painting['name']
            elif coordinate['x'] > 690 :
                selected_painting = const.scream_painting['name']
            else:
                print("coordinate is wrong!")

            if selected_painting is None :
                print("Error in class %s"%self.__class__.__name__)
                return py_trees.common.Status.FAILURE
            else:
                self.blackboard.set(BlackboardItems.USER_ENGAGED.value, value=True, overwrite=True)
                self.blackboard.set(BlackboardItems.SELECTED_PAINTING.value, selected_painting, overwrite=True)
                self.blackboard.set(BlackboardItems.LAST_FURTHER.value, value=False, overwrite=True)

                top_stack_item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
                self.knowledge_manager.add_item(UtteranceType.INIT, selected_painting, get_next_tag(self.knowledge_manager.get_tag(top_stack_item), True), backchannel=False)
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
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)

        top_stack_item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        helper = KnowledgeManagerHelper(self.knowledge_manager)


        if helper.is_rate_response(self.knowledge_manager.get_tag(top_stack_item)):
            return py_trees.common.Status.FAILURE
        
        if self.knowledge_manager.is_robot_utterance(top_stack_item):
            if helper.is_init_response(self.knowledge_manager.get_tag(top_stack_item)):
                dialog = Backchannel.CONFIRM.value
                self.knowledge_manager.add_item(UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(top_stack_item), backchannel=True)
                self._say(dialog)
                return py_trees.common.Status.SUCCESS               
            elif helper.is_further_response(self.knowledge_manager.get_tag(top_stack_item)):
             
                if helper.get_user_response_2_further(self.knowledge_manager.get_tag(top_stack_item)) == 'yes' and not self.blackboard.get(BlackboardItems.LAST_FURTHER.value):
                    dialog = Backchannel.CONFIRM.value
                    self.knowledge_manager.add_item(UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(top_stack_item), backchannel=True)
                    self._say(dialog)
                    self.blackboard.set(BlackboardItems.LAST_FURTHER.value, value=True, overwrite=True)
                    return py_trees.common.Status.SUCCESS
                else:
                    dialog = const.GET_USER_FEEDBACK
                    _tag = get_next_tag(self.knowledge_manager.get_tag(top_stack_item))
                    self.knowledge_manager.add_item(UtteranceType.RATE, "", _tag, backchannel=False)
                    self.knowledge_manager.add_item(UtteranceType.ROBOT, dialog, _tag, backchannel=False)
            # Todo else of elif ??
            self._say(dialog)
            return py_trees.common.Status.FAILURE
           
        else:
            dialog = Backchannel.CONFIRM.value
            self.knowledge_manager.add_item(UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(top_stack_item), backchannel=True)
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
        
        top_stack_item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        helper = KnowledgeManagerHelper(self.knowledge_manager)

        if self.knowledge_manager.is_further_utterance(top_stack_item):
            # Todo, say yes or no:
            dialog = const.YES_NO
            self.knowledge_manager.add_item(UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(top_stack_item), backchannel=False)
            self._say(dialog)
            # 5 second wait for answer
            self.pepper.get_user_speech()
            time.sleep(2)
            # Todo: static code
            _user_fb = "yes"
            self.knowledge_manager.add_item(UtteranceType.USER, _user_fb, self.knowledge_manager.get_tag(top_stack_item), backchannel=False)
            self.knowledge_manager.add_item(UtteranceType.ROBOT, "Your Answer is yes", self.knowledge_manager.get_tag(top_stack_item), backchannel=False)
        # Todo elif final step say thank put the rate in the file + utterance 
        elif helper.is_rate_response(self.knowledge_manager.get_tag(top_stack_item)):

            self.pepper.tablet_show_rate()
            coordinate = self.pepper._touch_down_feedback(lower_x=160, upper_x=1590, lower_y=370 , upper_y=680)
            _rate = self.get_user_rate(coordinate)
            print("rate is :", _rate)
            self.pepper.tablet_hide_web()
             # Todo: rate handling function
            self.knowledge_manager.add_item(UtteranceType.USER, _rate, self.knowledge_manager.get_tag(top_stack_item), backchannel=False)
            self.knowledge_manager.add_item(UtteranceType.ROBOT, const.THANKS, self.knowledge_manager.get_tag(top_stack_item), backchannel=False)
        else:
            dialog = "Selected Painting is " + ("" if self.blackboard.get(BlackboardItems.SELECTED_PAINTING.value) == None else self.blackboard.get(BlackboardItems.SELECTED_PAINTING.value))
            self.knowledge_manager.add_item(UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(top_stack_item), backchannel=False)


        return py_trees.common.Status.SUCCESS
    
    def get_user_rate(self, coordinate):
        if 160 < coordinate['x'] < 430:
            return 1
        elif 430 < coordinate['x'] < 700:
            return 2
        elif 700 < coordinate['x'] < 970:
            return 3
        elif 970 < coordinate['x'] < 1240:
            return 4
        elif 1240 < coordinate['x'] < 1510:
            return 5
        else:
            print("something goes wrong!")
        return 0

    
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
        
        

        top_stack_item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        helper = KnowledgeManagerHelper(self.knowledge_manager)

        print(str(self.knowledge_manager))

        if helper.is_further_response(self.knowledge_manager.get_tag(top_stack_item)):
            if helper.get_selected_painting(self.knowledge_manager.get_tag(top_stack_item)) == const.judgment_of_cambyses_painting['name']:
                dialog = const.JUDGEMNT_OF_CAMBYSES_PAINTING_FURTHER
            elif helper.get_selected_painting(self.knowledge_manager.get_tag(top_stack_item)) == const.scream_painting['name']:
                dialog = const.SCREAM_PAINTING_FURTHER
            else:
                print("Cannot find the painting")
                return py_trees.common.Status.FAILURE
            
        else:
            if helper.get_selected_painting(self.knowledge_manager.get_tag(top_stack_item)) == const.judgment_of_cambyses_painting['name']:
                dialog = const.JUDGEMNT_OF_CAMBYSES_PAINTING_DESCRIBTION
            elif helper.get_selected_painting(self.knowledge_manager.get_tag(top_stack_item))  == const.scream_painting['name']:
                dialog = const.SCREAM_PAINTING_DESCRIBTION
            else:
                print("Cannot find the painting")
                return py_trees.common.Status.FAILURE

        self.knowledge_manager.add_item(UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(top_stack_item), backchannel=False)
        


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
        

        top_stack_item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        helper = KnowledgeManagerHelper(self.knowledge_manager)
        self.pepper.set_speech_speed(75)
        self.pepper.say(KnowledgeManager.get_item_utterance(top_stack_item))
        self.pepper.reset_speach_speed()



        self.blackboard.set(BlackboardItems.ROBOT_IS_SPEAKING.value, value=True, overwrite=True )
        time.sleep(2)
        if helper.is_rate_response(self.knowledge_manager.get_tag(top_stack_item)):
            self.knowledge_manager.add_item(UtteranceType.FINISH, "", self.knowledge_manager.get_tag(top_stack_item), backchannel=False)

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class NoOneInitiative(py_trees.behaviour.Behaviour):

    def __init__(self, callback, pepper, knowledge_manager, name="No One has Initiative"):
        super(NoOneInitiative,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.knowledge_manager = knowledge_manager
        self.callback = callback
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)   

        top_stack_item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        helper = KnowledgeManagerHelper(self.knowledge_manager)
        if helper.is_rate_response(self.knowledge_manager.get_tag(top_stack_item)):
            if self.knowledge_manager.is_finish_state(top_stack_item):
                self.blackboard.set(BlackboardItems.USER_ENGAGED.value, value=False, overwrite=True)
                self.callback(self.knowledge_manager.get_list())
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE
        
        

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

        top_stack_item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        dialog = const.FURTHER_INFORMATION
        self.knowledge_manager.add_item(UtteranceType.FURTHER, dialog, get_next_tag(self.knowledge_manager.get_tag(top_stack_item)), backchannel=False)
        self._say(dialog)

        return py_trees.common.Status.SUCCESS

    def _say(self, dialog):
        self.pepper.set_speech_speed(80)
        self.pepper.say(dialog)
        self.pepper.reset_speach_speed()
    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)

        