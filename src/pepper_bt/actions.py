#!/usr/bin/env python

import py_trees
from pepper_bt.services import Pepper
import pepper_bt.configs as cfg
from pepper_bt.util import *
import pepper_bt.constant as const
from pepper_bt.topics import Speaking as SyncSpeak
import pepper_bt.topics as topics
from pepper_bt.knowledge_manager import KnowledgeManager, UtteranceType,KnowledgeManagerHelper
import time
 
    
class RobotStartsSpeaking(py_trees.behaviour.Behaviour):
    """
        Initializes a custom PyTorch dataset for dialogue processing.

        Args:
            speakers (pd.core.series.Series): Series containing speaker information.
            dialogues (pd.core.series.Series): Series containing dialogue utterances.
            emotions (pd.core.series.Series): Series containing emotion information.
            triggers (pd.core.series.Series): Series containing trigger information.
            device: PyTorch device to store the tensors.
            pad_token (str): Token to use for padding dialogue sequences.
            max_num_utterances (int): Maximum number of utterances to consider in a dialogue.

        Returns:
            tuple: A tuple containing tensors for speakers, dialogue IDs, dialogue masks, emotions, and triggers.

        Note:
            - The dataset preprocesses dialogues, speakers, emotions, and triggers with padding.
            - For each dialogue, a sequence of padding tokens is added to match the length of the longest dialogue in the dataset.
            - For each dialogue, dialogue ids and dialogue mask are created using the tokenizer.
            - For each speakers instance, a sequence of zeros is appended to align with the maximum number of utterances in any dialogue.
            - For each emotions instance, a sequence of arrays of zeros is appended to align with the maximum number of utterances in any dialogue.
            - For each triggers instance, a sequence of zeros is appended to align with the maximum number of utterances in any dialogue.
    """

    def __init__(self, pepper,knowledge_manager, name):
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
        self.pepper.present_gesture()
        # utterance
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

    def __init__(self, pepper, knowledge_manager, name):
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

        self.pepper.present_gesture(False)
        
        if self.pepper.tablet_show_web():           
            # Waiting for action of user
            paintings = const.PAINTINGS # Give back a dictionary with information about paints
            coordinate = self.pepper._touch_down_feedback(lower_x=150, upper_x=1680, lower_y=250 , upper_y=1020) # params set for kernel which occured the paints

            print("night: ")
            print(paintings['nighthawks'].lower_x)

            if coordinate['x'] < paintings['nighthawks'].upper_x :
                selected_painting = 'nighthawks'
            elif coordinate['x'] < paintings['mona_lisa'].upper_x  :
                selected_painting = 'mona_lisa'
            elif coordinate['x'] < paintings['starry_night'].upper_x  :
                selected_painting = 'starry_night'
            elif coordinate['x'] < paintings['the_persistence_of_memory'].upper_x  :
                selected_painting = 'the_persistence_of_memory'
            else:
                print("[ERROR]] coordinate is wrong!")

            print("selected paint is:",selected_painting)

            if selected_painting is None :
                print("[ERROR] There is no Painting")
                return py_trees.common.Status.FAILURE
            else:
                self.blackboard.set(BlackboardItems.USER_ENGAGED.value, value=True, overwrite=True)
                self.blackboard.set(BlackboardItems.SELECTED_PAINTING.value, selected_painting, overwrite=True)
                self.blackboard.set(BlackboardItems.LAST_FURTHER.value, value=False, overwrite=True)

                top_stack_item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
                self.knowledge_manager.add_item(UtteranceType.INIT, None, selected_painting, get_next_tag(self.knowledge_manager.get_tag(top_stack_item), True), topics.get_current_time(), backchannel=False)
                return py_trees.common.Status.SUCCESS
        else:
            print("[ERROR] Something goes wrong! (Engage User Action)")
            return py_trees.common.Status.FAILURE
        
    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class RobotTakesTurn(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name):
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
        
        if helper.is_init_response(top_stack_item):
            dialog = Backchannel.CONFIRM.value
            self.knowledge_manager.add_item(UtteranceType.INIT, UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(top_stack_item),topics.get_current_time() ,backchannel=True)
            return py_trees.common.Status.SUCCESS     
                  
        elif helper.is_further_response(top_stack_item):
            if helper.get_user_response_2_further(self.knowledge_manager.get_tag(top_stack_item)) == 'yes' and not self.blackboard.get(BlackboardItems.LAST_FURTHER.value):
                dialog = Backchannel.CONFIRM.value
                self.knowledge_manager.add_item(UtteranceType.FURTHER, UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(top_stack_item), topics.get_current_time(), backchannel=True)
                self.blackboard.set(BlackboardItems.LAST_FURTHER.value, value=True, overwrite=True)
                return py_trees.common.Status.SUCCESS
            else:
                _tag = get_next_tag(self.knowledge_manager.get_tag(top_stack_item))
                self.knowledge_manager.add_item(UtteranceType.RATE, None, "", _tag, topics.get_current_time(), backchannel=False)
                return py_trees.common.Status.FAILURE

        elif helper.is_rate_response(top_stack_item):
            return py_trees.common.Status.FAILURE
        
        elif helper.is_finish_response(top_stack_item):
            return py_trees.common.Status.FAILURE
        
        else:
            print("Something goes wrong! (RobotTakesTurn Action)")
            return py_trees.common.Status.FAILURE 
         

    def _say(self, dialog):
        self.pepper.set_speech_speed(cfg.VOICE_SPEED_LOW)
        self.pepper.say(dialog)
        self.pepper.reset_speach_speed()
        time.sleep(1)
    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class ProcessUserInput(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name):
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

        if self.knowledge_manager.is_further_state(top_stack_item):
            dialog = const.FURTHER_INFORMATION
            dialog += const.YES_NO
            self.knowledge_manager.add_item(UtteranceType.FURTHER, UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(top_stack_item), topics.get_current_time(),backchannel=False)
            self._say(dialog)
            # Default is "NO" 
            _user_response = self.pepper.listen_to_user()
            self.knowledge_manager.add_item(UtteranceType.FURTHER, UtteranceType.USER, _user_response, self.knowledge_manager.get_tag(top_stack_item), topics.get_current_time(), backchannel=False)
            self.knowledge_manager.add_item(UtteranceType.FURTHER, UtteranceType.ROBOT, "Your Answer is " + _user_response, self.knowledge_manager.get_tag(top_stack_item), topics.get_current_time(), backchannel=False)

        elif helper.is_rate_response(top_stack_item):
            # Ask user for rate
            dialog = const.GET_USER_FEEDBACK
            self.knowledge_manager.add_item(UtteranceType.RATE, UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(top_stack_item), topics.get_current_time(), backchannel=False)
            self._say(dialog)
            # Show the rate html
            self.pepper.tablet_show_rate()
            coordinate = self.pepper._touch_down_feedback(lower_x=160, upper_x=1590, lower_y=370 , upper_y=680)
            _rate = self.get_user_rate(coordinate)
            self.pepper.tablet_hide_web()

            self.knowledge_manager.add_item(UtteranceType.RATE, UtteranceType.USER, _rate, self.knowledge_manager.get_tag(top_stack_item), topics.get_current_time(), backchannel=False)
            self.knowledge_manager.add_item(UtteranceType.RATE, UtteranceType.ROBOT, const.THANKS, self.knowledge_manager.get_tag(top_stack_item), topics.get_current_time(), backchannel=False)

        else:
            current_time = topics.get_current_time()
            paintings = const.PAINTINGS  
            selected_painting = "" if self.blackboard.get(BlackboardItems.SELECTED_PAINTING.value) == None else paintings[self.blackboard.get(BlackboardItems.SELECTED_PAINTING.value)].name               
            dialog = "Selected Painting is " + selected_painting
            self.knowledge_manager.add_item(UtteranceType.INIT, UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(top_stack_item), current_time, backchannel=False)


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
        self.pepper.set_speech_speed(cfg.VOICE_SPEED_LOW)
        self.pepper.say(dialog)
        self.pepper.reset_speach_speed()


    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class ReactUserInput(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name):
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
        paintings = const.PAINTINGS
        top_stack_item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        helper = KnowledgeManagerHelper(self.knowledge_manager)
        selected_painting = helper.get_selected_painting(self.knowledge_manager.get_tag(top_stack_item))
        print(str(self.knowledge_manager))

        if helper.is_further_response(top_stack_item):
            _state = UtteranceType.FURTHER
            if selected_painting != "":
                dialog = paintings[selected_painting].further_info
            else:
                print("Cannot find the painting")
                return py_trees.common.Status.FAILURE
            
        else:
            _state = UtteranceType.INIT
            if selected_painting != "":
                dialog = paintings[selected_painting].describtion
            else:
                print("Cannot find the painting")
                return py_trees.common.Status.FAILURE

        self.knowledge_manager.add_item(_state, UtteranceType.ROBOT, dialog, self.knowledge_manager.get_tag(top_stack_item), topics.get_current_time(),backchannel=False)
        

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class EnsureUserAttentionbyGesture(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name):
        super(EnsureUserAttentionbyGesture,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)

        self.pepper.start_animation("Explain_1")

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class EnsureUserAttentionbyVisual(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name):
        super(EnsureUserAttentionbyVisual,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.pepper = pepper
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("[%s::initialise()]" % self.__class__.__name__)


    def update(self):
        self.logger.debug("[%s::update()]" % self.__class__.__name__)
        
        self.pepper.fade_ears()

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class EnsureUserAttentionbyVerbal(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name):
        super(EnsureUserAttentionbyVerbal,self).__init__(name = name)
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
        _dialog = helper.get_backchannel_utterance(self.knowledge_manager.get_tag(top_stack_item))
        self._say(_dialog)
        time.sleep(1)

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)

    def _say(self, dialog):
        self.pepper.set_speech_speed(cfg.VOICE_SPEED_LOW)
        self.pepper.say(dialog)
        self.pepper.reset_speach_speed()
        


class EnsurePositiveUnderstanding(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name):
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

        



class GiveUnderstandingEvidance(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name):
        super(GiveUnderstandingEvidance,self).__init__(name = name)
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

        self.blackboard.set(BlackboardItems.ROBOT_IS_SPEAKING.value, value=True, overwrite=True)
        time.sleep(2)

        if helper.is_rate_response(top_stack_item):
            self.knowledge_manager.add_item(UtteranceType.FINISH, None, "", self.knowledge_manager.get_tag(top_stack_item), topics.get_current_time(), backchannel=False)

        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class NoOneInitiative(py_trees.behaviour.Behaviour):

    def __init__(self, callback, pepper, knowledge_manager, name):
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

        if helper.is_rate_response(top_stack_item):
            return py_trees.common.Status.FAILURE
        
        elif helper.is_finish_response(top_stack_item):
            self.blackboard.set(BlackboardItems.USER_ENGAGED.value, value=False, overwrite=True)
            self.callback(self.knowledge_manager.get_list())
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)


class TryOtherLine(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name):
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
        _tag = get_next_tag(self.knowledge_manager.get_tag(top_stack_item))
        self.knowledge_manager.add_item(UtteranceType.FURTHER, None, "", _tag, topics.get_current_time(),backchannel=False)

        return py_trees.common.Status.SUCCESS

    def _say(self, dialog):
        self.pepper.set_speech_speed(cfg.VOICE_SPEED_LOW)
        self.pepper.say(dialog)
        self.pepper.reset_speach_speed()
    
    def terminate(self, new_status):
        self.logger.debug("[%s::terminate()]" % self.__class__.__name__)

        