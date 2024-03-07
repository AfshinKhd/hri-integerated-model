#!/usr/bin/env python
from enum import Enum
from pepper_bt.util import *

class UtteranceType(Enum):
    INIT = "init"
    ROBOT = "robot"
    FURTHER = "further"
    USER = "user"
    RATE = "rate"
    FINISH = "finish" #finishing presentation

class Utterance(Enum):
    STATE = 'state'
    SPEAKER = 'speaker'
    BACKCHANNEL = 'backchannel'
    UTTERANCE = 'utterance'
    TAG = 'tag'
    TIME = 'time'


class KnowledgeManager():

    def __init__(self):
        self._list = []

    def _add(self, item):
        self._list.append(item)

    def add_item(self, state, speaker, dialog, tag, _time, backchannel=False):
        self._add({Utterance.STATE.value: state.value, Utterance.SPEAKER.value: "" if speaker is None else speaker.value, Utterance.BACKCHANNEL.value: backchannel, 
                   Utterance.UTTERANCE.value: dialog, Utterance.TAG.value: tag, Utterance.TIME.value: _time})
    
    def get_list(self):
        return self._list
    
    @staticmethod
    def get_item_utterance(item):
        if item == None:
            return ""
        return item[Utterance.UTTERANCE.value]
    
    @staticmethod
    def get_tag(item):
        if item == None:
            return ""
        return item[Utterance.TAG.value]
    
    @staticmethod
    def is_robot_utterance(item):
        if item == None:
            return False
        return item[Utterance.SPEAKER.value] == UtteranceType.ROBOT.value
    
    @staticmethod
    def is_user_utterance(item):
        if item == None:
            return False
        return item[Utterance.STATE.value] == UtteranceType.USER.value
    
    @staticmethod
    def is_further_state(item):
        if item == None:
            return False
        return item[Utterance.STATE.value] == UtteranceType.FURTHER.value
    
    @staticmethod
    def is_rate_state(item):
        if item == None:
            return False
        return item[Utterance.STATE.value] == UtteranceType.RATE.value
    
    @staticmethod
    def is_finish_state(item):
        if item == None:
            return False
        return item[Utterance.STATE.value] == UtteranceType.FINISH.value
    
    @staticmethod
    def is_init_state(item):
        if item == None:
            return False
        return item[Utterance.STATE.value] == UtteranceType.INIT.value
     
    def _generator_list(self):
        for item in self._list[::-1]:
            yield item

    @staticmethod
    def pop(generated_list):
        try:
            return next(generated_list)
        except StopIteration as e:
            return None
        
    def __len__(self):
        return len(self._list)

    def __str__(self) :
        return str(self._list)

class KnowledgeManagerHelper():

    def __init__(self, knowledge_manager) :
        self.knowledge_manager = knowledge_manager


    def is_initial(self):
        if len(self.knowledge_manager) == 0:
            return True
        if len(self.knowledge_manager) == 1:
            generated_list = self.knowledge_manager._generator_list()
            if KnowledgeManager.pop(generated_list)['backchannel']:
                return True
        return False
    

    def is_introduce(self):
        if len(self.knowledge_manager) > 1:
            generated_list = self.knowledge_manager._generator_list()
            if KnowledgeManager.pop(generated_list)['state'] == UtteranceType.ROBOT.value:
                item = KnowledgeManager.pop(generated_list)
                if item['state'] == UtteranceType.INIT.value:
                    return True, item['utterance']
        return False, ""
    

    def is_init_response(self, top_stack):
        return self.knowledge_manager.is_init_state(top_stack)


    def is_further_response(self, top_stack):
        return self.knowledge_manager.is_further_state(top_stack)
    

    def is_finish_response(self, top_stack):
        return self.knowledge_manager.is_finish_state(top_stack)
    

    def is_rate_response(self, top_stack):
        return self.knowledge_manager.is_rate_state(top_stack)

    
    def get_user_response_2_further(self, tag):
        for item in self.knowledge_manager.get_list():
            if item.get(Utterance.TAG.value) == tag and item.get(Utterance.SPEAKER.value) == UtteranceType.USER.value:
                return item.get(Utterance.UTTERANCE.value)
        return ""
    
    def get_selected_painting(self, tag):
        for item in self.knowledge_manager.get_list():
            if item.get(Utterance.TAG.value)[0] == tag[0] and item.get(Utterance.STATE.value) == UtteranceType.INIT.value:
                return item.get(Utterance.UTTERANCE.value)
        return ""
    
    def get_backchannel_utterance(self, tag):
        for item in self.knowledge_manager.get_list():
            if item.get(Utterance.TAG.value) == tag and item.get(Utterance.SPEAKER.value) == UtteranceType.ROBOT.value and item.get(Utterance.BACKCHANNEL.value):
                return item.get(Utterance.UTTERANCE.value)
        return ""