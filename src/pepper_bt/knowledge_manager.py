#!/usr/bin/env python
from enum import Enum
from pepper_bt.util import *

class UtteranceType(Enum):
    INIT = "init"
    ROBOT = "robot"
    FURTHER = "further"
    USER = "user"
    RATE = "rate"
    FINISH = "finish" #finishing bt

class Utterance(Enum):
    STATE = 'state'
    BACKCHANNEL = 'backchannel'
    UTTERANCE = 'utterance'
    TAG = 'tag'


class KnowledgeManager():

    def __init__(self):
        self._list = []

    def _add(self, item):
        self._list.append(item)

    def add_item(self, state, dialog, tag, backchannel=False):
        self._add({Utterance.STATE.value: state.value, Utterance.BACKCHANNEL.value: backchannel, Utterance.UTTERANCE.value: dialog, Utterance.TAG.value:tag})
    
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
        return item[Utterance.STATE.value] == UtteranceType.ROBOT.value
    
    @staticmethod
    def is_user_utterance(item):
        if item == None:
            return False
        return item[Utterance.STATE.value] == UtteranceType.USER.value
    
    @staticmethod
    def is_further_utterance(item):
        if item == None:
            return False
        return item[Utterance.STATE.value] == UtteranceType.FURTHER.value
    
    @staticmethod
    def is_rate_utterance(item):
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
        #self.top_stack_item = knowledge_manager.knowledge_manager.pop(self.knowledge_manager._generator_list())

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
    
    def is_init_response(self, tag):
        for item in self.knowledge_manager.get_list():
            if item.get('tag') == tag and item.get('state') == UtteranceType.INIT.value:
                return True
        return False
    
    def is_further_response(self, tag):
        for item in self.knowledge_manager.get_list():
            if item.get('tag') == tag and item.get('state') == UtteranceType.FURTHER.value:
                return True
        return False
    
    def is_rate_response(self, tag):
        for item in self.knowledge_manager.get_list():
            if item.get('tag') == tag and item.get('state') == UtteranceType.RATE.value:
                return True
        return False
    
    def get_user_response_2_further(self, tag):
        for item in self.knowledge_manager.get_list():
            if item.get('tag') == tag and item.get('state') == UtteranceType.USER.value:
                return item.get('utterance')
        return ""
    
    def get_selected_painting(self, tag):
        for item in self.knowledge_manager.get_list():
            if item.get('tag')[0] == tag[0] and item.get('state') == UtteranceType.INIT.value:
                return item.get('utterance')
        return ""
    
