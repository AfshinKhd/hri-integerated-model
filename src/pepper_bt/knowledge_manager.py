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


class KnowledgeManager():

    def __init__(self):
        self._list = []

    def _add(self, item):
        self._list.append(item)

    def add_item(self, state, dialog, tag, backchannel=False):
        self._add({'state': state.value, 'backchannel': backchannel, 'utterance': dialog, 'tag':tag})
    
    def get_list(self):
        return self._list
    
    def get_init_utterance(self):
        for item in self._list:
            if item.get("state") == UtteranceType.INIT.value:
                return item.get("utterance")
        return ""

    @staticmethod
    def get_item_utterance(item):
        if item == None:
            return ""
        return item['utterance']
    
    @staticmethod
    def get_tag(item):
        if item == None:
            return ""
        return item['tag']
    
 
    
    @staticmethod
    def is_robot_utterance(item):
        if item == None:
            return False
        return item['state'] == UtteranceType.ROBOT.value
    
    @staticmethod
    def is_user_utterance(item):
        if item == None:
            return False
        return item['state'] == UtteranceType.USER.value
    
    @staticmethod
    def is_further_utterance(item):
        if item == None:
            return False
        return item['state'] == UtteranceType.FURTHER.value
    
    @staticmethod
    def is_rate_utterance(item):
        if item == None:
            return False
        return item['state'] == UtteranceType.RATE.value
    
    @staticmethod
    def is_finish_state(item):
        if item == None:
            return False
        return item['state'] == UtteranceType.FINISH.value
     
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
                print("here1")
                item = KnowledgeManager.pop(generated_list)
                print("here2")
                if item['state'] == UtteranceType.INIT.value:
                    print("here3")
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
    
    def is_rate_response(self):
        for item in self.knowledge_manager.get_list():
            if item.get('state') == UtteranceType.RATE.value:
                return True
        return False
    
    def get_user_response_2_further(self, tag):
        for item in self.knowledge_manager.get_list():
            if item.get('tag') == tag and item.get('state') == UtteranceType.USER.value:
                return item.get('utterance')
        return ""
    
