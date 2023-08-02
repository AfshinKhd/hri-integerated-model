#!/usr/bin/env python
from enum import Enum

class UtteranceType(Enum):
    INIT = "init"
    ROBOT = "robot"
    USER = "user"
    FINAL = "final"


class KnowledgeManager():

    def __init__(self):
        self._list = []

    def add(self, item):
        self._list.append(item)

    def set_item(self, state, dialog, backchannel=False):
        return {'state': state.value, 'backchannel': backchannel, 'speech': dialog}
    
    def _generator_list(self):
        for item in self._list[::-1]:
            yield item

    @staticmethod
    def pop(self, generated_list):
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