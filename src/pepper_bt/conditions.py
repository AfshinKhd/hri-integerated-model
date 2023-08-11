#!/usr/bin/env python

import typing
import py_trees
from py_trees import common
from pepper_bt.util import BlackboardItems
from knowledge_manager import KnowledgeManagerHelper


class UserEngaged(py_trees.behaviour.Behaviour):

    def __init__(self , name = "User Engaged"):
        super(UserEngaged,self).__init__(name = name)
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.blackboard = py_trees.Blackboard()


    def update(self) :
        self.logger.debug("[%s::update()] - user engaged: %r - painting: %s" % (self.__class__.__name__ , self.blackboard.get(BlackboardItems.USER_ENGAGED.value) ,self.blackboard.get(BlackboardItems.SELECTED_PAINTING.value)))
        if self.blackboard.get(BlackboardItems.USER_ENGAGED.value):
            if self.blackboard.get(BlackboardItems.SELECTED_PAINTING.value) != None:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.FAILURE
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Listening::terminate()]" % self.__class__.__name__)


class UserTurn(py_trees.behaviour.Behaviour):

    def __init__(self, pepper, knowledge_manager, name="User is Allowed Turn"):
        super(UserTurn,self).__init__(name = name)
        self.logger.debug("  %s [Listening::__init__()]" % self.__class__.__name__)
        self.knowledge_manager = knowledge_manager
        
        self.blackboard = py_trees.Blackboard()

    def initialise(self):
        self.logger.debug("  %s [Listening::initialise()]" % self.__class__.__name__)

    def update(self):
        self.logger.debug("  %s [Listening::update()]" % self.__class__.__name__)
        
        top_stack_item = self.knowledge_manager.pop(self.knowledge_manager._generator_list())
        helper = KnowledgeManagerHelper(self.knowledge_manager)

        if self.knowledge_manager.is_init_state(top_stack_item):
            return py_trees.common.Status.SUCCESS
        elif self.knowledge_manager.is_further_utterance(top_stack_item):
            return py_trees.common.Status.SUCCESS
        elif helper.is_rate_response(self.knowledge_manager.get_tag(top_stack_item)):
            if self.knowledge_manager.is_finish_state(top_stack_item):
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        self.logger.debug("  %s [Listening::terminate()]" % self.__class__.__name__)
    

 

