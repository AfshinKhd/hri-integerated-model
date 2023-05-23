#!/usr/bin/env python

import py_trees


class PermitRobotSpeak(py_trees.behaviour.Behaviour):

    def __init__(self):
        super(PermitRobotSpeak,self).__init__(name = "Permit Robot Speak")
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.state = py_trees.Blackboard()
        self.state.set("speaking_is_running",value=False,overwrite=True)
 
    def update(self) :
        self.logger.debug("[%s::update()] - speaking_is_running: %r" % (self.__class__.__name__ , self.state.get("speaking_is_running")))
        if self.state.get("speaking_is_running") == True:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


