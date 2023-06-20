#!/usr/bin/env python

import py_trees
from pepper_bt.services import Pepper
import pepper_bt.configs as cfg


class ShowPainting(py_trees.behaviour.Behaviour):

    def __init__(self,imageUrl):
        super(ShowPainting,self).__init__(name = "Show Painting on Tablet")
        self.img_url = imageUrl
        self.logger.debug("[%s::__init__()]" % self.__class__.__name__)
        self.show_painting = py_trees.Blackboard()
        self.show_painting.set("tablet_show_paint", value=False, overwrite=True)
        self.pepper_robot = Pepper(cfg.IP_ADDRESS, cfg.PORT)
 
    def update(self) :
        self.logger.debug("[%s::update]" % self.__class__.__name__)
        self.pepper_robot.tablet_show_image(self.img_url)
        self.show_painting.set("tablet_show_paint",value=True,overwrite=True)
        return py_trees.common.Status.SUCCESS