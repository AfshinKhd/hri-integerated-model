#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from pepper_bt.services import Pepper
import configs as cfg
import qi


class play_annimation():

    def __init__(self):
        rospy.init_node('pepper_annimation', anonymous=True)
        self.subscriber = rospy.Subscriber("/pepper_annim", String, self.callback)
        self.session = qi.Session()
        self.session.connect("tcp://" + "10.0.0.3" + ":" + "9559")
        self.animation_service = self.session.service("ALAnimationPlayer")
        #self.rate = rospy.Rate(10) # 10hz
        
            
    def listen(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        print("xxxxxxxxx")
        animation_finished = self.animation_service.run("animations/Stand/Gestures/" + "Explain_11", _async=True)
        animation_finished.value()
        # self.pepper = Pepper("10.0.0.3", "9559")
        # self.pepper.start_animation("Explain_11")
        # self.pepper.start_animation("Explain_11")
        

def launch_annimation():
    try:
        tp = play_annimation()
        tp.listen()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    try:
        tp = play_annimation()
        tp.listen()
    except rospy.ROSInterruptException:
        pass