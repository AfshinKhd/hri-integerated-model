#!/usr/bin/env python

import typing
import py_trees
from py_trees import common
import rospy
from std_msgs.msg import String, Bool
from pepper_bt.msg import SpeechParams
from naoqi_bridge_msgs.msg import AudioBuffer
import configs as cfg
import os
import pepper_bt.util 


class Speaking():

    def __init__(self):
        rospy.init_node('pepper_main')
        self.publisher = rospy.Publisher('/speech', String, queue_size=10)

    def say(self, message):
        rospy.sleep(2.0)
        self.publisher.publish(message)


class Listener_():

    def __init__(self):
        rospy.init_node('pepper_main', anonymous=True)
        self.subscriber = rospy.Subscriber("/naoqi_microphone/audio_raw", AudioBuffer, self.speech_cb)
        rospy.spin()

    def speech_cb(self,data):
        print("data is:",data)


class Speaking_():

    def __init__(self):

        #self.pepper = Pepper(cfg.IP_ADDRESS,cfg.PORT)
  

        rospy.init_node('chatter', anonymous=True)
        self.publisher = rospy.Publisher('/pepper_speech', SpeechParams, queue_size=10)
        self.subscriber = rospy.Subscriber("/pepper_speech", SpeechParams, self.speech_cb)
        # speech_param = SpeechParams()
        # speech_param.is_speaking = True
        # speech_param.msg = "message"
        # while not rospy.is_shutdown():
        #     self.publisher.publish(speech_param)
        #     os.sleep(1)

    def say(self, message):
        speech_param = SpeechParams()
        speech_param.is_speaking = True
        speech_param.msg = message

        self.pepper.say(message)

        self.publisher.publish(speech_param)
        print("saaay")
        rospy.spin()
        
    def speech_cb(self,data):
        print("data is:",data)

def get_current_time():
    # rospy.init_node('pepper_main', anonymous=False)
    current_time = rospy.Time.now()
    return current_time.to_sec()
    # return 10

def publisher():
    
      # Replace 'String' with the desired message type
    rate = rospy.Rate(10)  # Define the publishing rate (10 Hz in this case)

 

class Play_Animation():

    def __init__(self):
        rospy.init_node('pepper_main', anonymous=False)
        self.publisher = rospy.Publisher('/pepper_anim', String, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        
    def publish(self):
        print('play annimation')
        rospy.sleep(.1)
        self.publisher.publish("False")
        

if __name__ == "__main__":
    
    try:
        tp = Play_Animation()
        tp.publish()
    except rospy.ROSInterruptException:
        pass
    
    