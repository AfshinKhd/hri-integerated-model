#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from pepper_bt.services import Pepper
import configs as cfg
import qi


class Play_Animation():

    def __init__(self):
    
        rospy.init_node('pepper_animation', anonymous=True)
        self.subscriber = rospy.Subscriber("/pepper_anim", String, self.callback)

        self.connection_url = "tcp://" + cfg.IP_ADDRESS + ":" + str(cfg.PORT)
        try:
            self.app = qi.Application(["Play_Animation", "--qi-url=" + self.connection_url])
        except RuntimeError:
            print ("PepperApp connection has error to connect")

        rospy.on_shutdown(self.shutdown_callback)
        self.rate = rospy.Rate(10) # 10hz

    def shutdown_callback(self):
        # This function will be called when the node is shutting down
        # Perform your cleanup or other tasks here
        rospy.loginfo("Shutting down callback...")
        self.app.stop()
        # Your cleanup code here
   
        
            
    def listen(self):
 
        """
        This method is listener for /pepper_anim topic
        """
        
        self.app.start()
        session = self.app.session
        self.animation_service = session.service("ALAnimationPlayer")
        self.app.run()
        self.rate.sleep()
        rospy.spin()


    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " Animation is running: %s", data.data)
        animation_finished = self.animation_service.run("animations/Stand/Gestures/" + "Explain_11", _async=True)
        animation_finished.value()
        

def launch_annimation():
    try:
        tp = Play_Animation()
        tp.listen()
    except rospy.ROSInterruptException:
        print("pepper_animation is shutting down! + ros")
    except KeyboardInterrupt:
        print("pepper_animation is shutting down! +  ke")
    except RuntimeError:
        print("pepper_animation is shutting down! + run")

# if __name__ == "__main__":
#     try:
#         tp = Play_Animation()
#         tp.listen()
#     except rospy.ROSInterruptException:
#         pass