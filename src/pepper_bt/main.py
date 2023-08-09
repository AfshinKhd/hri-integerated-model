#!/usr/bin/env python

import py_trees
import time
#from pepper_bt.topics import Speaking

import pepper_bt.constant as const
from pepper_bt.conditions import *
from pepper_bt.actions import *
from pepper_bt.services import *
import pepper_bt.configs as cfg
from statemachine import StateMachine, State
from pepper_bt.services import Pepper
from pepper_bt.knowledge_manager import KnowledgeManager, UtteranceType

import py_trees
import rospy
from std_msgs.msg import String




class Pepper_Run():
    def __init__(self):
        self.pepper = Pepper(cfg.IP_ADDRESS, cfg.PORT)
        self.knowledge_manager = KnowledgeManager()
        self.behaviour_tree = self.PepperBTControl(self.pepper, self.knowledge_manager)

        self._stop_bt = False

        print("Pepper Starts Detection...")
        self.run()
        print("\nPepper Stop Presentation!\n")
        
        

    def on_presentation_is_finished(self, data_speech):
        print("on pressentation")
        self._stop_bt = True  
        save_speech_data(data_speech)  

    def run(self):
        for _unused_i in range(0, 10) :
            if self._stop_bt:
                print('break')
                break
            try:
                #py_trees.console.read_single_keypress()
                # behaviour_tree.tick()
                print("START BEHAVIOUR TREE...")
            # if fsm_control.is_detected_active():
                self.behaviour_tree.tick()
                
                time.sleep(1)
            except KeyboardInterrupt:
                break
        self.pepper.tablet_hide_web()
        self.pepper.stand()


    def PepperBTControl(self, pepper, knowledge_manager):
    
        root = self.create_tree(pepper, knowledge_manager)
        py_trees.logging.level = py_trees.logging.Level.DEBUG

        print(py_trees.display.print_ascii_tree(root))
        # Visualized the behavior tree - path:./catkin_ws
        print(py_trees.display.render_dot_tree(root))

        behaviour_tree = py_trees.trees.BehaviourTree(root)
        behaviour_tree.add_pre_tick_handler(self.pre_tick_handler)
        behaviour_tree.setup(timeout=15)

        return behaviour_tree
    

    def create_tree(self, pepper, knowledge_manager) :

        root = py_trees.composites.Sequence(name="Present Poster")

        establish_enagagment = py_trees.composites.Selector(name="Stablish Engagment")
        # Conditon
        user_engaged = UserEngaged("User Enagaged")
        # Action
        engage_user = EngageUser(pepper, "Engage User")
        #engage_user = py_trees.behaviours.Success("Enagage User")
        #engage_user = py_trees.behaviours.Running(name="Running")
        establish_enagagment.add_children([user_engaged,engage_user])


        interact_with_user = py_trees.composites.Selector(name="Interact with User")

        user_initiative = py_trees.composites.Sequence(name="User's Initiative")
        robot_initiative = py_trees.composites.Sequence(name="Robot's Initiative")
        no_one_initiative = NoOneInitiative(self.on_presentation_is_finished, pepper, knowledge_manager, "No One has Initiative")
        interact_with_user.add_children([user_initiative,robot_initiative,no_one_initiative])
    
        # Todo: I think dummy ?
        #user_is_speaking = py_trees.behaviours.Success("User is Speaking")
        # Condition
        user_turn = UserTurn(pepper, knowledge_manager, "User is Allowed Turn")
        process_user_input = ProcessUserInput(pepper, knowledge_manager, "Process User Input")
        attention_evidence = GivieAttentionEvidance(pepper, knowledge_manager, "Give Evidence of Attention, etc")
        user_initiative.add_children([user_turn,process_user_input,attention_evidence])

        robot_turn = py_trees.composites.Selector(name="Robot Has Turn")
        execute_presentation = py_trees.composites.Sequence(name="Execute Presentation")
        robot_initiative.add_children([robot_turn,execute_presentation])
    
        # Condition
        robot_is_speaking = py_trees.behaviours.Failure("Robot is Speaking")
        # Action
        robot_takes_turn = RobotTakesTurn(pepper, knowledge_manager, "Robot Takes Turn")
        robot_turn.add_children([robot_is_speaking, robot_takes_turn])

        # Todo : ????
        react_user_input = ReactUserInput(pepper, knowledge_manager, "React to User Input")
        # #react_user_input = py_trees.composites.Selector(name="React to User Input")
        ensure_joint_attention = py_trees.composites.Sequence(name="Ensure Joint Attention")
        deliver_presentation = py_trees.composites.Sequence(name="Deliver Presentation")
        execute_presentation.add_children([react_user_input, ensure_joint_attention, deliver_presentation])

        ensure_user_attention = EnsureUserAttention(pepper, "Ensure User Atttention")
        ensure_joint_attention.add_child(ensure_user_attention)

        # Action
        robot_starts_speaking = RobotStartsSpeaking(pepper, knowledge_manager, "Robot Starts Speaking")
        try_other_line = TryOtherLine(pepper, knowledge_manager, "Try Other Line")
        #ensure_positive_understanding = EnsurePositiveUnderstanding(pepper,knowledge_manager, "Ensure Positive Understanding")
        deliver_presentation.add_children([robot_starts_speaking, try_other_line])


        root.add_child(establish_enagagment)
        root.add_child(interact_with_user)

        return root


    def pre_tick_handler(self, behaviour_tree) :
        print("\n--------- Run %s ---------\n" % behaviour_tree.count)






    

def main():
    Pepper_Run()
    
    # pepper_detect_control = PepperDetectionControl()
    # PepperPresentationControl()
    # pepper_detect_control.cycle()
    # print(pepper_detect_control.current_state)
    # print(pepper_detect_control.send('cycle'))
    
    #fsm_control = PepperFSMControl()


    

    




class PepperFSMControl():
    def __init__(self):
        # self.behaviour_tree = behaviour_tree
        self.current_state = 'idle'
        # self.presentation_permission = True
        self.pepper = Pepper(cfg.IP_ADDRESS, cfg.PORT)
        self.human_greeter = self.pepper.start_recognizing_people()
        self.human_greeter.set_callback(self.callback_method)
        #ensuring push only one cycle event
        self.face_is_detected = False
        self.on_enter_idle()

    def cycle(self):
        if self.current_state == 'idle':
            self.current_state = 'detect'
            self.on_exit_idle()
            self.on_enter_detected()

        elif self.current_state == 'detect':
            self.current_state = 'idle'
            self.on_exit_detected()
            self.on_enter_idle

    def on_enter_idle(self):
        print("====enter idle STATE") 
        self.human_greeter.run()   
        

    def on_exit_idle(self):
        print("=====exit idle STATE")
        self.human_greeter.stop()

    def on_enter_detected(self):
        print("====enter people detected STATE")
        # print("Pepper starts Presentation...")
        # if self.presentation_permission:
        #     self.behaviour_tree.tick()
        #     self.presentation_permission = False
        # self.say()
     

    def on_exit_detected(self):
        print("====exit People detected STATE")

    def is_idle_active(self):
        return self.current_state == 'idle'
    
    def is_detected_active(self):
        return self.current_state == 'detect'

    def callback_method(self, value):
        # Do something with the received value
        print("Received value:", value)
        print("face detected value is : ", self.face_is_detected)
        if not self.face_is_detected:
            self.face_is_detected = True
            self.cycle()



class PepperDetectionControl(StateMachine):
    idle = State("Idle",initial=True)
    people_detected = State("Person Detected")

    cycle = idle.to(people_detected) | people_detected.to(idle)
    people_recognised = idle.to(people_detected)
    people_disappeared = people_detected.to(idle)

    def __init__(self):
        self.pepper = Pepper(cfg.IP_ADDRESS, cfg.PORT)
        self.human_greeter = self.pepper.start_recognizing_people()
        self.human_greeter.set_callback(self.callback_method)
        self.face_detected = False
        super(PepperDetectionControl, self).__init__()

    def before_cycle(self, event_data=None):
        message = event_data.kwargs.get("message", "")
        message = ". " + message if message else ""
        return "Running {} from {} to {}{}".format(
            event_data.event,
            event_data.transition.source.id,
            event_data.transition.target.id,
            message,
        )
    

  
        
    def on_enter_idle(self):
        print("====enter idle STATE")    
        self.human_greeter.run()

    def on_exit_idle(self):
        print("=====exit idle STATE")
        self.human_greeter.stop()

    def on_enter_people_detected(self):
        print("====enter people detected STATE")
        # PepperPresentationControl()
        behaviour_tree = Pepper_Run.PepperBTControl()
        behaviour_tree.tick()


    def on_exit_people_detected(self):
        print("^^^====exit People detected STATE")

    def callback_method(self, value):
        # Do something with the received value
        print("Received value:", value)
        if not self.face_detected:
            self.send('cycle')
            self.face_detected = True
        

    def say(self):
        print("heere")
        rospy.init_node('pepper_speaker')
        publisher = rospy.Publisher('/speech', String, queue_size=10)
        rospy.sleep(2.0)
        publisher.publish("this is test")