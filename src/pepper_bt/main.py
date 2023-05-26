#!/usr/bin/env python

import py_trees
import time
from pepper_bt.topics import Speaking
import pepper_bt.constant as const
from pepper_bt.conditions import PermitRobotSpeak
from pepper_bt.actions import ShowPainting

def pre_tick_handler(behaviour_tree) :
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)



def create_tree() :
    root = py_trees.composites.Sequence(name="Present Poster")

    #first row - left leaves
    establish_enagagment = py_trees.composites.Selector(name="Stablish Engagment")
    user_engaged = py_trees.behaviours.Failure("User Enagaged")
    engage_user = py_trees.behaviours.Success("Enagage User")
    #engage_user = py_trees.behaviours.Running(name="Running")
    establish_enagagment.add_children([user_engaged,engage_user])

    #first row - right leaves 
    interact_with_user = py_trees.composites.Selector(name="Interact with User")
    ####second row - left leaves 
    user_initiative = py_trees.composites.Sequence(name="User's Initiative")
    ####second row - right leaves
    robot_initiative = py_trees.composites.Sequence(name="Robot's Initiative")
    no_one_initiative = py_trees.behaviours.Failure("No One has Initiative")
    interact_with_user.add_children([user_initiative,robot_initiative,no_one_initiative])

    ########third row 
    user_is_speaking = py_trees.behaviours.Success("User is Speaking")
    user_turn = py_trees.behaviours.Success("User Is Allowed Turn")
    attention_evidence = py_trees.behaviours.Failure("Give Evidence of Attention,etc")
    user_initiative.add_children([user_is_speaking,user_turn,attention_evidence])

    ########third row 
    robot_turn = py_trees.composites.Selector(name="Robot has Turn")
    execute_presentation = py_trees.composites.Sequence(name="Execute Presentation")
    robot_initiative.add_children([robot_turn,execute_presentation])

    ################fourth row 
    robot_speaking = py_trees.behaviours.Success("Robot is Speaking")
    robot_takes_turn = py_trees.behaviours.Success("Robot takes Turn")
    robot_turn.add_children([robot_speaking,robot_takes_turn])

    ################fourth row 
    react_user_input = py_trees.behaviours.Success("React to User Input")
    #react_user_input = py_trees.composites.Selector(name="React to User Input")
    ensure_joint_attention = py_trees.composites.Sequence(name="Ensure Joint Attention")
    deliver_presentation = py_trees.composites.Sequence(name="Deliver Presentation")
    execute_presentation.add_children([react_user_input, deliver_presentation])

    ################################fifth row 
    speak = py_trees.composites.Selector(name="Speak")
    show_paint = ShowPainting(const.SCREAM_PAINTING_URL)
    #show_paint = py_trees.behaviours.Success("Show Painting on Tablet")
    deliver_presentation.add_children([show_paint, speak])

    ################################################################sixth row
    permit_robot_speak = PermitRobotSpeak()
    #permit_robot_speak = py_trees.behaviours.Success("Permit Robot Speak")
    speak_mic = Speaking(const.SCREAM_PAINTING)
    #speak_mic = py_trees.behaviours.Success("Robot Starts Speaking")
    speak.add_children([permit_robot_speak,speak_mic])

    root.add_child(establish_enagagment)
    root.add_child(interact_with_user)
    #root.add_child(user_is_speaking)
    return root



def main():
    root = create_tree()

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    print(py_trees.display.print_ascii_tree(root))
    # Visualized the behavior tree - path:./catkin_ws
    print(py_trees.display.render_dot_tree(root))

    behaviour_tree = py_trees.trees.BehaviourTree(root)
    behaviour_tree.add_pre_tick_handler(pre_tick_handler)
    behaviour_tree.setup(timeout=15)


    for _unused_i in range(0, 1):
        try:
            #py_trees.console.read_single_keypress()
            behaviour_tree.tick()
            time.sleep(1)
        except KeyboardInterrupt:
            break
    
    print("\n")