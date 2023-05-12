#!/usr/bin/env python

import py_trees
import time
from pepper_bt.topics import Speaking

def pre_tick_handler(behaviour_tree: py_trees.trees.BehaviourTree) -> None:
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)



def create_tree() -> py_trees.behaviour.Behaviour:
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

    ########third row <== second row - left leaves
    #user_is_speaking = Speaking(message="Test")
    user_is_speaking = py_trees.behaviours.Success("User Is Speaking")
    user_turn = py_trees.behaviours.Success("User Is Allowed Speaking")
    attention_evidence = py_trees.behaviours.Success("Give Evidence of Attention,etc")
    user_initiative.add_children([user_is_speaking,user_turn,attention_evidence])

    root.add_child(establish_enagagment)
    root.add_child(interact_with_user)
    #root.add_child(user_is_speaking)
    return root



def main() -> None:
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
            behaviour_tree.tick()
            time.sleep(0.5)
        except KeyboardInterrupt:
            break
    
    print("\n")