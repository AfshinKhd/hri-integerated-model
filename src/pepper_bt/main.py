#!/usr/bin/env python

import py_trees
import time



def create_tree() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Selector(name="Root")
    return root



def main() -> None:
    root = create_tree()

    print(py_trees.display.print_ascii_tree(root))
    # Visualized the behavior tree - path:./catkin_ws
    print(py_trees.display.render_dot_tree(root))

    behaviour_tree = py_trees.trees.BehaviourTree(root)
    behaviour_tree.setup(timeout=15)


    for _unused_i in range(0, 4):
        try:
            behaviour_tree.tick()
            time.sleep(0.5)
        except KeyboardInterrupt:
            break
    
    print("\n")