#!/usr/bin/python3

import rospy
import py_trees
import py_trees_ros

from manipulation import PickObject, PlaceObject


if __name__=="__main__":
    rospy.init_node("strategic_node")
    # test_var = rospy.get_param("/test_var")

    root = py_trees.composites.Sequence(name=f"pick_and_place",
                                        children=[PickObject(f"pick_object"),
                                                  PlaceObject(f"place_object")])
    tree = py_trees.trees.BehaviourTree(root)
    ros_tree = py_trees_ros.trees.BehaviourTree(root)
    ros_tree.setup(timeout=3.0)
    py_trees.logging.level = py_trees.logging.Level.INFO

    done = False
    while not rospy.is_shutdown() and not done:
        ros_tree.tick()
        if ros_tree.root.status == py_trees.common.Status.SUCCESS:
            print("Behavior tree succeeded")
            done = True
        elif ros_tree.root.status == py_trees.common.Status.FAILURE:
            print("Behavior tree failed.")
            done = True
        rospy.sleep(0.5)