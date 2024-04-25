import yaml
import rospy
import py_trees
import actionlib
import py_trees_ros

from communication_msgs.msg import TaskArray
from communication_msgs.msg import PickupObjectAction, PutObjectAction, \
                                   MoveToAction, FindObjectAction, \
                                   OpenSeeDSetterAction, OpenSeeDSetterGoal
from behaviours import ActionServer, TasksParser, CurrentTaskSetter, EmergencyStop, FromBlackBoard


def construct_tree(cfg):
    parallel_root = py_trees.composites.Parallel(name="parallel_root",
                                                 policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

    task_parser_sequence = py_trees.composites.Sequence(name="task_parser_sequence")
    task_executor_sequence = py_trees.composites.Sequence(name="task_executor_sequence")

    parallel_root.add_children([task_parser_sequence, task_executor_sequence])

    parallel_interceptors = py_trees.composites.Parallel(name="parallel_interceptors",
                                                         policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    
    segmentation_setter = FromBlackBoard(name="OpenSeeDSet",
                                         action_client=actionlib.SimpleActionClient("text_query_generation", OpenSeeDSetterAction),
                                         action_goal=OpenSeeDSetterGoal,
                                         blackboard_var="tasks")
    task_parser = TasksParser(name="TasksParser")
    
    task_parser_sequence.add_children([parallel_interceptors, segmentation_setter, task_parser])

    llm_plan_interceptor = py_trees_ros.subscribers.ToBlackboard(name="llm_plan_interceptor",
                                                                 topic_name=cfg["llm_topic"],
                                                                 topic_type=TaskArray,
                                                                 blackboard_variables = {"tasks": None})
    telegram_command_interceptor = py_trees_ros.subscribers.ToBlackboard(name="telegram_command_interceptor",
                                                                 topic_name=cfg["telegram_commands_topic"],
                                                                 topic_type=TaskArray,
                                                                 blackboard_variables = {"tasks": None})
    parallel_interceptors.add_children([llm_plan_interceptor, telegram_command_interceptor])

    current_task_selector = py_trees.composites.Selector(name="current_task_selector")
    tasks = py_trees.composites.Selector(name="tasks")
    task_executor_sequence.add_children([current_task_selector, tasks])

    current_task_setter = CurrentTaskSetter(name="current_task_setter")
    idle = py_trees.behaviours.Running(name="idle")
    current_task_selector.add_children([current_task_setter, idle])
    
    emergency_stop = EmergencyStop(name="emergency_stop?")
    parallel_tasks = py_trees.composites.Parallel(name="parallel_tasks",
                                                  policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    tasks.add_children([emergency_stop, parallel_tasks])
    parallel_tasks.add_children([ActionServer(name="PUT", action_server_name="put_object", action=PutObjectAction),
                                 ActionServer(name="PICK_UP", action_server_name="pick_up_object", action=PickupObjectAction),
                                 ActionServer(name="MOVE_TO", action_server_name="move_to_location", action=MoveToAction),
                                 ActionServer(name="FIND", action_server_name="find_object", action=FindObjectAction)])
    return parallel_root

if __name__=="__main__":
    rospy.init_node("behaviour_tree", log_level=rospy.INFO)
    
    with open('main.yml', 'r') as file:
        cfg = yaml.safe_load(file)

    parallel_root = construct_tree(cfg)

    ros_tree = py_trees_ros.trees.BehaviourTree(parallel_root)
    bb = py_trees.blackboard.Blackboard()

    ros_tree.setup(timeout=10.0)
    py_trees.logging.level = py_trees.logging.Level.INFO 

    done = False
    while not done and not rospy.is_shutdown():
        rospy.logdebug("<----------------------------------->")
        ros_tree.tick()
        if ros_tree.root.status == py_trees.common.Status.FAILURE:
            rospy.logerr(f"{ros_tree.root.tip().name} FAILED")
            done = True
        rospy.sleep(cfg["tree_tick_rate"])

        rospy.logdebug(f"parsed_tasks: {bb.get('parsed_tasks')}")
        rospy.logdebug(f"current_task: {bb.get('current_task')}")
