import rospy
import py_trees
import actionlib
from actionlib_msgs.msg import GoalStatus
from communication_msgs.msg import \
    PickObjectAction, PickObjectGoal, \
    PlaceObjectAction, PlaceObjectGoal


class PickObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.client = actionlib.SimpleActionClient("pick_object", PickObjectAction)
        self.logger.info(f"wait_for_server")
        #self.client.wait_for_server()

    def initialise(self):
        self.client.send_goal(PickObjectGoal(what="pass", where="pass"))
        rospy.sleep(0.25) # Ensure goal was received before checking state

    def update(self):
        status = self.client.get_state()
        if status == GoalStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        if status == GoalStatus.ACTIVE or status == GoalStatus.PENDING:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.FAILURE
        
    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")

class PlaceObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.client = actionlib.SimpleActionClient("place_object", PlaceObjectAction)
        self.logger.info(f"wait_for_server")
        #self.client.wait_for_server()

    def initialise(self):
        self.client.send_goal(PlaceObjectGoal(where="pass"))
        rospy.sleep(0.25) # Ensure goal was received before checking state

    def update(self):
        status = self.client.get_state()
        if status == GoalStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        if status == GoalStatus.ACTIVE or status == GoalStatus.PENDING:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.FAILURE
        
    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")
