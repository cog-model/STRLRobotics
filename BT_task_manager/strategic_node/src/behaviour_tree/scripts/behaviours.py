import rospy
import py_trees
import actionlib
from actionlib_msgs.msg import GoalStatus

from communication_msgs.msg import PickupObjectGoal, PutObjectGoal, MoveToGoal, FindObjectGoal
from husky_tidy_bot_cv.srv import Reset
from instances import Instruction, LLMTask, LLMTask_object, Command


class TasksParser(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.bb = py_trees.blackboard.Blackboard()

        self.logger.warning(f"Wait for server")
        rospy.wait_for_service("/tracker_3d/reset")
        self.logger.info(f"Ready")
        self.reset_3d_tracking_service = rospy.ServiceProxy("/tracker_3d/reset", Reset)

    def initialize(self):
        self.logger.debug(f"Initialize")

    def update(self):
        tasks = self.bb.get("tasks").tasks

        if not tasks:
            return py_trees.common.Status.FAILURE

        parsed_tasks = []
        for _task in tasks:
            if Instruction[_task.type] in (Instruction.MOVE_TO, Instruction.PICK_UP,
                                           Instruction.PUT, Instruction.RECOGNIZE):
                if not _task.object or not _task.location:
                    return py_trees.common.Status.FAILURE
                parsed_tasks.append(LLMTask(type=Instruction[_task.type],
                                            object=_task.object,
                                            location=_task.location))
            elif Instruction[_task.type] == Instruction.FIND:
                parsed_tasks.append(LLMTask_object(type=Instruction[_task.type],
                                                   object=_task.object))
            elif Instruction[_task.type] == Instruction.STOP:
                parsed_tasks.append(Command(type=Instruction[_task.type]))
            else:
                self.logger.error(f"Not implemented")
                return py_trees.common.Status.FAILURE

        self.bb.set("parsed_tasks", parsed_tasks)

        ### reset 3d tracking here to increase delay between new command call in plan
        self.logger.warning(f"!!! Resetting 3D tracking !!!")
        self.reset_3d_tracking_service()

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Terminated with status {new_status}")

class CurrentTaskSetter(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.bb = py_trees.blackboard.Blackboard()
        self.locked = True

    def initialize(self):
        self.logger.debug(f"Initialize")
        self.locked = True
        # parallel_root is a "pseudo parallel": subtrees will tick sequentially even if success_on_one will trigger
        # => lock mechanism to prevent task setting on success tick of interceptors

    def update(self):
        tasks = self.bb.get("parsed_tasks")

        if self.locked and tasks:
            self.locked = False
            return py_trees.common.Status.RUNNING
        if not tasks:
            return py_trees.common.Status.FAILURE

        self.bb.set("current_task", tasks.pop(0))
        self.locked = True
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.locked = True
        self.logger.debug(f"Terminated with status {new_status}")

class EmergencyStop(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.bb = py_trees.blackboard.Blackboard()

    def initialize(self):
        self.logger.debug(f"Initialize")

    def update(self):
        current_task = self.bb.get("current_task")
        if current_task and (current_task.type == Instruction.STOP):
            self.logger.warning("!!! EmergencyStop !!!")
            self.bb.set("tasks", None)
            self.bb.set("parsed_tasks", None)
            self.bb.set("current_task", None)
            current_client = self.bb.get("current_client")
            if current_client:
                current_client.cancel_goal()
                current_client.stop_tracking_goal()
                self.logger.warning(f"!!! Send cancel_goal to {current_client.action_client.ns} !!!")
                self.bb.set("current_client", None)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"Terminated with status {new_status}")

class ActionServer(py_trees.behaviour.Behaviour):
    def __init__(self, name, action_server_name, action):
        super().__init__(name)
        self.name = name
        self.client = actionlib.SimpleActionClient(action_server_name, action)
        self.logger.warning(f"Wait for server")
        self.client.wait_for_server()
        self.logger.info(f"Ready")
        self.bb = py_trees.blackboard.Blackboard()
        self.name_to_goal_correspondences = {"PUT": PutObjectGoal,
                                             "PICK_UP": PickupObjectGoal,
                                             "MOVE_TO": MoveToGoal,
                                             "FIND": FindObjectGoal}

    def initialize(self):
        self.logger.debug(f"Initialize")

    def update(self):
        status = self.client.get_state()
        self.logger.debug(str(status))

        if status == GoalStatus.ACTIVE:
            return py_trees.common.Status.RUNNING
        if status in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED):
            result = None
            try:
                result = self.client.get_result()
            except:
                self.logger.warning("!!! Can't get result, perhaps server died !!!")
            current_task_ = self.bb.get("current_task")
            self.bb.set("current_task", None)
            self.client.stop_tracking_goal()
            self.bb.set("current_client", None)
            if status == GoalStatus.SUCCEEDED:
                self.logger.info(f"{self.name} is SUCCEEDED with result: {str(result)}")
                return py_trees.common.Status.SUCCESS
            elif status == GoalStatus.ABORTED:
                self.logger.warning(f"!!! {self.name} result: {str(result)} !!!")
                if result and result.code == 1 and self.name == "MOVE_TO":
                    tasks = self.bb.get("parsed_tasks")
                    tasks.insert(0, LLMTask_object(type=Instruction.FIND, object=current_task_.object))
                    tasks.insert(1, current_task_)
                    self.bb.set("parsed_tasks", tasks)
                    return py_trees.common.Status.SUCCESS
                return py_trees.common.Status.FAILURE

        current_task = self.bb.get("current_task")
        if current_task and (current_task.type.name == self.name):
            if isinstance(current_task, LLMTask_object):
                goal = self.name_to_goal_correspondences[self.name](
                    object=current_task.object)
            elif isinstance(current_task, LLMTask):
                goal = self.name_to_goal_correspondences[self.name](
                    object=current_task.object,
                    location=current_task.location)
            else:
                self.logger.error(f"Current task {current_task} goal parsing is not implemented!")
                return py_trees.common.Status.FAILURE
            self.bb.set("current_client", self.client)
            self.logger.info(f"Send goal to {self.name}: {current_task}")
            self.client.send_goal(goal)
            return py_trees.common.Status.RUNNING

        if status == GoalStatus.LOST:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"Terminated with status {new_status}")

class FromBlackBoard(py_trees.behaviour.Behaviour):
    def __init__(self, name, action_client, action_goal, blackboard_var):
        super().__init__(name)
        self.name = name
        self.client = action_client
        self.goal = action_goal        

        self.logger.warning(f"Wait for server")
        self.client.wait_for_server()
        self.logger.info(f"Ready")

        self.bb = py_trees.blackboard.Blackboard()
        self.blackboard_var = blackboard_var

    def update(self):
        status = self.client.get_state()
        self.logger.debug(str(status))

        if status == GoalStatus.ACTIVE:
            return py_trees.common.Status.RUNNING
        if status == GoalStatus.SUCCEEDED:
            corrected_plan = self.client.get_result() # TaskArray msg
            self.bb.set(self.blackboard_var, corrected_plan.result)
            self.client.stop_tracking_goal()
            return py_trees.common.Status.SUCCESS
        if status == GoalStatus.ABORTED:
            self.client.stop_tracking_goal()
            return py_trees.common.Status.FAILURE

        var = self.bb.get(self.blackboard_var) # TaskArray msg
        self.logger.info(f"Setting OpenSeed with {var}")
        self.client.send_goal(self.goal(var))
        return py_trees.common.Status.RUNNING

    def initialize(self):
        self.logger.debug(f"Initialize")

    def terminate(self, new_status):
        self.logger.debug(f"Terminated with status {new_status}")
