from fixed_cats import FIXED_CATEGORIES
from husky_tidy_bot_cv.msg import Categories
from communication_msgs.msg import OpenSeeDSetterAction, OpenSeeDSetterResult, OpenSeeDSetterFeedback
from std_msgs.msg import String
from conversions import to_classes_msg
import rospy
import actionlib
from kas_utils.time_measurer import TimeMeasurer
import copy
import open_clip
import torch


class TextGenerationAction:
    # create messages that are used to publish feedback/result
    _feedback = OpenSeeDSetterFeedback()
    _result = OpenSeeDSetterResult()

    def __init__(self, name, location_status_topic):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            OpenSeeDSetterAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )

        self.out_location_status_topic = location_status_topic

        self.out_location_status_topic = rospy.Publisher(
            self.out_location_status_topic, String, queue_size=1)
        
        self.categories = copy.deepcopy(FIXED_CATEGORIES)
        self.num_categories = len(self.categories)
        self.categories_name = [cat["name"] for cat in self.categories]
        self.cat_name_to_id = {cat["name"]: cat["id"] for cat in self.categories}
        self.from_ros_tm = TimeMeasurer("  from ros")
        self.text_query_generation_tm = TimeMeasurer("  text query generation")
        self.to_ros_tm = TimeMeasurer("  to ros")
        self.total_tm = TimeMeasurer("total")

        self.model_clip, _, self.preprocess = open_clip.create_model_and_transforms('RN101', pretrained='openai')
        self.tokenizer = open_clip.get_tokenizer('RN101')
        self.model_clip.cuda()

        self.cats_for_clip = ["white", "orange", "other", "yellow", "black", "blue", "green"]
        self.text = self.tokenizer(self.cats_for_clip).cuda()
        with torch.no_grad(), torch.cuda.amp.autocast():
            self.text_features = self.model_clip.encode_text(self.text)
            self.text_features /= self.text_features.norm(dim=-1, keepdim=True)

        self.similarity_threshold = 0.8
        self.categories_to_publish = self.categories
        self._as.start()
        rospy.loginfo('%s: Started server to creaste a list of categories from Task Array' % (self._action_name))

    def execute_cb(self, goal):
        # helper variables
        success = True
        categories_to_publish = []
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating list of categories from Task Array' % (self._action_name))
        
        # start executing the action
        with self.total_tm:
            with self.from_ros_tm:
                print(goal._type)
                if goal._type == "communication_msgs/OpenSeeDSetterGoal":
                    tasks = goal.tasks.tasks
                else:
                    raise RuntimeError("Unkown message type")

            with self.text_query_generation_tm:
                for task in tasks:
                    # check that preempt has not been requested by the client
                    if self._as.is_preempt_requested():
                        rospy.loginfo('%s: Preempted' % self._action_name)
                        self._as.set_preempted()
                        success = False
                        break
                    task_object = task.object
                    task_location = task.location
                    category_to_publish = {}
                    if task_object != 'unspecified' and task_object != 'objects' and task_object != '':
                        if task_object in self.categories_name:
                            category_to_publish = self.categories[self.cat_name_to_id[task_object]]
                        else:
                            category_to_publish = {
                                "id": self.num_categories,
                                "name": task_object,
                                'supercategory': '',
                            }

                        publishing_names = [cat["name"] for cat in categories_to_publish]

                        if category_to_publish["name"] not in publishing_names:
                            categories_to_publish.append(category_to_publish)
                            if category_to_publish not in self.categories:
                                self.categories.append({
                                    "id": self.num_categories,
                                    "name": category_to_publish["name"],
                                    'supercategory': '',
                                })
                                self.num_categories = len(self.categories)
                                self.categories_name = [cat["name"] for cat in self.categories]
                                self.cat_name_to_id = {cat["name"]: cat["id"] for cat in self.categories}

                    if task_location != 'unspecified' and task_location != 'objects' and task_location != '':
                        category_to_publish = {}
                        if task_location in self.categories_name:
                            category_to_publish = self.categories[self.cat_name_to_id[task_location]]
                        elif task_location in ["box", "bin", "container", "basket"]:
                            category_to_publish = self.categories[self.cat_name_to_id['box']]
                        elif task_location in ["nightstand", "bedside table", "cabinet", "basket", "bedside-table"]:
                            category_to_publish = self.categories[self.cat_name_to_id["nightstand"]]
                        else:
                            category_to_publish = {
                                "id": self.num_categories,
                                "name": task_location,
                                'supercategory': '',
                            }

                        publishing_names = [cat["name"] for cat in categories_to_publish]

                        if category_to_publish["name"] not in publishing_names:
                            categories_to_publish.append(category_to_publish)
                            if category_to_publish not in self.categories:
                                self.categories.append({
                                    "id": self.num_categories,
                                    "name": category_to_publish["name"],
                                    'supercategory': '',
                                })
                                self.num_categories = len(self.categories)
                                self.categories_name = [cat["name"] for cat in self.categories]
                                self.cat_name_to_id = {cat["name"]: cat["id"] for cat in self.categories}

                    if task_object == "objects" or task_location == "objects":
                        for cat in FIXED_CATEGORIES:
                            if cat["name"] not in categories_to_publish:
                                categories_to_publish.append(cat)

        self.categories_to_publish = categories_to_publish
        # publish the feedback
        self._as.publish_feedback(self._feedback)


        self._result.result = goal.tasks

        for task in self._result.result.tasks:
            if task.location in ["box", "bin", "container", "basket"]:
                task.location = "box"
            elif task.location in ["nightstand", "bedside table", "cabinet", "basket", "bedside-table"]:
                task.location = "nightstand"
            elif "box" in task.location or "bin" in task.location or "container" in task.location or "basket" in task.location:
                task.location = task.location.replace("bin", "box")
                task.location = task.location.replace("basket", "box")
                task.location = task.location.replace("container", "box")
                cat = self.tokenizer([task.location.split(" ")[0]]).cuda()
                with torch.no_grad(), torch.cuda.amp.autocast():
                    cats_features = self.model_clip.encode_text(cat)
                    cats_features /= cats_features.norm(dim=-1, keepdim=True)            
                cats_probs = cats_features @ self.text_features.T
                if torch.max(cats_probs[0]) > self.similarity_threshold:
                    new_class = int(torch.argmax(cats_probs[0]).cpu())
                    task.location = self.cats_for_clip[new_class] + " box"
            if task.location not in ["box", "nightstand", "table", "white box", "orange box", "unspecified", ""]:
                success = False

        print(self._result)
        if success:
            location_status = String()
            location_status.data = "OK!"
            self.out_location_status_topic.publish(location_status)
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            location_status = String()
            location_status.data = "Failed to find a location in LLM plan!"
            self.out_location_status_topic.publish(location_status)
            rospy.loginfo('%s: Failed' % self._action_name)
            self._as.set_aborted(self._result)


if __name__ == "__main__":
    rospy.init_node('text_query_generation_node')
    text_query_generation_server = TextGenerationAction("text_query_generation", "/location_status")

    out_labels_topic = rospy.Publisher(
        "/segmentation_labels", Categories, queue_size=1)

    while not rospy.is_shutdown():
        if len(text_query_generation_server.categories_to_publish) == 0:
            rospy.loginfo(f'Empty list of objects and locations. Nothing to segment.')
        classes_msg = to_classes_msg(text_query_generation_server.categories_to_publish)
        out_labels_topic.publish(classes_msg)
        rospy.sleep(1)

    del text_query_generation_server
