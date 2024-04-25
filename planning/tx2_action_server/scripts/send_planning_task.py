#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

rospy.init_node('send_planning_task')
task_publisher = rospy.Publisher('/task', Float32MultiArray, queue_size=5)
task = Float32MultiArray()
task.layout.data_offset = 0
task.layout.dim.append(MultiArrayDimension())
task.layout.dim[0].label = "width"
task.layout.dim[0].size  = 4
task.layout.dim[0].stride  = 4
task.data = [-87, -12, -90, -12]
print('Task:', task)
rospy.sleep(1)
task_publisher.publish(task)
rospy.sleep(1)
print('Task published')
