#! /usr/bin/python3
import rospy
import argparse
import rostopic
import roslib
def arguments():
    args = argparse.ArgumentParser(prog='topic mirroring')
    args.add_argument('topic_from')
    args.add_argument('topic_to')
    return args.parse_args()

pub = None
def cb_publish(msg):
    pub.publish(msg)
    rospy.loginfo_once(f'PUBLISHED')

def main():
    args =  arguments()
    rospy.init_node('mirror', anonymous=True)
    topic_type = rostopic.get_topic_type(args.topic_from)[0]
    data_class =  roslib.message.get_message_class(topic_type)
    global pub
    pub = rospy.Publisher(args.topic_to, data_class, queue_size=10)
    rospy.Subscriber(args.topic_from, data_class, cb_publish)
    rospy.spin()
    
if __name__ == '__main__':
    main()
