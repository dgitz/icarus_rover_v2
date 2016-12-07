#!/usr/bin/env python
# license removed for brevity
import rospy
import tensorflow
from std_msgs.msg import String
import pdb
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('imageclassifier_node', anonymous=True)
    label_lines = [line.rstrip() for line 
                   in tensorflow.gfile.GFile(
                       "/home/robot/other_packages/tensorflow/tensorflow/examples/label_image/data/imagenet_comp_graph_label_strings.txt")]
                   
    for i in range(0,len(label_lines)):
        print label_lines[i]
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException as e:
        pass