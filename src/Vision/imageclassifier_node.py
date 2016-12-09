#!/usr/bin/env python
# license removed for brevity
import rospy
import tensorflow
from std_msgs.msg import String
import time
import pdb
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('imageclassifier_node', anonymous=True)
    label_lines = [line.rstrip() for line 
                   in tensorflow.gfile.GFile(
                       "/home/robot/other_packages/tensorflow/tensorflow/examples/label_image/data/imagenet_comp_graph_label_strings.txt")]
                   
    with tensorflow.gfile.FastGFile("/home/robot/other_packages/tensorflow/tensorflow/examples/label_image/data/tensorflow_inception_graph.pb", 'rb') as f:
        graph_def = tensorflow.GraphDef()
        graph_def.ParseFromString(f.read())
        _ = tensorflow.import_graph_def(graph_def, name='')
    with tensorflow.Session() as sess:
        softmax_tensor = sess.graph.get_tensor_by_name('softmax:0')
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        image_data = tensorflow.gfile.FastGFile("/home/robot/config/targets/outlet/outlet1.jpg", 'rb').read()
        
        start = time.time()
        
        predictions = sess.run(softmax_tensor, \
           {'DecodeJpeg/contents:0': image_data})
        top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]
        end = time.time()
        rospy.loginfo("Duration: %.5f",end-start)
        for j in range(0,5):
             rospy.loginfo("Label: %s score: %.5f",label_lines[top_k[j]],predictions[0][top_k[j]])
                
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException as e:
        pass