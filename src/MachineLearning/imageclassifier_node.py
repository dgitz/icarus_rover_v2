#!/usr/bin/env python
# license removed for brevity
#topic: /dgitzrosmaster_cameracapture_node/raw_image
import rospy
import tensorflow
import struct
from std_msgs.msg import String
import time
from sensor_msgs.msg import Image
import numpy as np
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
import os
import glob
#topic: /dgitzrosmaster_cameracapture_node/raw_image
cv_image = []
image = []
image_array = []
bridge = []
received_image = 0
import pdb
def image_callback(data):
    try:
        global cv_image
        global bridge
        global received_image
        received_image = 1
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
def runnode():
    rospy.init_node('imageclassifier_node', anonymous=True)
    global cv_image
    global bridge
    global received_image
    received_image = 0
    bridge = CvBridge()
    rospy.Subscriber("/ComputeModule1_cameracapture_node/raw_image",Image,image_callback)
    print "Loading Network Labels"
    label_lines = [line.rstrip() for line 
                   in tensorflow.gfile.GFile(
                       #"/home/robot/other_packages/tensorflow/tensorflow/examples/label_image/data/imagenet_comp_graph_label_strings.txt")]
                       "/home/robot/config/output_labels.txt")]
                       #"/home/robot/other_packages/tensorflow/tensorflow/examples/classify_image/imagenet_synset_to_human_label_map.txt")]
    print "Loading Network Graph"
    with tensorflow.gfile.FastGFile(
        #"/home/robot/other_packages/tensorflow/tensorflow/examples/label_image/data/tensorflow_inception_graph.pb", 'rb') as f:
        "/home/robot/config/output_graph.pb",'rb') as f:
        #"/home/robot/other_packages/tensorflow/tensorflow/examples/classify_image/classify_image_graph_def.pb",'rb') as f:
        graph_def = tensorflow.GraphDef()
        graph_def.ParseFromString(f.read())
        _ = tensorflow.import_graph_def(graph_def, name='')
    print "Creating Session"
    with tensorflow.Session() as sess:
        #For classify_image and label_image: softmax:0
        #For retrained label_image: final_result:0
        softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')
        
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        try:
            if(received_image == 0):
                print "Waiting on image"
            if(received_image == 1):
                start = time.time()
                v = cv_image.reshape(480,640,3)
                predictions = sess.run(softmax_tensor,{'DecodeJpeg:0': v})
                top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]
                end = time.time()
                rospy.loginfo("Duration: %.5f",end-start)
                count = 0
                for node_id in top_k:
                    count = count + 1
                    if(count > 5):
                        break
                    human_string = label_lines[node_id]
                    score = predictions[0][node_id]
                    print('%s (score = %.5f)' % (human_string, score))
                rospy.loginfo("Label: %s score: %.5f",label_lines[top_k[0]],predictions[0][top_k[0]])
        except Exception as e: 
            print(e)
        rate.sleep()

if __name__ == '__main__':
    try:
        runnode()
    except rospy.ROSInterruptException as e:
        pass
