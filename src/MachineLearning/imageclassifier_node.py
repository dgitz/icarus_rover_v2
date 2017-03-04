#!/usr/bin/env python
# license removed for brevity

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
import sys
import socket
sys.path.append(os.path.abspath("/home/robot/catkin_ws/src/icarus_rover_v2/include"))
from Definitions import *
from icarus_rover_v2.msg import diagnostic
from icarus_rover_v2.msg import heartbeat
from icarus_rover_v2.msg import usermessage
from icarus_rover_v2.msg import command

#topic: /dgitzrosmaster_cameracapture_node/raw_image
cv_image = []
image = []
image_array = []
bridge = []
received_image_counter = 0
image_width = 0
image_height = 0
image_channels = 0
last_target = ""
search_for_target = 1
import pdb
def command_callback(data):
    global search_for_target
    if(data.Command == FINDTARGET_ID):
        search_for_target = data.Option1
def image_callback(data):
    try:
        global cv_image
        global bridge
        global received_image_counter
        received_image_counter = received_image_counter + 1
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        global image_width
        global image_height
        global image_channels
        image_height,image_width,image_channels = cv_image.shape
    except CvBridgeError as e:
        print(e)
def runnode():
    
    
    
    rospy.init_node('imageclassifier_node', anonymous=True)
    diag_topic = socket.gethostname() + '_imageclassifier_node/diagnostic'
    diagnostic_pub = rospy.Publisher(diag_topic,diagnostic,queue_size=10)
    heartbeat_topic = socket.gethostname() + '_imageclassifier_node/heartbeat'
    heartbeat_pub = rospy.Publisher(heartbeat_topic,heartbeat,queue_size=10)
    usermessage_pub = rospy.Publisher('/usermessage',usermessage,queue_size=10)
    command_sub = rospy.Subscriber('/command',command,command_callback)
    diag = diagnostic()
    diag.DeviceName = socket.gethostname()
    diag.Node_Name = rospy.get_name()
    diag.System = ROVER
    diag.SubSystem = ENTIRE_SYSTEM
    diag.Component = VISION_NODE
    global cv_image
    global bridge
    global received_image_counter
    global image_width
    global image_height
    global image_channels
    global last_target
    received_image = 0
    bridge = CvBridge()
    image_topic = rospy.get_param('~image_topic')
    rospy.Subscriber(image_topic,Image,image_callback)
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
            beat = heartbeat()
            beat.Node_Name = socket.gethostname() + '_imageclassifier_node'
            beat.stamp = rospy.Time().now()
            heartbeat_pub.publish(beat)
            #if(received_image == 0):
            #    a = 1;#print "Waiting on image"
            if(search_for_target == 1):
                if(received_image_counter == 10):
                    received_image_counter = 0;
                    start = time.time()
                    v = cv_image.reshape(image_height,image_width,image_channels)
                    predictions = sess.run(softmax_tensor,{'DecodeJpeg:0': v})
                    top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]
                    end = time.time()
                    received_image = 0;
                    #rospy.loginfo("Duration: %.5f",end-start)
                    top_score  = predictions[0][top_k[0]]
                    #print top_score
                    if(top_score > 0.7):
                        diag.Diagnostic_Type = SENSORS
                        diag.Level = NOTICE
                        diag.Diagnostic_Message = NOERROR
                        diag.Description = "Found Target " + label_lines[top_k[0]]
                        print diag.Description
                        diagnostic_pub.publish(diag)
                        usermsg = usermessage()
                        usermsg.Level = LEVEL1
                        found_target = label_lines[top_k[0]]
                        if(found_target != last_target):
                            usermsg.message = "Found Target " + found_target
                            usermessage_pub.publish(usermsg)
                            last_target = found_target
                    #rospy.loginfo("Label: %s score: %.5f",label_lines[top_k[0]],predictions[0][top_k[0]])
                
                #label_lines[top_k[0]],predictions[0][top_k[0]])
                
#                 count = 0
#                 for node_id in top_k:
#                     count = count + 1
#                     if(count > 5):
#                         break
#                     human_string = label_lines[node_id]
#                     score = predictions[0][node_id]
#                     print('%s (score = %.5f)' % (human_string, score))
#                rospy.loginfo("Label: %s score: %.5f",label_lines[top_k[0]],predictions[0][top_k[0]])
        except Exception as e: 
            print(e)
        rate.sleep()

if __name__ == '__main__':
    try:
        runnode()
    except rospy.ROSInterruptException as e:
        pass
