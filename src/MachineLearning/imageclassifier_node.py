#!/usr/bin/env python
# license removed for brevity
#topic: /dgitzrosmaster_cameracapture_node/raw_image
import rospy
import tensorflow
from std_msgs.msg import String
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import glob
#cv_image = []
#bridge = []
import pdb
#def image_callback(data):
#    try:
#        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#    except CvBridgeError as e:
#        print(e)
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('imageclassifier_node', anonymous=True)
    #bridge = CvBridge()
    #rospy.Subscriber("/dgitzrosmaster_cameracapture_node/raw_image",Image,image_callback)
    label_lines = [line.rstrip() for line 
                   in tensorflow.gfile.GFile(
                       #"/home/robot/other_packages/tensorflow/tensorflow/examples/label_image/data/imagenet_comp_graph_label_strings.txt")]
                       "/home/robot/config/output_labels.txt")]
                       #"/home/robot/other_packages/tensorflow/tensorflow/examples/classify_image/imagenet_synset_to_human_label_map.txt")]
    with tensorflow.gfile.FastGFile(
        #"/home/robot/other_packages/tensorflow/tensorflow/examples/label_image/data/tensorflow_inception_graph.pb", 'rb') as f:
        "/home/robot/config/output_graph.pb",'rb') as f:
        #"/home/robot/other_packages/tensorflow/tensorflow/examples/classify_image/classify_image_graph_def.pb",'rb') as f:
        graph_def = tensorflow.GraphDef()
        graph_def.ParseFromString(f.read())
        _ = tensorflow.import_graph_def(graph_def, name='')
    with tensorflow.Session() as sess:
        #For classify_image and label_image: softmax:0
        #For retrained label_image: final_result:0
        softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')
        
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        try:
        #image_data = tensorflow.gfile.FastGFile("/home/robot/config/targets/train/outlet/2017_01_09_02_22_16.jpg", 'rb').read()
		
            #newest = max(glob.iglob('/home/robot/Pictures/*.jpg'), key=os.path.getctime)
			newest = max(glob.iglob('/home/robot/external/Unsorted-Pictures/*.jpg'), key=os.path.getctime)
            #print newest
            image_data = tensorflow.gfile.FastGFile(newest,'rb').read()
            start = time.time()
            
            predictions = sess.run(softmax_tensor, \
               {'DecodeJpeg/contents:0': image_data})
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
            donothing = 1
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException as e:
        pass
