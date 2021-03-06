from styx_msgs.msg import TrafficLight

import tensorflow as tf
import os
import cv2
import numpy as np
import time

import rospy
import sys

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

from utils import label_map_util
from utils import visualization_utils as vis_util

class TLClassifier(object):
    def __init__(self):
        """
        Code adapted from TensorflowModel API
        https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb
        """
        # Pass to network if nothing is detected.
        self.current_light = TrafficLight.UNKNOWN  
        modelDir = os.path.dirname(os.path.realpath(__file__))

        modelPath = os.path.join(modelDir, 'ssd_inception_coco_frz_ynb_C_021.pb')

        if not os.path.exists(modelPath):
            rospy.logerr('Frozen graph not found in directory: {}'.format(modelPath))
            rospy.loginfo('Frozen graph not found in directory: {}'.format(modelPath))

        """
        List of label strings for 1.Green, 2.Red, 3.Yellow and 4.Off
        """    
        labelsPath = os.path.join(modelDir, 'label.pbtxt')
        if not os.path.exists(labelsPath):
            rospy.logerr('Label file not found in directory: {}'.format(labelsPath))
        classNum = 4

        #Optimizing model
        self.config = tf.ConfigProto()
        self.config.gpu_options.allow_growth = True
        self.config.gpu_options.per_process_gpu_memory_fraction = 1
        jit_level = tf.OptimizerOptions.ON_1
        self.config.graph_options.optimizer_options.global_jit_level = jit_level

        ##### Build network
        """
        Load label map
        Obtain the categories List
        Take the indexes of categories
        """
        label_map = label_map_util.load_labelmap(labelsPath)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=classNum, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)
        
        
        self.detectGraph = tf.Graph()

        with self.detectGraph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(modelPath, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detectGraph, config=self.config)

        # Definite input and output Tensors for detectGraph
        self.image_tensor = self.detectGraph.get_tensor_by_name('image_tensor:0')

        """
        Each box represents a part of the image where a particular object was detected.
        """
        
        self.detection_boxes = self.detectGraph.get_tensor_by_name('detection_boxes:0')
        """
        Each score represent how level of confidence for each of the objects.
        Score is shown on the result image, together with the class label.
        """
        self.detection_scores = self.detectGraph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detectGraph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detectGraph.get_tensor_by_name('num_detections:0')

        print("Graph is loaded!!")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        img_exp = np.expand_dims(image, axis=0)

        #img_dir = os.path.abspath(os.path.join(os.getcwd(),"images"))
        #img_name = os.path.join(img_dir, "%12i.jpg" % time0)
        #img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)        
        #cv2.imwrite(img_name, img)

        # Detection.
        with self.detectGraph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores,
                self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: img_exp})

        #time1 = time.time()
        #print("Time in milliseconds", (time1 - time0) * 1000)

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
     
        vis_util.visualize_boxes_and_labels_on_image_array(
                image, boxes, classes, scores,
                self.category_index,
                use_normalized_coordinates=True,
                line_thickness=6)
        
        #img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        #img_name = os.path.join(img_dir, "%12i_vis.jpg" % time0)    
        #cv2.imwrite(img_name, img)

        min_score_thresh = .75
        max_score = 0
        max_id = 4
        max_name = ""
        for i in range(boxes.shape[0]):
            if scores is None or scores[i] > min_score_thresh:
                if scores[i] > max_score:
                    class_name = self.category_index[classes[i]]['name']
                    class_id = self.category_index[classes[i]]['id']  # if needed
                    max_name = class_name                    
                    max_score = scores[i]
                    max_id = i
        
        """
        Assign the categories based on scores
        """
        if max_name == 'Red':
            self.current_light = TrafficLight.RED
        elif max_name == 'Green':
            self.current_light = TrafficLight.GREEN
        elif max_name == 'Yellow':
            self.current_light = TrafficLight.YELLOW
        else:
            self.current_light = TrafficLight.UNKNOWN

        #print('Box #: {}'.format(max_id))
        #print('{}'.format(max_name))
        #print('{}'.format(max_score))

        return self.current_light

if __name__ == '__main__':
    try:
        TLClassifier()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start light_classifier node.')