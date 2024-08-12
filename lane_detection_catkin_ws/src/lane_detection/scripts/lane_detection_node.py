#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultrafastLaneDetector import UltrafastLaneDetector, ModelType

class LaneDetectionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.model_path = os.path.join(os.path.dirname(__file__), '../models/tusimple_18.pth')
        self.model_type = ModelType.TUSIMPLE
        self.use_gpu = False
        self.lane_detector = UltrafastLaneDetector(self.model_path, self.model_type, self.use_gpu)
        
        # Update the topic to match your ROS bag
        self.image_sub = rospy.Subscriber("/resized/camera_fl/image_color", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/lane_detection/output", Image, queue_size=1)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Detect lanes
            output_img = self.lane_detector.detect_lanes(cv_image)
            
            # Convert OpenCV image back to ROS Image message
            output_msg = self.bridge.cv2_to_imgmsg(output_img, "bgr8")
            
            # Publish the output image
            self.image_pub.publish(output_msg)
            
            # Display the image using OpenCV
            cv2.imshow("Lane Detection Output", output_img)
            cv2.waitKey(1)  # Add a small delay to allow the image to be displayed
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    rospy.init_node('lane_detection_node', anonymous=True)
    lane_detection_node = LaneDetectionNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down lane detection node")
    cv2.destroyAllWindows()
