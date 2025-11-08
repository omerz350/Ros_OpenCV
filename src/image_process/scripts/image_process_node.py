#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def callback(msg):
    bridge = CvBridge()
    # ROS -> OpenCV 
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # 1) Grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 2) Edge Detection (Canny) 
    edges = cv2.Canny(gray, 100, 200)

    # 3) Text Eklemek 
    text_image = frame.copy()
    cv2.putText(text_image, "ROS + OpenCV", (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Geri ROS formatına çevir
    gray_msg = bridge.cv2_to_imgmsg(gray, encoding="mono8")
    edge_msg = bridge.cv2_to_imgmsg(edges, encoding="mono8")
    text_msg = bridge.cv2_to_imgmsg(text_image, encoding="bgr8")

    # Publish et
    pub_gray.publish(gray_msg)
    pub_edge.publish(edge_msg)
    pub_text.publish(text_msg)

def main():
    rospy.init_node('image_processing_node', anonymous=True)

    global pub_gray, pub_edge, pub_text
    pub_gray = rospy.Publisher('/image_gray', Image, queue_size=10)
    pub_edge = rospy.Publisher('/image_edges', Image, queue_size=10)
    pub_text = rospy.Publisher('/image_text', Image, queue_size=10)

    rospy.Subscriber('/usb_cam/image_raw', Image, callback)

    rospy.loginfo("Image Processing Node Started.")
    rospy.spin()

if __name__ == '__main__':
    main()
