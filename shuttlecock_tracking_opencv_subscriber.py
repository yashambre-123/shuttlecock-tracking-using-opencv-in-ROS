#!/usr/bin/env python


import numpy
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def process_contours(binary_masked_image,rgb_image,contours):
    black_image = numpy.zeros([binary_masked_image.shape[0], binary_masked_image.shape[1],3],'uint8')
    for c in contours:
        area=cv2.contourArea(c)
        perimeter=cv2.arcLength(c,True)
        ((x,y),radius)=cv2.minEnclosingCircle(c)
        if (area>100):
            cv2.drawContours(rgb_image,[c],-1,(255,0,255),2)
            cv2.drawContours(black_image,[c],-1,(255,0,255),2)
            cx,cy=get_contour_centre(c)
            cv2.circle(rgb_image,(cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image,(cx,cy),(int)(radius),(0,0,255),1)
    print("Number of contours:",len(contours))
    cv2.imshow("Contour Black Image",black_image)
    cv2.imshow("Contour RGB Image",rgb_image)
    cv2.waitKey(1)


def get_contour_centre(c):
    M = cv2.moments(c)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy



def my_callback(ros_image):
    bridge=CvBridge()
    rgb_image=bridge.imgmsg_to_cv2(ros_image,"bgr8")
    cv2.imshow("RGB Image",rgb_image)
    hsv_image=cv2.cvtColor(rgb_image,cv2.COLOR_BGR2HSV)
    cv2.imshow("HSV Image",hsv_image)
    yellow_lower=(30,100,200)
    yellow_upper=(60,255,255)
    binary_masked_image=cv2.inRange(hsv_image,yellow_lower,yellow_upper)
    cv2.imshow("Binary Masked Image",binary_masked_image)
    contours,hierarchy=cv2.findContours(binary_masked_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    process_contours(binary_masked_image,rgb_image,contours)

if __name__=="__main__":
    rospy.Subscriber("/usb_cam/image_raw",Image,my_callback)
    rospy.init_node("my_shuttlecock_node",anonymous=True)
    rospy.spin()
