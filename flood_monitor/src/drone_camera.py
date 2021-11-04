#!/usr/bin/env python3

import rospy
import os 
import math
from std_msgs.msg import Float64MultiArray, String, Int32
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import numpy as np

from sensor_msgs.msg import Image, CameraInfo, CompressedImage

import configparser

import sys
import tempfile
import csv
import tf
import message_filters

import cv2, cv_bridge

targets = []

config = configparser.ConfigParser()
config.read('config.ini')

class Follower:
    def __init__(self):
        
        self.bridge = cv_bridge.CvBridge()
        image = rospy.Subscriber('camera/downward_cam/camera/image', Image, self.image_callback)
        self.status = rospy.Publisher('camera/flood_status', Int32, queue_size = 10)
        self.pos = rospy.Subscriber("controller", Float64MultiArray, self.pos_callback)
        self.data = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.cmd_vel_pub = rospy.Publisher('camera/cmd_vel',Twist, queue_size=10)
        self.twist = Twist()
     
    def pos_callback(self, data):  
        global targets
        targets = np.array(data.data)
        targets = targets.reshape([-1,2])
        
    def image_callback(self, msg):

        #print(d01_posZ)
        # Image segmentation, to differentiate which is water and land.
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8') #bgr8
        image = image[:, 30:image.shape[1]-30, :]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  
        lower_flood = np.array([96, 86, 28]) 
        upper_flood = np.array([104,255,255])
        lower_ground = np.array([1,1,1])
        upper_ground = np.array([95,175,180])
        lower_red = np.array([0,96,97])
        upper_red = np.array([0,255,255]) 
        mask_flood = cv2.inRange(hsv, lower_flood, upper_flood)
        mask_ground = cv2.inRange(hsv, lower_ground, upper_ground)
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        
        cont, hierarchy = cv2.findContours(mask_flood, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, cont, -1, (0,0,255),2)
        cont1, hierarchy = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, cont1, -1, (0,255,255),50)

        
        h, w, d = image.shape
        
        search_top = 0
        search_bot = h
        mask_flood[0:search_top, 0:w] = 0
        mask_flood[search_bot:h, 0:w] = 0
        mask_ground[0:search_top, 0:w] = 0
        mask_ground[search_bot:h, 0:w] = 0
        M_flood = cv2.moments(mask_flood)
        M_ground = cv2.moments(mask_ground)
        M_red = cv2.moments(mask_red)
        
        global control
        # Determine the state of seeing the flood or not, and then publishing the state.
        if M_flood['m00'] > 0 and M_ground['m00'] > 0:
            print('[camera] see water and ground')
            state = 1
            self.status.publish(state)
        
        elif M_flood['m00'] > 0 and M_flood['m10'] > 0 and M_ground['m00'] <= 0 and M_ground['m10'] <= 0:
            print('[camera] see only flood')
            state = 1
            self.status.publish(state)
        
        elif M_flood['m00'] <= 0 and M_flood['m10'] <= 0 and M_ground['m00'] > 0:
            print('[camera] see only ground')
            state = 0
            self.status.publish(state)
            
        else:
            print('[Exception!!!]')
            state = 1
            self.status.publish(state)
            
        cv2.imshow("camera", image)
        cv2.waitKey(3)
    

    def callback(self, data):
        #camera = data.pose[idx[17]]
        #camera_vel = data.twist[idx[17]]
        drone_camera = data.pose[int(config['index']['camera'])]
        drone_camera_vel = data.twist[int(config['index']['camera'])]
        drone_camera_posX = drone_camera.position.x
        drone_camera_posY = drone_camera.position.y
        drone_camera_posZ = drone_camera.position.z
        drone_camera_velX = drone_camera_vel.linear.x 
        drone_camera_velY = drone_camera_vel.linear.y
        drone_camera_velZ = drone_camera_vel.linear.z
        
         
        print("X:",drone_camera_posX)
        print("Y:",drone_camera_posY)
        print("Z:",drone_camera_posZ)
        pub = self.cmd_vel_pub
        calc_odom = (Vector3(0,0,-0.8*(drone_camera_posZ-25)))
        pub.publish(Twist((calc_odom),Vector3(0,0,0)))
        rospy.Rate(1).sleep
  
           

    
def listener():
    rospy.init_node('camera')
    follower = Follower()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
# END ALL
