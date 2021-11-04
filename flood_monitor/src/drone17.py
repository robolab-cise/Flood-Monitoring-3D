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
        """
        """
        self.bridge = cv_bridge.CvBridge()
        image = rospy.Subscriber('drone17/downward_cam/camera/image', Image, self.image_callback)
        self.status = rospy.Publisher('drone17/flood_status', Int32, queue_size = 10)
        self.pos = rospy.Subscriber("controller", Float64MultiArray, self.pos_callback)
        self.data = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.cmd_vel_pub = rospy.Publisher('drone17/cmd_vel',Twist, queue_size=10)
        self.twist = Twist()
     
    def pos_callback(self, data):
        print(data.data)
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
        lower_ground = np.array([1,1,1])  #gray np.array([0, 0, 0])
        upper_ground = np.array([95,175,180])#gray np.array([110,110, 110]) [95,175, 180]
        mask_flood = cv2.inRange(hsv, lower_flood, upper_flood)
        mask_ground = cv2.inRange(hsv, lower_ground, upper_ground)
        
        h, w, d = image.shape
        #print(h,w)
        
        search_top = 0#3*h/4
        search_bot = h#3*h/4 + 20
        mask_flood[0:search_top, 0:w] = 0
        mask_flood[search_bot:h, 0:w] = 0
        mask_ground[0:search_top, 0:w] = 0
        mask_ground[search_bot:h, 0:w] = 0
        M_flood = cv2.moments(mask_flood)
        M_ground = cv2.moments(mask_ground)
        

        global control
        # Determine the state of seeing the flood or not, and then publishing the state.        
        if M_flood['m00'] > 0 and M_ground['m00'] > 0:
            print('[drone17] see water and ground')
            state = 1
            self.status.publish(state)
        
        elif M_flood['m00'] > 0 and M_ground['m00'] <= 0 and M_ground['m10'] <= 0:
            print('[drone17] lost_ground')
            state = 1
            self.status.publish(state)
            
        elif M_flood['m00'] <= 0 and M_flood['m10'] <= 0 and M_ground['m00'] > 0:
            print('[drone17] lost_flood')
            state = 0
            self.status.publish(state)
        else:
            print('[Exception!!!]')
            state = 1
            self.status.publish(state)
        
        cv2.imshow("drone17", image)
        cv2.waitKey(3)
    

    def callback(self, data):
        # drone17 = data.pose[idx[16]]
        # drone17_vel = data.twist[idx[16]]
        drone17 = data.pose[int(config['index']['drone17'])]
        drone17_vel = data.twist[int(config['index']['drone17'])]
        d17_posX = drone17.position.x
        d17_posY = drone17.position.y
        d17_posZ = drone17.position.z
        d17_velX = drone17_vel.linear.x
        d17_velY = drone17_vel.linear.y

        global targets
        if len(targets) != 0:
            # print('[drone17] targets:', targets)
            target = targets[8]
            posX = target[0]
            posY = target[1]
            
            errX = posX - d17_posX
            errY = posY - d17_posY
    
            # print("[drone17] target: ", target, "Err: ", (errX, errY))
    
            ####################    
            ## POSITION BASED ##
            ####################
            h_vel_scale = float(config['velocity']['h_vel_scale'])       # The velocity in the horizontal plane
            v_vel_scale = float(config['velocity']['v_vel_scale'])       # The velocity in the vertical plane
            drone_height = float(config['position']['z_des']) 
            calc_odom = (Vector3(h_vel_scale * (d17_posX-posX), h_vel_scale * (d17_posY-posY), v_vel_scale*(d17_posZ-drone_height)))
            pub = self.cmd_vel_pub
            pub.publish(Twist(calc_odom, Vector3(0,0,0)))
            
        rospy.Rate(10).sleep
           

    
def listener():
    rospy.init_node('d_17')
    follower = Follower()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
# END ALL
