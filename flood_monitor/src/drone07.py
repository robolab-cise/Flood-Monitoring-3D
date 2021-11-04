#!/usr/bin/env python3

import os
import sys
import tempfile
import csv
import tf
import rospy
import math
import message_filters
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import Twist, Vector3
import numpy as np

import cv2, cv_bridge
from sensor_msgs.msg import Image, CameraInfo

import configparser
import time

control = [0,0,0]
config = configparser.ConfigParser()
config.read('config.ini')
MAX_DIST = int(config['distance']['max'])
        
class Follower:
    def __init__(self):
       
        self.bridge = cv_bridge.CvBridge()
        image = rospy.Subscriber('drone07/downward_cam/camera/image', Image, self.image_callback)
        self.data = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.cmd_vel_pub = rospy.Publisher('drone07/cmd_vel',Twist, queue_size=10)
        self.twist = Twist()
        
    
    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8') #bgr8
        image = image[:, 30:image.shape[1]-30, :]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  
        lower_flood = np.array([85, 40, 215]) # np.array([100, 100, 0]) #yellow #np.array([0, 100, 100]) # blue np.array([90, 115, 186]) 85, 40, 215
        upper_flood = np.array([100,60,235]) # np.array([255, 255, 120]) #yellow #np.array([100, 255, 255]) # blue  np.array([110,240,255])  100,60,255
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
        
        if M_flood['m00'] > 0 and M_ground['m00'] > 0:
            #print('fine')
            cx_flood = int(M_flood['m10']/M_flood['m00'])
            cy_flood = int(M_flood['m01']/M_flood['m00'])
            cv2.circle(image, (cx_flood, cy_flood), 10, (0,0,255), -1)
            cx_ground = int(M_ground['m10']/M_ground['m00'])
            cy_ground = int(M_ground['m01']/M_ground['m00'])
            cv2.circle(image, (cx_ground, cy_ground), 10, (255,0,0), -1)

            cx_middle = min(cx_flood, cx_ground) + int((max(cx_flood, cx_ground)-min(cx_flood, cx_ground))/2)
            cy_middle = min(cy_flood, cy_ground) + int((max(cy_flood, cy_ground)-min(cy_flood, cy_ground))/2)
            cv2.circle(image, (cx_middle, cy_middle), 10, (0,0,0), -1)
            cv2.line(image, (cx_flood, cy_flood), (cx_ground, cy_ground), (0,255,0), 1)

           
            
            # BEGIN CONTROL
            err_x = cx_middle - w/2
            err_y = cy_middle - h/2
            
            #print("err_x: ", err_x, "err_y: ", err_y)
            
            velX = -float(err_y)/300.0 + 1.0
            velY = -float(err_x)/70.0
            velZ = 0 #-1.1
            self.twist.angular.z = -float(math.atan2((cy_flood - cy_ground),(cx_flood - cx_ground)))*1.8

            global control
            control = [velX, velY, velZ]
            
            self.cmd_vel_pub.publish(self.twist)
        
        elif M_flood['m00'] > 0 and M_ground['m00'] <= 0 and M_ground['m10'] <= 0:
            print('[drone07] lost_ground')
            cx_flood = int(M_flood['m10']/M_flood['m00'])
            cy_flood = int(M_flood['m01']/M_flood['m00'])
            cv2.circle(image, (cx_flood, cy_flood), 10, (0,0,255), -1)
            velX = 0.0
            velY = 0.0
            velZ = 2.8
            control = [velX, velY, velZ]
            #global control
            
        elif M_flood['m00'] <= 0 and M_flood['m10'] <= 0 and M_ground['m00'] > 0:
            print('[drone07] lost_flood')
            cx_ground = int(M_ground['m10']/M_ground['m00'])
            cy_ground = int(M_ground['m01']/M_ground['m00'])
            cv2.circle(image, (cx_ground, cy_ground), 10, (255,0,0), -1)
            velX = 0.0
            velY = 0.0
            velZ = 2.8
            control = [velX, velY, velZ]
            #global control
            
        else:
            velX = 0.0
            velY = 0.0
            velZ = 0.0
            control = [velX, velY, velZ]
            #global control
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            print('[drone07] else')
        
        
        
        # cv2.imshow("d7", image)
        # cv2.waitKey(3)
    

    def callback(self, data):

        drone06 = data.pose[int(config['index']['drone06'])]
        drone06_vel = data.twist[int(config['index']['drone06'])]
        d06_posX = drone06.position.x
        d06_posY = drone06.position.y
        d06_posZ = drone06.position.z
        d06_velX = drone06_vel.linear.x
        d06_velY = drone06_vel.linear.y

        drone07 = data.pose[int(config['index']['drone07'])]
        drone07_vel = data.twist[int(config['index']['drone07'])]
        d07_posX = drone07.position.x
        d07_posY = drone07.position.y
        d07_posZ = drone07.position.z
        d07_velX = drone07_vel.linear.x
        d07_velY = drone07_vel.linear.y

        drone08 = data.pose[int(config['index']['drone08'])]
        drone08_vel = data.twist[int(config['index']['drone08'])]
        d08_posX = drone08.position.x
        d08_posY = drone08.position.y
        d08_posZ = drone08.position.z
        d08_velX = drone08_vel.linear.x
        d08_velY = drone08_vel.linear.y


        #print(control,d04_posZ)
        ####################    
        ## POSITION BASED ##
        ####################
        global control
        velX, velY, velZ = control
        #print("[drone07] ini velx: ", velX)
        repul = 0.0
        pub = self.cmd_vel_pub
        curr_time = time.time() - start
        dist_87 = float(math.sqrt((d08_posX-d07_posX)**2 + (d08_posY-d07_posY)**2))
        dist_76 = float(math.sqrt((d06_posX-d07_posX)**2 + (d06_posY-d07_posY)**2))

        if d07_posZ < 2 and dist_87 > 15:            
            calc_odom = (Vector3(-0.1*(d07_velX-d08_posX),-0.1*(d07_posY-d08_posY),-0.05*(d07_posZ-3)))
            print("[drone07] ATR APPEAR!!")
        else:

            if d07_posZ >= 2 and curr_time > 10:
                        
                if dist_87 <= MAX_DIST and d08_posZ >= 2:
                    #print("[drone07] d87 REPUL APPEAR!!", dist_87)
                    b87 = 1.8*float((20.0/(MAX_DIST**7))*(dist_87**7) - (70.0/(MAX_DIST**6))*(dist_87**6) + (84.0/(MAX_DIST**5))*(dist_87**5) - (35.0/(MAX_DIST**4))*(dist_87**4) +1)
                else:
                    b87 = 0.0
                        
                if dist_76 <= MAX_DIST and d06_posZ >= 2:
                    #print("[drone07] d76 REPUL APPEAR!!", dist_76)
                    b76 = -0.4*float((20.0/(MAX_DIST**7))*(dist_76**7) - (70.0/(MAX_DIST**6))*(dist_76**6) + (84.0/(MAX_DIST**5))*(dist_76**5) - (35.0/(MAX_DIST**4))*(dist_76**4) +1)

                else:
                    b76 = 0.0
                            
                        

                repul = round((b76+ b87), 2)
                        
            else:
                repul = 0.0

            new_velX = velX - repul

            calc_odom = (Vector3(new_velX,velY,-0.8*(d07_posZ-3)))

        pub.publish(Twist(calc_odom, Vector3(0,0,0)))
        #print('[drone07] position control now')
        rospy.Rate(1).sleep
           

    
def listener():
    rospy.init_node('d_7')
    follower = Follower()
    rospy.spin()

if __name__ == '__main__':
    start = time.time()
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
# END ALL
