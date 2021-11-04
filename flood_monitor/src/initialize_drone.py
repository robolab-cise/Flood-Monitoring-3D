#!/usr/bin/env python3
'''
Created on Jun 4, 2021

@author: Michael, Shaiful Nizam
@since: May 7, 2021
@organization: RoboLab (Ritsumeikan University, College of Information Science and Engineering)
'''

import rospy

from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
import sys

import math
import time
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator
import numpy as np
from shapely.geometry import Polygon
from shapely.geometry import Point
from geovoronoi import coords_to_points, points_to_coords, voronoi_regions_from_coords
from geovoronoi.plotting import plot_voronoi_polys_with_points_in_area
import mpl_scatter_density
from astropy.visualization import LogStretch
from astropy.visualization.mpl_normalize import ImageNormalize
from PIL import Image

"""
Get the position of the drone robot

Args:
    drone_name - The name of the drone the user wants to control (Object)
Returns:
    response.pose - (position: {x, y, z}, velocity: {x, y, z}). Extract the position and the
                    velocity of the robot.
"""
def get_pose(drone_name):
    sets = GetModelStateRequest()
    sets.model_name = drone_name
    response = call(sets)
    return response.pose


"""
This class is used to calculate and draw voronoi diagram.

Functions:
1. update(inner_pos) - to update the voronoi drawing
2. reshape_coords(coords) - reshape the coordinates
3. match_pair(poly_shapes, coords, new_centroids) - match the pair for the poly shapes
4. value_contribution(x_unit, y_unit, poly_shapes) - 
5. centroid_calculation(region_value, poly_shapes)
6. plot_Voronoi(ax, outer, poly_shapes, coords, new_coords)
7. plot_density(ax, x, y, norm)
8. diff_equation_x(y_list, t, e, omega)
9. diff_equation_y(y_list,t,e,omega)
10. cal_tra(new_coords,new_centroids)
11. compensate(pts, N_POINTS)
12. sampling(polyshapes, targets, standard_deviation=20, numbers=1000)
"""
class Initialize:
    def __init__(self):
        """
        Initialize the variable for the class Voronoi

        Variable:
            pub - Rospy's publisher to /controller that has message of Float64MultiArray
            pub_centroid - Rospy's publisher to /centroid that has message of Float64MultiArray
            ranges - Give the range for the voronoi diagram drawn on the left
            targets - Give the centroid of the density plot drawn on the right
            weights - Give the weight for each centroid for the area of importance
            standard_deviation - Give a standard_deviation for the random normal distribution
            numbers - Size of the random variable for the plot
            count - Count the number of drone updated
            
        Args:
            None
        Returns:
            None
        """
        self.pub = rospy.Publisher('/controller', Float64MultiArray, queue_size=10)
    
    
        
    def update(self, init_pos):
        """
        Updates the voronoi diagram

        Args:
            inner_pos - The inner robot position (Objects)
        Returns:
            None
        """
        data2send = Float64MultiArray(data = init_pos)
        self.pub.publish(data2send)
        


if __name__ == '__main__':

    rospy.init_node('view_node')

    call = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    rate = rospy.Rate(1)
    inner_drones = ['drone03','drone10', 'drone11', 'drone12', 'drone13','drone14', 'drone15', 'drone16', 'drone17'] 
    init_pos = [0, 0, 
                -13.33, -13.33, 
                13.33, -13.33, 
                13.33, 13.33, 
                -13.33, 0, 
                13.33, 0, 
                -13.33, 13.33, 
                0, 13.33, 
                0, -13.33]
    init = Initialize()
    count =0
    while count < 20:
        init.update(init_pos)
        rate.sleep()
        count = count + 1
        
