#!/usr/bin/env python3
'''
Created on May 7, 2021

@author: Michael, Shaiful Nizam
@since: May 30, 2021
@organization: RoboLab (Ritsumeikan University, College of Information Science and Engineering)
@status: Finished
@todo: Combine the voronoi.py and get_pose_2021.py so that it will calculate the outer boundary
'''

import rospy

from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from std_msgs.msg import Float64MultiArray, Int32
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
1. update(drone_pos) - to update the voronoi drawing
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

class Voronoi:
    def __init__(self):
        """
        Initialize the variables drone03, drone10, drone11, drone12, drone13, drone14, drone15, drone16 and drone17.
        The drone variables represents the status that drone see water [1] or not [0] - default value = 0.
        Including the following variables:
            pub - (Publisher) Rospy's publisher to topic "/controller" that has message of Float64MultiArray
            pub_centroid - (Publisher) Rospy's publisher to topic "/centroid" that has message of Float64MultiArray
            ranges - (list of set) Give the vertical and horizontal range for the voronoi diagram drawn (i.e dimension)
            targets - (list) Give the centroid of the density plot drawn on the right
            weights - (list) Give the weight for each centroid for the area of importance
            standard_deviation - (int) Give a standard_deviation for the random normal distribution
            numbers - (int) Size of the random variable for the plot.
            hasNotUpdated - (bool) for updating the flood status.
            
        Args:
            None
        Returns:
            None
        """
        self.pub = rospy.Publisher('/controller', Float64MultiArray, queue_size=10)
        self.pub_centroid = rospy.Publisher('/centroid', Float64MultiArray, queue_size=10)
        
        self.drone3_subsciber = rospy.Subscriber('drone03/flood_status', Int32, self.get_drone03_flood_status)
        self.drone10_subsciber = rospy.Subscriber('drone10/flood_status', Int32, self.get_drone10_flood_status)
        self.drone11_subsciber = rospy.Subscriber('drone11/flood_status', Int32, self.get_drone11_flood_status)
        self.drone12_subsciber = rospy.Subscriber('drone12/flood_status', Int32, self.get_drone12_flood_status)
        self.drone13_subsciber = rospy.Subscriber('drone13/flood_status', Int32, self.get_drone13_flood_status)
        self.drone14_subsciber = rospy.Subscriber('drone14/flood_status', Int32, self.get_drone14_flood_status)
        self.drone15_subsciber = rospy.Subscriber('drone15/flood_status', Int32, self.get_drone15_flood_status)
        self.drone16_subsciber = rospy.Subscriber('drone16/flood_status', Int32, self.get_drone16_flood_status)
        self.drone17_subsciber = rospy.Subscriber('drone17/flood_status', Int32, self.get_drone17_flood_status)
        
        self.drone03 = 0
        self.drone10 = 0
        self.drone11 = 0
        self.drone12 = 0
        self.drone13 = 0
        self.drone14 = 0
        self.drone15 = 0
        self.drone16 = 0
        self.drone17 = 0
        
        self.ranges = [(-20, 20), (-20, -20), (20, -20), (20, 20)]
        self.weights = []
        self.targets = []
        self.standard_deviation = 3.0
        self.numbers = 1000
        self.hasNotUpdated = True 
        self.plot_initialization()
        self.count = 0
   
    def get_drone03_flood_status(self, data):
        """
        To set the value of the variable, drone3.
        If it sees water, status = 1.
        If it does not see water, status = 0.

        Args:
            data - (Dictionary) flood status from the subscriber.
        Returns:
            None
        """
        self.drone03 = data.data
    
    def get_drone10_flood_status(self, data):
        """
        To set the value of the variable, drone10.
        If it sees water, status = 1.
        If it does not see water, status = 0.

        Args:
            data - (Dictionary) flood status from the subscriber.
        Returns:
            None
        """
        self.drone10 = data.data
    
    def get_drone11_flood_status(self, data):
        """
        To set the value of the variable, drone11.
        If it sees water, status = 1.
        If it does not see water, status = 0.

        Args:
            data - (Dictionary) flood status from the subscriber.
        Returns:
            None
        """
        self.drone11 = data.data
    
    def get_drone12_flood_status(self, data):
        """
        To set the value of the variable, drone12.
        If it sees water, status = 1.
        If it does not see water, status = 0.

        Args:
            data - (Dictionary) flood status from the subscriber.
        Returns:
            None
        """
        self.drone12 = data.data
        
    def get_drone13_flood_status(self, data):
        """
        To set the value of the variable, drone13.
        If it sees water, status = 1.
        If it does not see water, status = 0.

        Args:
            data - (Dictionary) flood status from the subscriber.
        Returns:
            None
        """
        self.drone13 = data.data
        
    def get_drone14_flood_status(self, data):
        """
        To set the value of the variable, drone14.
        If it sees water, status = 1.
        If it does not see water, status = 0.

        Args:
            data - (Dictionary) flood status from the subscriber.
        Returns:
            None
        """
        self.drone14 = data.data
    
    def get_drone15_flood_status(self, data):
        """
        To set the value of the variable, drone15.
        If it sees water, status = 1.
        If it does not see water, status = 0.

        Args:
            data - (Dictionary) flood status from the subscriber.
        Returns:
            None
        """
        self.drone15 = data.data
        
    def get_drone16_flood_status(self, data):
        """
        To set the value of the variable, drone16.
        If it sees water, status = 1.
        If it does not see water, status = 0.

        Args:
            data - (Dictionary) flood status from the subscriber.
        Returns:
            None
        """
        self.drone16 = data.data
        
    def get_drone17_flood_status(self, data):
        """
        To set the value of the variable, drone17.
        If it sees water, status = 1.
        If it does not see water, status = 0.

        Args:
            data - (Dictionary) flood status from the subscriber.
        Returns:
            None
        """
        self.drone17 = data.data
    
    def plot_initialization(self):
        """
        Initialize for both plot using the matplotlib.

        Args:
            None
        Returns:
            None
        """
        
        # Initialize the figure for matplotlib
        self.fig = plt.figure(figsize = [13, 5])
        self.fig.subplots_adjust(wspace = 0.3, hspace = 0, top = 0.9, bottom = 0.2)
        
        # Add the subplot for the scatter density
        self.ax1 = self.fig.add_subplot(121, projection = 'scatter_density')
        self.ax2 = self.fig.add_subplot(122, projection = 'scatter_density')
        
        # Set equal scaling (i.e., make circles circular) by changing dimensions of the plot box. 
        self.ax1.axis('scaled')
        self.ax2.axis('scaled')
        
        # Normalize the image
        self.norm = ImageNormalize(vmin = 0., vmax = 1000, stretch = LogStretch())
        self.major_locator = MultipleLocator(100)
        
    def update_status(self, drone_pos):
        """
        Update status of the flood.

        Args:
            drone_pos - (List) List of drone position.
        Returns:
            None
        """
        
        flood_state = []
        
        flood_state.extend([self.drone03, self.drone10, self.drone11, self.drone12, self.drone13, self.drone14, self.drone15, self.drone16, self.drone17])
        
        flood_state = np.array(flood_state)
        
        print('drone03:', self.drone03)
        print('drone10:', self.drone10)
        print('drone11:', self.drone11)
        print('drone12:', self.drone12)
        print('drone13:', self.drone13)
        print('drone14:', self.drone14)
        print('drone15:', self.drone15)
        print('drone16:', self.drone16)
        print('drone17:', self.drone17)
        print()
        print(flood_state)
        
        if np.all(flood_state == 0) and self.hasNotUpdated: # If all flood_state = 0. np.all will compare the condition to all elements in the list.
            print("Do nothing...")
            pass
        elif 1 in flood_state and not np.all(flood_state == 1) and self.hasNotUpdated: # If flood_state contains 1, but not all flood_state = 1.
            self.weights = []
            self.targets = []
            numOfDroneThatSeesWater = 1
            for index, state in enumerate(flood_state):
                if state == 1:
                    self.targets.append(list(drone_pos[index]))
                    numOfDroneThatSeesWater += 1
            
            for i in range(numOfDroneThatSeesWater):
                self.weights.append([1/numOfDroneThatSeesWater])
                
            print(self.targets)
            print(self.weights)
            self.update(drone_pos)

        elif np.all(flood_state == 1) or not self.hasNotUpdated: # If all flood_state = 1.
            if self.hasNotUpdated:
                self.weights = []
                self.targets = []
                for index, state in enumerate(flood_state):
                    if state == 1:
                        self.targets.append(list(drone_pos[index]))
                self.weights = [[1/9] for i in range(9)]
                self.hasNotUpdated = False 
            print(self.targets)
            print(self.weights)
            self.update(drone_pos)
        
    def update(self, drone_pos):
        """
        Updates the voronoi diagram

        Args:
            drone_pos - The inner robot position (Objects)
        Returns:
            None
        """
        
            ################Initialization#############
    
        outer_pos = self.ranges

        N = len(drone_pos)
        self.area_shape = Polygon(outer_pos)

        # centroid_poly = np.array(self.area_shape.centroid.xy).astype(np.float64)
        pts = [p for p in coords_to_points(drone_pos) if p.within(self.area_shape)]  # converts to shapely Point
        pts = self.compensate(pts, N)

        self.coords = points_to_coords(pts)   # convert back to simple NumPy coordinate array
        self.poly_shapes, pts, self.poly_to_pt_assignments = voronoi_regions_from_coords(self.coords, self.area_shape, accept_n_coord_duplicates = 0)
        
        self.x_unit, self.y_unit = self.sampling(self.poly_shapes, self.targets, self.standard_deviation, self.numbers)
        
        ############ Pair Points and Centroids ######
        self.region_value = self.value_contribution(self.x_unit, self.y_unit, self.poly_shapes)
        self.poly_centroids = self.centroid_calculation(self.region_value, self.poly_shapes)
        self.new_centroids = self.match_pair(self.poly_shapes, list(self.coords), list(self.poly_centroids))
        
        new_coords = self.cal_tra(self.coords, self.new_centroids)
        coords_to_points(new_coords)
        
        

        ########### Visualize ##########
        
        # Plot Voronoi
        self.j = 0
        self.plot_Voronoi(self.ax1, outer_pos, self.poly_shapes, self.coords, new_coords)
        
        # Plot Density Function
        self.plot_density(self.ax2, self.x_unit, self.y_unit, self.norm)
        plt.savefig('figure/FIG_' + str(self.count) + '.png')

        ########### Trajectory Calculation ##########

        new_coords = np.array(new_coords).reshape(-1)
        data2send = Float64MultiArray(data = new_coords)
        self.pub.publish(data2send)
        centroid2send = Float64MultiArray(data = self.new_centroids)
        self.pub_centroid.publish(centroid2send)
        

        plt.pause(0.000001)
        self.ax1.clear()
        self.ax2.clear()
        self.count += 1

    def match_pair(self, poly_shapes, coords, centroids): # sort order of centroid list to find pair of each region and its centroid
        """
        Matching the pair of each drone to the centroids

        Args:
            poly_shapes - (Points) The polygon shapes of the boundary
            coords - (Points) The coordinates of each point in the boundary
            centroids - (Array) The centroids of each drone
        Returns:
            sorted_centroids - (Array) The sorted centroids from the coordinated
        """
        sorted_centroids = []
        points = coords_to_points(coords)
        for p in points:
            for j, poly in enumerate(poly_shapes):
                if p.within(poly): 
                    pair = centroids[j]
                    sorted_centroids.append(pair)
        return sorted_centroids

    def value_contribution(self, x_unit, y_unit, poly_shapes):
        """
        Give a value contribution to the x and y coord according to the poly shapes

        Args:
            x_unit - 
            y_unit - 
            poly_shapes - 
        Returns:
            region_value - 
        """
        point_value = np.vstack((np.array(x_unit), np.array(y_unit))).T # make pairs of samples
        poly_nums = len(poly_shapes)
        region_value =[[] for _ in range(poly_nums)] #this stores coordinates of samples in a region
        for p in point_value:
            for j, poly in enumerate(poly_shapes):
                point = Point(p) # convert sample coordinate to Point
                if point.within(poly): # if the sample locates in the region polyshapes[j] then append this p to region_value[j]
                    region_value[j].append(p)
        return np.array(region_value)

    def centroid_calculation(self, region_value, poly_shapes):
        """
        Updates the voronoi diagram

        Args:
            drone_pos - The inner robot position (Objects)
        Returns:
            None
        """
        sum_value = []
        for i in range(len(poly_shapes)):
            init = [0,0]
            for j in region_value[i]:
                init += j
            sum_value.append(init)
            
        poly_centroids = []
        for i in range(len(poly_shapes)):
            poly_size = len(region_value[i])
            if poly_size == 0: # If number of sample in a region is 0 then use its centroid as the next target
                poly_centroids.append(poly_shapes[i].centroid.coords[0])
            else: # next target is the CM of samples in a region
                poly_dense_x = sum_value[i][0]/poly_size
                poly_dense_y = sum_value[i][1]/poly_size
                poly_centroids.append([poly_dense_x,poly_dense_y])
        return poly_centroids
        
    def plot_Voronoi(self, ax, outer, poly_shapes, coords, new_coords): # plot voronoi diagram
        """
        Graph on the left of the plot showing the Voronoi distribution among
        the drone.

        Args:
            ax - (plt) To initialize the axis plot for the matplotlib
            outer - 
            poly_shapes - 
            coords - 
            new_coords - 
        Returns:
            None
        """
        self.area_shape = Polygon(outer)
        for centroid in self.new_centroids:
            c1 = centroid
            ax.plot(c1[0],c1[1], 'rs', markersize = 8) # plot inner drones as red circle
        
        for ranges in self.ranges:
            r1 = ranges
            ax.plot(r1[0], r1[1], 'ys', markersize=8)
        
        
        font = {'size':20}
        ax.set_xlim([-20, 20]) # set range of x-axis
        ax.set_ylim([-20, 20]) # set range of x-axis
        ax.xaxis.set_major_locator(self.major_locator) # set grid
        ax.yaxis.set_major_locator(self.major_locator)
        ax.set_xlabel('x(m)', font, labelpad=10) # set x-label
        ax.set_ylabel('y(m)', font, labelpad=10)
        ax.tick_params(labelsize=18)
        plot_voronoi_polys_with_points_in_area(ax, self.area_shape, poly_shapes, coords,
                                               self.poly_to_pt_assignments, points_color='black', # plot centroid as black square
                                               points_markersize=40, voronoi_and_points_cmap=None,
                                               plot_voronoi_opts={'alpha': 0.5})

    def plot_density(self, ax, x, y, norm):
        """
        Graph on the right of the plot showing the density of each drone
        that sees water. Moreover, it shows the area coverage among the
        environment.

        Args:
            ax - 
            x - 
            y - 
            norm - 
        Returns:
            None
        """
        size = 40
        shift_by = size /2
        # plot samples as normal scatter plot (you can see each group colored different colors)
        color = ['+C0', '+C1', '+C2', '+C3', '+C4', '+C5', '+C6', '+C7', '+C8']
        for i in range(len(self.targets)):
            if i==0:
                ax.plot(x[:int(self.numbers * self.weights[i][0])] - min(self.ranges[:])[0] - shift_by, y[:int(self.numbers * self.weights[i][0])] - min(self.ranges[:])[1] - shift_by, color[i], alpha=0.5)
                prev = int(self.numbers * self.weights[i][0])
            else:
                ax.plot(x[prev:prev + int(self.numbers * self.weights[i][0])] - min(self.ranges[:])[0] - shift_by, y[prev:prev + int(self.numbers * self.weights[i][0])]-min(self.ranges[:])[1] - shift_by, color[i], alpha=0.5)
                prev += int(self.numbers * self.weights[i][0])
        
        
        img = (Image.open('/home/robolab/catkin_ws/src/flood_monitor/scripts/2.png')).resize((size, size))  #(186,246)
        ax.imshow(img, extent=(-shift_by, shift_by, -shift_by, shift_by), origin="lower", alpha=0.5)
        font = {'size':20}
        # ax.set_xlim([0,max(self.ranges[:])[0]*2])
        # ax.set_ylim([0,max(self.ranges[:])[1]*2])
        ax.set_xlim([-20, 20])
        ax.set_ylim([-20, 20])
        ax.xaxis.set_major_locator(self.major_locator)
        ax.yaxis.set_major_locator(self.major_locator)
        ax.set_xlabel('x(m)',font, labelpad=10)
        ax.set_ylabel('y(m)',font, labelpad=10)
        ax.tick_params(labelsize=18)

    def diff_equation_x(self, y_list, t, e, omega):
        """
        Calculate the differential equation of x for calculating the
        trajectory of the drone movement

        Args:
            y_list - 
            t - 
            e - 
            omega - 
        Returns:
            result - 
        """
        ki = 200
        sum_phi = y_list[1] + e
        coef = 0
        for i in range(2,len(y_list)):
            if i%2 == 0:
                coef = int(i/2)
                sum_phi += (y_list[i] + e*math.cos(coef*omega*t)) * math.cos(coef*omega*t)
            else:
                coef = int((i-1)/2)
                sum_phi += (y_list[i] + e*math.sin(coef*omega*t)) * math.sin(coef*omega*t)
        
        result = []
        result.append(-ki*e-sum_phi + 20*math.cos(math.pi*t))
        result.append(-e)
        for i in range(2,len(y_list)):
            if i%2 == 0:
                coef = int(i/2)
                result.append(coef*e*omega*math.sin(coef*omega*t) + ki*e*math.cos(coef*omega*t))
            else:
                coef = int((i-1)/2)
                result.append((-e)*coef*omega*math.cos(coef*omega*t) + ki*e*math.sin(coef*omega*t))
        return np.array(result)
                         
    def diff_equation_y(self, y_list, t, e, omega):
        """
        Calculate the differential equation of y for calculating the
        trajectory of the drone movement

        Args:
            y_list - 
            t - 
            e - 
            omega - 
        Returns:
            result - 
        """
        ki = 200
        sum_fu = y_list[1] + e
        coef = 0
        for i in range(2,len(y_list)):
            if i%2 == 0:
                coef = int(i/2)
                sum_fu += (y_list[i] + e*math.cos(coef*omega*t)) * math.cos(coef*omega*t)
            else:
                coef = int((i-1)/2)
                sum_fu += (y_list[i] + e*math.sin(coef*omega*t)) * math.sin(coef*omega*t)
        result = []
        result.append(-ki*e-sum_fu + 20*math.sin(math.pi*t))
        result.append(-e)
        for i in range(2,len(y_list)):
            if i%2 == 0:
                coef = int(i/2)
                result.append(coef*e*omega*math.sin(coef*omega*t) + ki*e*math.cos(coef*omega*t))
            else:
                coef = int((i-1)/2)
                result.append((-e)*coef*omega*math.cos(coef*omega*t) + ki*e*math.sin(coef*omega*t))
        return np.array(result)

    def cal_tra(self, new_coords,new_centroids):
        """
        To calculate the trajectory of the drone to the centroid
        According to the Voronoi.

        Args:
            new_coords - 
            new_centroids - 
        Returns:
            point_lists - 
        """
        T=2
        t=np.linspace(0,T,num=150) # in 2time step update 50 times  
        omega = math.pi*2/T
        point_lists = []
        for i in range(len(new_coords)):
            y_list_x = [new_coords[i][0],0,0,0,0,0,0,0,0,0,0]
            y_list_y = [new_coords[i][1],0,0,0,0,0,0,0,0,0,0]
            result_x = odeint(self.diff_equation_x, y_list_x, t, args=(new_coords[i][0]-new_centroids[i][0],omega))
            result_y = odeint(self.diff_equation_y, y_list_y, t, args=(new_coords[i][1]-new_centroids[i][1],omega))
            result_xt = result_x[:,0]
            result_yt = result_y[:,0]
            new_result = np.vstack((np.array(result_xt), np.array(result_yt))).T
            point_lists.append(list(new_result)[1])
        return point_lists

    def compensate(self, pts, N_POINTS):
        """
        To compensate the position of the drone and the Voronoi algorithm

        Args:
            pts - 
            N_POINTS - 
        Returns:
            pts - 
        """
        while len(pts)<N_POINTS:
            drone_pos = points_to_coords(pts)
            randx = np.random.uniform(min(self.ranges[:])[0], max(self.ranges[:])[0], N_POINTS-len(pts))
            randy = np.random.uniform(min(self.ranges[:])[1], max(self.ranges[:])[1], N_POINTS-len(pts))
            compensated = np.vstack((randx, randy)).T
            if len(drone_pos):
                drone_pos = np.append(drone_pos, compensated, axis=0)
            else:
                drone_pos = compensated
                
                
            pts = [p for p in coords_to_points(drone_pos) if p.within(self.area_shape)]  # converts to shapely Point
            print('[get_pose_2021] %d of %d drone"s pos is available' % (len(pts), N_POINTS))
        return pts

    # Taking sample for the density and centroid using normal density distribution.
    def sampling(self, polyshapes, targets, standard_deviation=20, numbers=1000):
        """
        Sampling according to gaussian distribution with x, y and sigma(x),
        sigma(y) the same value. Therefore, there is no multi-dimensional distribution.

        Args:
            polyshapes - (Points) The polygon shape of the boundary
            targets - (List of List) The target centroid of the gaussian distribution
            standard_deviation - (int) The standard deviation for the gaussian distribution
            numbers - The number of point that will be deployed for the distribution
        Returns:
            x_unit - (Array) Give the gaussian sample for the x coordinate 
            y_unit - (Array) Give the gaussian sample for the y coordinate
        """
        ########### Density Distribution and Centroids Calculation #############
        # frames=np.linspace(0,4*np.pi,self.N, endpoint=True) # for circumference circle movement
        
        if len(targets)==1:
            x_unit = np.random.normal(targets[0][0], standard_deviation, int(numbers * self.weights[0][0]))
            y_unit = np.random.normal(targets[0][1], standard_deviation, int(numbers * self.weights[0][0]))
        elif len(targets)>1:
            x_unit = np.array([])
            y_unit = np.array([])
            for i in range(len(targets)):
                x_unit = np.concatenate((x_unit,np.random.normal(targets[i][0], standard_deviation, int(numbers * self.weights[i][0]))))
                y_unit = np.concatenate((y_unit,np.random.normal(targets[i][1], standard_deviation, int(numbers * self.weights[i][0]))))

        return x_unit, y_unit



        
        
        

if __name__ == '__main__':

    rospy.init_node('view_node')

    call = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    rate = rospy.Rate(1)
    inner_drones = ['drone03','drone10', 'drone11', 'drone12', 'drone13','drone14', 'drone15', 'drone16', 'drone17']
    voronoi = Voronoi()
    while not rospy.is_shutdown():
        pos_x = []
        pos_y = []
        for inner_drone in inner_drones:
            # Get position of drones
            pose = get_pose(inner_drone)
            x = pose.position.x
            y = pose.position.y
            # print('[get_pose_2021] inner:', inner_drone, ', x:', x, ', y:', y)
            pos_x.append(x)
            pos_y.append(y)
            
        drone_pos = np.vstack((np.array(pos_x).astype(np.float64), np.array(pos_y).astype(np.float64))).T
        # Update the position of both the outer and inner drones.
        voronoi.update_status(drone_pos)
        rate.sleep()
        
        
