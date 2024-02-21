
import numpy as np
import math
import rospy
from priority_queue import PriorityQueue
from nav_msgs.msg import OccupancyGrid, GridCells

# Define robot parameters
wheel_base = 0.58  # distance between the wheels
wheel_radius = 0.13  # radius of the wheels
astar_frontier = rospy.Publisher('/mapping_package/astar_frontier', GridCells, queue_size=10)
astar_expanded = rospy.Publisher('/mapping_package/astar_expanded', GridCells, queue_size=10)

# Function to determine the Instantaneous Center of Curvature (ICC)
def calculate_icc(position0_x, position0_y, orientation_0, position1_x, position1_y, orientation_1):
    
    if ((position0_x == position1_x) | (position0_y == position1_y)):
        return np.inf, np.inf
    
    # Calculate the centers of curvature and radii for each arc
    icc0_x = position0_x + np.cos(orientation_0 + np.pi / 2) * wheel_base / 2
    icc0_y = position0_y + np.sin(orientation_0 + np.pi / 2) * wheel_base / 2
    radius0 = np.sqrt((position1_x - icc0_x)**2 + (position1_y - icc0_y)**2)
    
    icc1_x = position1_x + np.cos(orientation_1 - np.pi / 2) * wheel_base / 2
    icc1_y = position1_y + np.sin(orientation_1 - np.pi / 2) * wheel_base / 2
    radius1 = np.sqrt((position0_x - icc1_x)**2 + (position0_y - icc1_y)**2)
    
    return icc0_x, icc0_y, radius0, icc1_x, icc1_y, radius1

# Function to calculate the desired path between two poses
def calculate_desired_path(position0_x, position0_y, orientation_0, position1_x, position1_y, orientation_1):
    # Calculate the center of curvature
    icc0_x, icc0_y, radius0, icc1_x, icc1_y, radius1  = calculate_icc(position0_x, position0_y, orientation_0, position1_x, position1_y, orientation_1)
    

    icc_x = icc0_x
    icc_y = icc0_y
    radius = radius0
    
    # Generate points along the circular arc
    theta = np.linspace(orientation_0, orientation_1, 100)
    path_x = np.round(icc_x + radius * np.cos(theta + np.pi / 2))
    path_y = np.round(icc_y + radius * np.sin(theta + np.pi / 2))
    path = np.column_stack((path_x, path_y))

        
    # Filter out repeated points
    path = np.unique(path, axis=0)
    path = np.absolute(path)
    
    return path

# Function to calculate the curvature of the path
def calculate_curvature(path, position0_x, position0_y, orientation_0, position1_x, position1_y, orientation_1):
    # For a circular arc path, the curvature is constant
    icc_x, icc_y = calculate_icc(position0_x, position0_y, orientation_0, position1_x, position1_y, orientation_1)
    radius = np.sqrt((position1_x - icc_x)**2 + (position1_y - icc_y)**2)
    curvature = 1.0 / radius
    return curvature

# Function to control the robot's motion along the path
def follow_path(position0_x, position0_y, orientation_0, position1_x, position1_y, orientation_1):
    # Calculate the desired path
    path = calculate_desired_path(position0_x, position0_y, orientation_0, position1_x, position1_y, orientation_1)
    return path
    
    # Calculate the curvature of the path
    # curvature = calculate_curvature(path, pose0, pose1)
        
        
        
        
        
        
        
        
        

# main function
if __name__ == '__main__':
    #pose0 = Pose()
    #pose0.position.x = 1
    #pose0.position.y = 1
    #pose0.orientation = np.pi/2
    #pose1 = Pose()
    #pose1.position.x = 3
    #pose1.position.y = 3
    #pose1.orientation = np.pi
    #print(follow_path(pose0, pose1))
    
    position_0x = 1
    position_0y = 1
    orientation_0 = np.pi/2
    position_1x = 3
    position_1y = 3
    orientation_1 = np.pi
    print(follow_path(position_0x, position_0y, orientation_0, position_1x, position_1y, orientation_1))
    
    