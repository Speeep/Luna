import math
import numpy
import rospy
import tf
from nav_msgs.msg import OccupancyGrid


# Grid Cell Size
GRID_CELL_SIZE_M = 0.08

# initialize node
rospy.init_node('map_node')

# Initialize occupancy grid message
map_msg = OccupancyGrid()
map_msg.header.frame_id = 'map'
resolution = GRID_CELL_SIZE_M
width = 6.88 / GRID_CELL_SIZE_M
height = 5 / GRID_CELL_SIZE_M

# Map update rate (defaulted to 5 Hz)
rate = 5.0

def set_obstacles(grid, x, y, size): # size in m
    grid[x, y] = int(100)
    if (size > 0.1):
        if  grid[x+1, y] < int(1):
            grid[x+1, y] = int(100)
        if  grid[x-1, y] < int(1):
            grid[x-1, y] = int(100)
        if  grid[x-1, y-1] < int(1):
            grid[x-1, y-1] = int(100)
        if  grid[x, y-1] < int(1):
            grid[x, y-1] = int(100)
        if grid[x+1, y-1] < int(1):
            grid[x+1,y-1] = int(100)
        if  grid[x-1, y+1] < int(1):
            grid[x-1, y+1] = int(100)
        if  grid[x, y+1] < int(1):
            grid[x, y+1] = int(100)
        if  grid[x+1, y+1] < int(1):
            grid[x+1, y+1] = int(100)
        if grid[x+1, y-1] < int(1):
            grid[x+1,y-1] = int(100)
        if  grid[x-1, y+1] < int(1):
            grid[x-1, y+1] = int(100)
        if  grid[x, y+1] < int(1):
            grid[x, y+1] = int(100)
        if  grid[x+1, y+1] < int(1):
            grid[x+1, y+1] = int(100)

# Publishers
occ_pub = rospy.Publisher("/robot/map", OccupancyGrid, queue_size = 10)

# main function
if __name__ == '__main__':
    
    # fill map_msg with the parameters
    map_msg.info.resolution = resolution
    map_msg.info.width = width
    map_msg.info.height = height
    map_msg.data = range(width*height)

    # initialize grid with -1 (unknown)
    grid = numpy.ndarray((width, height), buffer=numpy.zeros((width, height), dtype=numpy.int),
             dtype=numpy.int)
    grid.fill(int(-1))

    # set map origin [meters]
    map_msg.info.origin.position.x = 0.0
    map_msg.info.origin.position.y = 0.0

    loop_rate = rospy.Rate(rate)
    while not rospy.is_shutdown():

        # stamp current ros time to the message
        map_msg.header.stamp = rospy.Time.now()
        # build ros map message and publish
        for i in range(width*height):
            map_msg.data[i] = grid.flat[i]
        occ_pub.publish(map_msg)
        loop_rate.sleep()