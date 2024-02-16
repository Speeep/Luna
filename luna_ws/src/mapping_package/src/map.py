import math
import numpy
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped


# Grid Cell Size
GRID_CELL_SIZE_M = 0.08

# initialize node
rospy.init_node('map')

# Initialize occupancy grid message
map_msg = OccupancyGrid()
map_msg.header.frame_id = 'map'
resolution = GRID_CELL_SIZE_M
width = int(6.88 / GRID_CELL_SIZE_M) # 86
height = int(5 / GRID_CELL_SIZE_M) # 62

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

# TF Broadcaster
tf_broadcaster = tf.TransformBroadcaster()

# main function
if __name__ == '__main__':
    
    # fill map_msg with the parameters
    map_msg.info.resolution = resolution
    map_msg.info.width = width
    map_msg.info.height = height
    map_msg.data = [0] * (width * height)

    # initialize grid with -1 (unknown)
    grid = numpy.ndarray((height, width), buffer=numpy.zeros((width, height), dtype=numpy.int),
             dtype=numpy.int)
    grid.fill(int(0))

    for i in range(width):
        grid[0][i] = 100
        grid[height-1][i] = 100

    for i in range(height):
        grid[i][0] = 100
        grid[i][width-1] = 100

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

        # Publish TF transform between "map" and "world" frame
        tf_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            rospy.Time.now(),
            "map",
            "world"
        )

        loop_rate.sleep()