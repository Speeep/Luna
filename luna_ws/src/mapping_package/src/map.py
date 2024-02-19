import math
import numpy
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, PointStamped
from std_msgs.msg import Float32MultiArray

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

# initialize grid with -1 (unknown)
grid = numpy.ndarray((height, width), buffer=numpy.zeros((width, height), dtype=numpy.int),dtype=numpy.int)
grid.fill(int(0))

def world_to_grid(x, y):

    # Grid cell indices for (x, y) point
    grid_x = int(x / resolution)
    grid_y = int(y / resolution)
    return (grid_y, grid_x)

def set_obstacles(grid, x, y, radius): # size in m
    x = int(x)
    y = int(y)
    grid[x, y] = 100
    rad = int(radius // GRID_CELL_SIZE_M)
    padding = 3  # Padding thickness
    
    # Loop through the cells within the obstacle radius around the specified coordinate
    for i in range(x - rad - padding, x + rad + padding + 1):
        for j in range(y - rad - padding, y + rad + padding + 1):
            # Calculate the distance between the current cell and the center (x, y)
            distance = math.sqrt((i - x)**2 + (j - y)**2)
            # Check if the current cell is within the radius of the circle
            if distance <= rad:
                # Check if the current cell is within the grid boundaries
                if 0 <= i < len(grid) and 0 <= j < len(grid[0]):
                    # If the cell is within the circle, set its value to 100
                    grid[i, j] = 100
            # If the cell is within the padding area (outside the circle but within the padding thickness),
            # set its value to 1
            elif distance <= rad + padding:
                if grid[i, j] < 10:
                    grid[i, j] = 10

def set_obstacles_cb(obstacle_msg):
    data = obstacle_msg.data
    x = data[0]
    y = data[1]
    radius = data[2]
    set_obstacles(grid=grid, x=x, y=y, radius=radius)

def clicked_point_cb(click_msg):
    x = click_msg.point.x
    y = click_msg.point.y
    grid_x, grid_y = world_to_grid(x, y)
    set_obstacles(grid=grid, x=grid_x, y=grid_y, radius=0.0)

# Publishers
occ_pub = rospy.Publisher("/robot/map", OccupancyGrid, queue_size = 10)
rospy.Subscriber('/map/obstacle', Float32MultiArray, set_obstacles_cb)
rospy.Subscriber('/clicked_point', PointStamped, clicked_point_cb)

# TF Broadcaster
tf_broadcaster = tf.TransformBroadcaster()

# main function
if __name__ == '__main__':
    
    # fill map_msg with the parameters
    map_msg.info.resolution = resolution
    map_msg.info.width = width
    map_msg.info.height = height
    map_msg.data = [0] * (width * height)

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