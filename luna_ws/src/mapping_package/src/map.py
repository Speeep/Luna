import math
import numpy
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, PointStamped
from std_msgs.msg import Float32MultiArray, Bool

class Map_Node:
    # Grid Cell Size
    GRID_CELL_SIZE_M = 0.08

    # Initialize occupancy grid message
    map_msg = OccupancyGrid()

    width = int(6.88 / GRID_CELL_SIZE_M) # 86
    height = int(5 / GRID_CELL_SIZE_M) # 62

    # initialize grid with -1 (unknown)
    grid = numpy.ndarray((height, width), buffer=numpy.zeros((width, height), dtype=numpy.int),dtype=numpy.int)

    # TF Broadcaster
    tf_broadcaster = tf.TransformBroadcaster()
    
    def __init__(self):
        # initialize node
        rospy.init_node('map')

        self.grid.fill(int(0))

        # fill map_msg with the parameters
        self.map_msg.info.resolution = self.GRID_CELL_SIZE_M
        self.map_msg.info.width = self.width
        self.map_msg.info.height = self.height
        self.map_msg.data = [0] * (self.width * self.height)
        self.map_msg.header.frame_id = 'map'
        # set map origin [meters]
        self.map_msg.info.origin.position.x = 0.0
        self.map_msg.info.origin.position.y = 0.0

        # Publishers
        self.occ_pub = rospy.Publisher("/robot/map", OccupancyGrid, queue_size = 10)
        self.change_pub = rospy.Publisher("robot/map_change", Bool, queue_size = 3)

        # Subscribers
        self.obstacle_sub = rospy.Subscriber('/map/obstacle', Float32MultiArray, self.set_obstacle_cb)
        self.click_sub = rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_cb)

        for i in range(self.width):
            self.grid[0][i] = 100
            self.grid[self.height-1][i] = 100

        for i in range(self.height):
            self.grid[i][0] = 100
            self.grid[i][self.width-1] = 100

    def set_obstacle_cb(self, obstacle_msg):
        data = obstacle_msg.data
        x = data[0]
        y = data[1]
        radius = data[2]
        grid_x, grid_y = self.world_to_grid(x, y)
        self.set_obstacle(x=grid_x, y=grid_y, radius=radius)

    def clicked_point_cb(self,click_msg):
        x = click_msg.point.x
        y = click_msg.point.y
        grid_x, grid_y = self.world_to_grid(x, y)
        self.set_obstacle(x=grid_x, y=grid_y, radius=0.0)

    def set_obstacle(self, x, y, radius): # size in m
        # x = int(x)
        # y = int(y)
        # grid[x, y] = 100
        rad = math.ceil(radius // self.GRID_CELL_SIZE_M)
        padding = 4  # Padding thickness

        #this is a flag to tell if the drivable space is changed
        new_obstacle = False
        
        # Loop through the cells within the obstacle radius around the specified coordinate
        for i in range(x - rad - padding, x + rad + padding + 1):
            for j in range(y - rad - padding, y + rad + padding + 1):

                # Calculate the distance between the current cell and the center (x, y)
                distance = math.sqrt((i - x)**2 + (j - y)**2)

                # Check if the current cell is within the radius of the circle
                if distance <= rad:

                    # Check if the current cell is within the grid boundaries
                    if 0 <= i < len(self.grid) and 0 <= j < len(self.grid[0]):

                        # If the cell is within the circle, set its value to 100
                        if(self.grid[i, j] < 10):
                            new_obstacle = True
                        
                        self.grid[i, j] = 100

                # If the cell is within the padding area (outside the circle but within the padding thickness),
                # set its value to 1
                elif distance <= rad + padding:
                    if self.grid[i, j] < 10:
                        self.grid[i, j] = 10
                        new_obstacle = True
        
        self.change_pub.publish(new_obstacle)
        
    def world_to_grid(self, x, y):

        # Grid cell indices for (x, y) point
        grid_x = int(x / self.GRID_CELL_SIZE_M)
        grid_y = int(y / self.GRID_CELL_SIZE_M)
        return (grid_y, grid_x)
    
    def publish_map(self):
        
        # stamp current ros time to the message
        self.map_msg.header.stamp = rospy.Time.now()
        # build ros map message and publish
        for i in range(self.width*self.height):
            self.map_msg.data[i] = self.grid.flat[i]
        self.occ_pub.publish(self.map_msg)

        # Publish TF transform between "map" and "world" frame
        self.tf_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            rospy.Time.now(),
            "map",
            "world"
        )
        
    

# main function
if __name__ == '__main__':
    # Map update rate (defaulted to 5 Hz)
    rate = 5.0

    mapNode = Map_Node()
    loop_rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        loop_rate.sleep()
        mapNode.publish_map()

