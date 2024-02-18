import math
import numpy as np
import rospy
import tf
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import TransformStamped, Point
from scipy import ndimage
import functools

# Grid Cell Size
GRID_CELL_SIZE_M = 0.08
map_msg = OccupancyGrid()
map_msg.header.frame_id = 'map'
resolution = GRID_CELL_SIZE_M
width = int(6.88 / GRID_CELL_SIZE_M) # 86
height = int(5 / GRID_CELL_SIZE_M) # 62
 # Map update rate (defaulted to 5 Hz)
rate = 5.0


class Mapping:
    def __init__(self):
        rospy.init_node("map")
        rospy.Subscriber('/map', OccupancyGrid, self.update_map)
        
        # Initialize occupancy grid message
        self.map_msg = map_msg
        
        # fill map_msg with the parameters
        self.map_msg.info.resolution = resolution
        self.map_msg.info.width = width
        map_msg.info.height = height
        map_msg.data = [0] * (width * height)
        
        # initialize grid with -1 (unknown)
        self.grid = np.ndarray((height, width), buffer=np.zeros((width, height), dtype=np.int),
                dtype=np.int)
        self.grid.fill(int(0))
        
        for i in range(width):
            self.grid[0][i] = 100
            self.grid[height-1][i] = 100

        for i in range(height):
            self.grid[i][0] = 100
            self.grid[i][width-1] = 100

        # set map origin [meters]
        map_msg.info.origin.position.x = 0.0
        map_msg.info.origin.position.y = 0.0
        
        # Publishers
        self.occ_pub = rospy.Publisher("/robot/map", OccupancyGrid, queue_size = 10)

        # TF Broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
    
        rospy.sleep(1.0)
        rospy.loginfo("Mapping node ready")

    def set_obstacles_neighbors(grid, x, y, size): # size in m
        grid[x, y] = int(100)
        if (size > (resolution)):
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

    def update_map(self, map_msg):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        self.map_msg = map_msg
            
    @staticmethod
    def neighbors_of_8(self, map_msg, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param map_msgF [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        neighbors = [(x-1, y-1), (x, y-1), (x+1, y-1), (x-1, y), (x+1, y), (x-1, y+1), (x, y+1), (x+1, y+1)]

        return neighbors

    @staticmethod
    def world_to_grid(self, x, y):
        
        # Grid cell indices for (x, y) point
        grid_x = int((x - self.map_msg.info.origin.position.x) / map_msg.info.resolution)
        grid_y = int((y - self.map_msg.info.origin.position.y) / map_msg.info.resolution)
        
        point = Point()
        point.x = grid_x
        point.y = grid_y
        point.z = 0.0
        return point

    @staticmethod
    def grid_to_world(grid_x, grid_y):
        # real-world coordinates for given grid cell
        x = (grid_x * map_msg.info.resolution) + map_msg.info.origin.position.x + (map_msg.info.resolution / 2.0)
        y = (grid_y * map_msg.info.resolution) + map_msg.info.origin.position.y + (map_msg.info.resolution / 2.0)

        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0
        return point

    @staticmethod
    def grid_to_index(map_msg, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        return y * map_msg.info.width + x
        
    @staticmethod
    def index_to_grid(map_msg, i):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell index.
        :return  ([int], [int]) The (x,y) coordinate .
        """
        x = i % map_msg.info.width
        y = i // map_msg.info.width
        return (y, x)

    @staticmethod
    def is_cell_walkable(self, map_msg, x, y):
        width  = map_msg.info.width
        height  = map_msg.info.height
        map_array = map_msg.data
        index = self.grid_to_index(map_msg,x,y)
        boundary_value = 0
        if(index <= len(map_array) - 1):
            boundary_value = map_array[self.grid_to_index(map_msg,x,y)]
        else:
            boundary_value = -1

        if(x < 0 or y < 0 or x > width or y > height or boundary_value == 100 or boundary_value < 0):
            return False

        return True
        
    def publish_grid_cells(self, cells, publisher):
        # create and publish grid cells message
        world_cells = [self.grid_to_world(self.msg, i[1], i[0]) for i in cells]

        resolution = self.mapdata.info.resolution
        msg_grid_cells = GridCells()
        msg_grid_cells.header.stamp = rospy.Time.now()
        msg_grid_cells.header.frame_id = "map"
        msg_grid_cells.cell_height = resolution
        msg_grid_cells.cell_width = resolution
        msg_grid_cells.cells = world_cells

        publisher.publish(msg_grid_cells)
        
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        loop_rate = rospy.Rate(rate)
        while not rospy.is_shutdown():

            # stamp current ros time to the message
            map_msg.header.stamp = rospy.Time.now()
            # build ros map message and publish
            for i in range(width*height):
                map_msg.data[i] = self.grid.flat[i]
            self.occ_pub.publish(map_msg)

            # Publish TF transform between "map" and "world" frame
            self.tf_broadcaster.sendTransform(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
                rospy.Time.now(),
                "map",
                "world"
            )

            loop_rate.sleep()

# main function
if __name__ == '__main__':
    Mapping.run()
    