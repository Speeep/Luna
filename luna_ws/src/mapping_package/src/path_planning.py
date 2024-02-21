import numpy as np
import math
import rospy
from priority_queue import PriorityQueue
from std_msgs.msg import Bool
from nav_msgs.srv import GetPlan
from nav_msgs.msg import GridCells, Path, OccupancyGrid
from geometry_msgs.msg import Point, Pose, PoseStamped
from map import Mapping

class path_planning:
    def __init__(self):
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        rospy.Service("plan_path", GetPlan, self.plan_path)
        
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.astar_frontier = rospy.Publisher('/path_planner/astar_frontier', GridCells, queue_size=10)
        self.astar_expanded = rospy.Publisher('/path_planner/astar_expanded', GridCells, queue_size=10)
        self.path_test = rospy.Publisher('/path_planner/path_test', Path, queue_size=10)
        self.map_msg = Mapping()
        
        self.invalid_path = rospy.Publisher('/path_planner/invalid', Bool, queue_size=10)
        
        rospy.Subscriber('/map', OccupancyGrid, self.update_map)
        
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")
        
    
    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        return math.sqrt(math.pow(x2-x1, 2) + math.pow(y2-y1, 2))
    
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
        grid_x = int((x - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        grid_y = int((y - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution)
        
        point = Point()
        point.x = grid_x
        point.y = grid_y
        point.z = 0.0
        return point

    @staticmethod
    def grid_to_world(self, grid_x, grid_y):
        # real-world coordinates for given grid cell
        x = (grid_x * self.map_msg.info.resolution) + self.map_msg.info.origin.position.x + (self.map_msg.info.resolution / 2.0)
        y = (grid_y * self.map_msg.info.resolution) + self.map_msg.info.origin.position.y + (self.map_msg.info.resolution / 2.0)

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
    
    def a_star(self, map_msg, start, goal):
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        map_array = map_msg.data
        print(start)
        frontier = PriorityQueue()
        frontier.put(start,0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None 
        cost_so_far[start] = 0
        expanded_cells = []

        msg_grid_cells = GridCells()
        msg_grid_cells.header.frame_id = "map"
        msg_grid_cells.cell_height =  map_msg.info.resolution
        msg_grid_cells.cell_width =  map_msg.info.resolution
        
        while not frontier.empty():

            current = frontier.get()
            x = current[0]
            y = current[1]
            expanded_cells.append(self.grid_to_world(map_msg, x, y))

            rospy.sleep(.002)
            
            # publish expanded cells message
            msg_grid_cells.header.stamp = rospy.Time.now()
            msg_grid_cells.cells = expanded_cells
            self.astar_expanded.publish(msg_grid_cells)

            # publish frontier message
            queue = [self.grid_to_world(map_msg, c[1][0], c[1][1]) for c in frontier.get_queue()]

            msg_grid_cells.header.stamp = rospy.Time.now()
            msg_grid_cells.cells = queue
            self.astar_frontier.publish(msg_grid_cells)

            if current == goal:
                break
            
            adjacent_cells = self.neighbors_of_4(self, map_msg, x, y)
            adjacent_cells = [c for c in adjacent_cells]
            for next in self.neighbors_of_8(self, map_msg, x, y):
                new_cost = cost_so_far[current]
                if(next in adjacent_cells):
                    #print("next: ", next, " is adjacent to ", current)
                    new_cost += 1
                else:
                    new_cost += 1.41
                    
                if self.grid_to_index(map_msg, next[0], next[1]) not in self.c_space and (next not in cost_so_far or new_cost < cost_so_far[next]):
                    cost_so_far[next] = new_cost
                    def calc_heuristic(next):
                        distance = self.euclidean_distance(next[0], next[1], goal[0], goal[1])
                        neighbors = self.neighbors_of_8(self, map_msg, next[0], next[1])
                        cspace_weight = 0
                        print(self.c_space[1])
                        for neighbor in neighbors:
                            if self.grid_to_index(map_msg, neighbor[0], neighbor[1]) in self.c_space:
                                print(next, " is near c_space")
                                cspace_weight += 1
                                break
                        
                        turn_weight = 0
                        if(came_from[current]):
                            previous_angle = math.atan2(current[1] - came_from[current][1], current[0] - came_from[current][0])
                            current_angle = math.atan2(next[1] - current[1], next[0] - current[0])
                            if(previous_angle != current_angle):
                                turn_weight += 1
                        return (distance * 1.5) + (cspace_weight * 5) + turn_weight
                        
                    priority = new_cost + calc_heuristic(next)
                    frontier.put(next, priority)
                    came_from [next]= current
                
        print("DONE SEARCHING")
        path = [goal]
        if(not goal in came_from):
            print("NO GOAL FOUND")
            return None
        else:
            while came_from[path[-1]] is not None:
                path.append(came_from[path[-1]])
            path.reverse()
            return path
    
    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        prev_point = path[0]
        to_purge = []
        # Loop through points in the path starting with index 1
        prev_angle = 0
        for i in range(1,len(path)):
            current_point = path[i]
            
            current_angle = math.atan2(current_point[1] - prev_point[1], current_point[0] - prev_point[0])
            
            # compare angle of previous move to current move, if the same, add previous point to purge list
            if(abs(prev_angle - current_angle) < 0.005):
                to_purge.append(path[i-1])
                
            prev_point = current_point
            prev_angle = current_angle
        # remove all unnecessary nodes
        for i in to_purge:
            path.remove(i)
        
        return(path)
    
    @staticmethod
    def path_to_poses(self, map_msg, path):
        """
        Converts the given path into a list of PoseStamped.
        :param map_msg [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        poses = []
        for point in path:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose = Pose()
            pose_stamped.pose.position = self.grid_to_world(map_msg, point[0], point[1])

            poses.append(pose_stamped)
        
        return poses
    
    def path_to_message(self, map_msg, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        path_msg.poses = self.path_to_poses(self, map_msg, path)
        self.path_test.publish(path_msg)
        rospy.loginfo("Returning a Path message")
        return path_msg
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

if __name__ == '__main__':
    path_planning().run()