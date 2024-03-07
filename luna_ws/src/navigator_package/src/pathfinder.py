from math import sqrt
import rospy
import queue
from std_msgs.msg import Float32, Int32
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Point


class PathFinder:
    # state variables
    goal = (0,0)
    occupancy_grid:OccupancyGrid = OccupancyGrid()
    current_pose = (1,1)

    def __init__(self):
        #subscribers
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.nav_goal_cb)
        self.map_sub = rospy.Subscriber('/robot/map', OccupancyGrid, self.new_map_cb)
        self.fused_pose_sub = rospy.Subscriber('/jetson/filtered_pose', PoseStamped, self.fused_pose_cb)

        #publishers
        self.path_pub = rospy.Publisher('/jetson/nav_path', Path, queue_size = 1)
        self.occupancy_grid.info.resolution = 1
    

    # callback functions
    def nav_goal_cb(self, nav_goal:PoseStamped):
        point = nav_goal.pose.position
        resolution = self.occupancy_grid.info.resolution

        self.goal = (int(point.x / resolution), int(point.y / resolution))

        if self.isDrivable(self.goal):
            print("drivable")
            self.path_pub.publish(self.a_star())

        else:
            print("not drivable")

    def new_map_cb(self, grid:OccupancyGrid):
        self.occupancy_grid = grid
    
    def fused_pose_cb(self, pose:PoseStamped):
        point = pose.pose.position
        resolution = self.occupancy_grid.info.resolution

        self.current_pose = (int(point.x / resolution), int(point.y / resolution))
        
    #helper functions

    #isDrivable takes a tuple representing x and y cell numbers in the occupancy grid
    def isDrivable(self, postion):
        if postion[0] < 0 or postion[1] < 0 or postion[0] >= self.occupancy_grid.info.width or postion[1] >= self.occupancy_grid.info.height:
            return False
        
        grid = self.occupancy_grid.data
        index = postion[1] * self.occupancy_grid.info.width + postion[0]

        if grid[index] < 10:
            return True
        return False
    
    def neighbors_of_8(self, position):
        x = position[0]
        y = position[1]
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                new_pos = (x+dx, y+dy, dx, dy)
                if (dx,dy) != (0,0) and self.isDrivable(new_pos):
                    neighbors.append(new_pos)
        return neighbors
    
    def euchlid_dist(self, start, end):
        return sqrt((start[0] - end[0]) **2 + (start[1] - end[1])**2)

    def a_star(self):
        
        frontier = queue.PriorityQueue()

        frontier.put(self.current_pose, 0)

        came_from = {}
        cost_to = {}
        dir_to = {}


        came_from[self.current_pose] = None
        cost_to[self.current_pose] = 0
        dir_to[self.current_pose] = (0,0)

        while not frontier.empty():
            current = frontier.get()

            if current == self.goal:
                break

            current_dir = dir_to[current]

            for next in self.neighbors_of_8(current):
                next_pos = (next[0], next[1])
                next_dir = (next[2], next[3])

                new_cost = cost_to[current] + self.euchlid_dist(current, next_pos)

                # reward paths that dont turn as much
                if(next_dir != current_dir):
                    new_cost += .5
                
                
                if next_pos not in cost_to or new_cost < cost_to[next_pos]:
                    cost_to[next_pos] = new_cost
                    dir_to[next_pos] = next_dir
                    
                    heuristic = self.euchlid_dist(next_pos, self.goal)
                    priority = new_cost + heuristic
                    frontier.put(next_pos, priority)
                    came_from[next_pos] = current
        
        # reconstruct the path back
        current = self.goal
        path = Path()
        path.header.frame_id = "world"
        cost = 0
        try:
            while current != self.current_pose:
                pose = PoseStamped()

                resolution = self.occupancy_grid.info.resolution

                pose.pose.position.x = current[0] * resolution + resolution / 2
                pose.pose.position.y = current[1] * resolution + resolution / 2

                path.poses.append(pose)
                current = came_from[current]
            
            return path
        except KeyError:
            return -1


# main
if __name__ == '__main__':
    rospy.init_node("pathfinder")
    PathFinder()
    rospy.spin()

