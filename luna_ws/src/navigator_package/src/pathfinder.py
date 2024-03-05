from math import cos, sqrt, sin
import rospy
from std_msgs.msg import Float32, Int32
from nav_msgs import OccupancyGrid
from geometry_msgs import PoseStamped

rospy.init_node("pathfinder")



# state variables
nav_goal = (0,0,0)

# callback functions
def nav_goal_cb(nav_goal):
    


# publishers

# subscribe and hit that like button
rospy.subscriber('/move_base_simple/goal', PoseStamped, nav_goal_cb)

if __name__ == '__main__':
    