### Explorer Path Planning Code ###
#### Created By Kios ####
##### 16 Nov 2023 #####
import rospy
from std_msgs.msg import Header, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from kios_solution.msg import area
from visualization_msgs.msg import MarkerArray
import numpy as np
import math

debug = False
TAG = ""
odom = Odometry()
position = Point()
target = 0
command_thread = None
cmdPub = None
coordinates = None
grid_resolution = 6
namespace = "jurong"

def set_tag(tag):
    global TAG
    TAG = tag

def log_info(info):
    global TAG, debug
    if debug:
        rospy.loginfo(TAG + f"{info}")
    #print(TAG)

def odomCallback(msg):
    global odom, position
    odom = msg
    position = odom.pose.pose.position

def euclidean_distance(p1,point):
    p2 = np.zeros((3,1))
    p2[0] = point.x
    p2[1] = point.y
    p2[2] = point.z
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def main():
    # init
    global grid_resolution, namespace, debug, odom, position
    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_resolution = rospy.get_param('grid_resolution')
        set_tag("[" + namespace.upper() + " PATH SCRIPT]: ")
        if namespace == 'jurong':
            agent_ID = 0
        else:
            agent_ID = 1
    except Exception as e:
        print(e)
        namespace = "ERROR"
        scenario = 'mbs'
        debug = True
        set_tag("[" + namespace.upper() + " SCRIPT]: ")
		
    rospy.init_node(namespace, anonymous=True)
    log_info(namespace)

    rate = rospy.Rate(10)

    # subscribe to self topics
    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", Odometry, odomCallback)

    # target point publisher
    target_pub = rospy.Publisher("/"+namespace+"/command/targetPoint", Point, queue_size=1)

    # Get inspection area details
    log_info("Waiting for area details")
    area_details = rospy.wait_for_message("/world_coords/"+namespace, area)

    min_x = area_details.minPoint.x
    max_x = int(area_details.minPoint.x + area_details.size.x * area_details.resolution.data) 
    min_y = area_details.minPoint.y
    max_y = int(area_details.minPoint.y + area_details.size.y * area_details.resolution.data)
    min_z = area_details.minPoint.z
    max_z = int(area_details.minPoint.z + area_details.size.z * area_details.resolution.data)

    
    target_points1 = np.array([[min_x, max_y, max_z],
                             [max_x, max_y, max_z]])
    
    target_points2 = np.array([[min_x, min_y, max_z],
                             [max_x, min_y, max_z]])
    
    if agent_ID == 0:
        target_points = target_points1
    else:
        target_points = target_points2

    log_info("Waiting for adjecency build")
    rospy.wait_for_message("/"+namespace+"/adjacency_viz", MarkerArray)
    rate.sleep()

    for point in target_points:
        target_msg = Point()
        target_msg.x = point[0]
        target_msg.y = point[1]
        target_msg.z = point[2]
        log_info("Setting target to point: " + str(point))
        while (euclidean_distance(point,position) >= area_details.resolution.data):
            #log_info(euclidean_distance(point,position))
            target_pub.publish(target_msg)
            rate.sleep()






if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("terminating...")
    except Exception as e:
        print(e)
    finally:
        exit()