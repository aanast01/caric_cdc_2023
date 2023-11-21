### Explorer Path Planning Code ###
#### Created By Kios ####
##### 16 Nov 2023 #####
import rospy
from std_msgs.msg import Header, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from kios_solution.msg import area, multiPoint
from caric_mission.srv import CreatePPComTopic
from visualization_msgs.msg import MarkerArray
import numpy as np
import math

build_map = True
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
arrived = False

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

def arrivedCallback(msg):
    global arrived
    arrived = msg.data

def euclidean_distance(p1,point):
    p2 = np.zeros((3,1))
    p2[0] = point.x
    p2[1] = point.y
    p2[2] = point.z
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def process_boxes(msg):
    global debug
    points = msg.points
    minx = 99999
    miny = 99999
    minz = 99999
    maxx = -99999
    maxy = -99999
    maxz = -99999
    for point in points:
        if minx > point.x:
            #print("min x: " + str(point.x))
            minx = point.x
        if maxx < point.x:
            #print("max x: " + str(point.x))
            maxx = point.x

        if miny > point.y:
            #print("min y: " + str(point.y))
            miny = point.y
        if maxy < point.y:
            #print("max y: " + str(point.y))
            maxy = point.y

        if minz > point.z:
            #print("min z: " + str(point.z))
            minz = point.z
        if maxz < point.z:
            #print("max z: " + str(point.z))
            maxz = point.z

    #if debug:
    #    print("min x: " + str(minx) + " max x: " + str(maxx) + "\nmin y: " + str(miny) + " max y: " + str(maxy) + "\nmin z: " + str(minz) + " max z: " + str(maxz))

    return [minx, maxx, miny, maxy, minz, maxz]

def main():
    # init
    global grid_resolution, namespace, debug, odom, position, arrived
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
    
    rospy.Subscriber("/"+namespace+"/arrived_at_target", Bool, arrivedCallback)


    # target point publisher
    target_pub = rospy.Publisher("/"+namespace+"/command/targetPoint", Point, queue_size=1)

     # Wait for service to appear
    log_info("Waiting for ppcom")
    rospy.wait_for_service('/create_ppcom_topic')
    # Create a service proxy
    create_ppcom_topic = rospy.ServiceProxy('/create_ppcom_topic', CreatePPComTopic)
    # Register the topic with ppcom router
    if namespace == 'jurong':
        response = create_ppcom_topic(namespace, ['raffles'], '/'+namespace+'/command/update', 'std_msgs', 'Bool')
    else:
        response = create_ppcom_topic(namespace, ['jurong'], '/'+namespace+'/command/update', 'std_msgs', 'Bool')

    # update flag publisher
    flag_pub = rospy.Publisher("/"+namespace+"/command/update", Bool, queue_size=1, latch=False)

    # Get inspection area details
    log_info("Waiting for area details")
    area_details = rospy.wait_for_message("/world_coords/"+namespace, area)

    min_x = area_details.minPoint.x
    max_x = int(area_details.minPoint.x + area_details.size.x * area_details.resolution.data)
    mid_x = (min_x + max_x)/2.0

    min_y = area_details.minPoint.y
    max_y = int(area_details.minPoint.y + area_details.size.y * area_details.resolution.data)
    mid_y = (min_y + max_y)/2.0

    min_z = area_details.minPoint.z
    max_z = int(area_details.minPoint.z + area_details.size.z * area_details.resolution.data)
    mid_z = ((min_z + max_z)/2.0) + 10
    
    # get waypoints from TSP
    waypoints = rospy.wait_for_message("/"+namespace+"_waypoints/"+namespace, multiPoint)
    
    if scenario != 'hangar':
        # if x dimension is longest
        if (max_x >= max_y) and (max_x >= max_z):
            target_points_jurong = np.array([[min_x, max_y, mid_z],
                                                [max_x, max_y, mid_z]])
            
            target_points_raffles = np.array([[min_x, min_y, mid_z],
                                                [max_x, min_y, mid_z]])
        # if y dimension is longest
        elif (max_y >= max_x) and (max_y >= max_z):
            target_points_jurong = np.array([[min_x, min_y, mid_z],
                                                [min_x, max_y, mid_z]])
            
            target_points_raffles = np.array([[max_x, min_y, mid_z],
                                                [max_x, max_y, mid_z]])
        # if z dimension is longest
        elif (max_z >= max_x) and (max_z >= max_y):
            # if x dimension is longer than y
            if (max_x >= max_y):
                target_points_jurong = np.array([[mid_x, min_y, mid_z],
                                                    [mid_x, min_y, max_z]])
                
                target_points_raffles = np.array([[mid_x, max_y, mid_z],
                                                    [mid_x, max_y, max_z]])
                
            # if y dimension is longer than x
            else: 
                target_points_jurong = np.array([[min_x, mid_y, mid_z],
                                                [min_x, mid_y, max_z]])
            
                target_points_raffles = np.array([[max_x, mid_y, mid_z],
                                                    [max_x, mid_y, max_z]])
    else:
        target_points_jurong = np.array([[mid_x, max_y, max_z],
                                         [mid_x, min_y, max_z]])

    if agent_ID == 0:
        target_points = target_points_jurong
    else:
        target_points = target_points_raffles

    log_info("Waiting for adjacency build")
    rospy.wait_for_message("/"+namespace+"/adjacency_viz", MarkerArray)
    init_pos = position
    rate.sleep()

    if build_map:
        # go to initial points for map building
        for point in target_points:
            target_msg = Point()
            target_msg.x = point[0]
            target_msg.y = point[1]
            target_msg.z = point[2]
            log_info("Setting target to point: " + str(point))
            while not arrived:              #(euclidean_distance(point,position) >= area_details.resolution.data):
                #log_info(euclidean_distance(point,position))
                target_pub.publish(target_msg)
                rate.sleep()
            arrived = False

        bool_msg = Bool()
        bool_msg.data = True
        flag_pub.publish(bool_msg)
        rate.sleep()

        log_info("Waiting for map merge to complete")
        # wait for neighbor update flag
        if namespace == 'jurong':
            if scenario != 'hangar':
                rospy.wait_for_message("/"+namespace+"/command/update", Bool)
                rospy.wait_for_message("/raffles/command/update/"+namespace, Bool)
        else:
            rospy.wait_for_message("/"+namespace+"/command/update", Bool)
            rospy.wait_for_message("/jurong/command/update/"+namespace, Bool)
        

    # go to TSP points
    for point in waypoints.points:
        log_info("Setting target to point: " + str(point))
        while not arrived:              #(euclidean_distance(point,position) >= area_details.resolution.data):
            #log_info(euclidean_distance(point,position))
            target_pub.publish(point)
            rate.sleep()
        arrived = False
    

    # Return to Home (ensure LOS with GCS)
    log_info("Setting target to initial point: " + str(init_pos))
    while not arrived:              #(euclidean_distance(point,position) >= area_details.resolution.data):
        #log_info(euclidean_distance(point,position))
        target_pub.publish(init_pos)
        rate.sleep()
    arrived = False

    
    while not rospy.is_shutdown():
        log_info("Finished")
        flag_pub.publish(bool_msg)
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