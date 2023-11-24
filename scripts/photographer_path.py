### Photographer Path Planning Code ###
#### Created By Kios ####
##### 21 Nov 2023 #####
import sys
import rospy
from std_msgs.msg import String, Bool, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
from kios_solution.msg import area, multiPoint
from caric_mission.srv import CreatePPComTopic
from visualization_msgs.msg import MarkerArray
import numpy as np
import math

repeat = True
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
drone_IDs = {'gcs':0, 'jurong':1, 'raffles':2, 'sentosa':3, 'changi':4, 'nanyang':5}

def calculateCircuits(positions, num_of_nodes, TravellingCost):
    UAVs = len(positions)
    # positions = index where each uav is located

    # Initialization of Set S matrices and CircuitX, CircuitY.
    Set_S_source = [[] for i in range(0, UAVs)]
    Set_S_destination = [[] for i in range(0, UAVs)]
    Set_S_cost = [[] for i in range(0, UAVs)]
    listV1 = [0 for i in range(0, num_of_nodes)]
    # print("POSS: ",positions[:])
    for z in range(0, UAVs):
        listV1[positions[z]] = 1
    # print positions
    # print "before listV: ", listV1
    # assignment of the first K nodes.
    while (sum(listV1) < num_of_nodes):
        for i in range(0, UAVs):
            node = 0
            flag = False
            futureCost = sys.maxsize
            for j in range(0, num_of_nodes):
                if (listV1[j] == 0):
                    if (futureCost >= TravellingCost[positions[i]][j]):
                        futureCost = TravellingCost[positions[i]][j]
                        node = j
                        flag = True
            if flag:
                listV1[node] = 1
                Set_S_source[i].append(positions[i])
                #Set_S_destination[i].append(positions[i])
                Set_S_destination[i].append(node)
                Set_S_cost[i].append(futureCost)
                positions[i] = node

    
    return Set_S_destination

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

def euclidean_distance_3d(p1,p2):
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def main():
    # init
    global grid_resolution, namespace, debug, odom, position, arrived, repeat
    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_resolution = rospy.get_param('grid_resolution')
        set_tag("[" + namespace.upper() + " PATH SCRIPT]: ")
    except Exception as e:
        print(e)
        namespace = "ERROR"
        scenario = 'mbs'
        debug = True
        set_tag("[" + namespace.upper() + " PATH SCRIPT]: ")
		
    rospy.init_node(namespace, anonymous=True)
    log_info(namespace)

    rate = rospy.Rate(10)

    # subscribe to self topics
    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", Odometry, odomCallback)
    
    rospy.Subscriber("/"+namespace+"/arrived_at_target", Bool, arrivedCallback)


    # target point publisher
    target_pub = rospy.Publisher("/"+namespace+"/command/targetPoint", Point, queue_size=1)
    # velocity publisher
    velo_pub = rospy.Publisher("/"+namespace+"/command/velocity", Float32, queue_size=1)

    filename_msg = rospy.wait_for_message("/waypoints/"+namespace, String)
    
    log_info("Waiting for traj script")
    rospy.wait_for_message("/"+namespace+"/arrived_at_target", Bool)
    init_pos = position
    rate.sleep()


    # Generate and go to TSP points
    log_info("Loading waypoints")
    cleared_inspect_points = np.loadtxt(filename_msg.data, delimiter=",")
    count = 0
    while repeat:
        neighbors = rospy.wait_for_message("/"+namespace+"/nbr_odom_cloud", PointCloud2)
        uav_positions = np.empty((0,3))
        uav_indices = np.array([])
        for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
            if point[3] != drone_IDs['gcs']:
                uav_positions = np.append(uav_positions, [[point[0], point[1], point[2]]], axis=0)
                uav_indices = np.append(uav_indices, point[3])

        pos = 0
        while (pos < len(uav_indices)) and (uav_indices[pos] < drone_IDs[namespace]):
            pos += 1
        
        uav_positions = np.insert(uav_positions, pos, [position.x, position.y, position.z], axis=0)
        uav_indices = np.insert(uav_indices, pos, drone_IDs[namespace])

        num_of_agents = uav_positions.shape[0]


        points = np.concatenate((uav_positions, cleared_inspect_points))
        num_of_nodes = points.shape[0]

        adjacency = np.zeros((num_of_nodes,num_of_nodes))
        for i in range(num_of_nodes):
            for j in range(num_of_nodes):
                adjacency[i,j] = euclidean_distance_3d(points[i],points[j])

        log_info("Running mTSP")

        waypointsMatrix = calculateCircuits([i for i in range(num_of_agents)], num_of_nodes, adjacency)

        for waypoint in waypointsMatrix[pos]:
            point = Point()
            point.x = points[waypoint,0]
            point.y = points[waypoint,1]
            point.z = points[waypoint,2]
            log_info("Setting target to point: " + str(point))
            while not arrived:
                target_pub.publish(point)
                rate.sleep()
            arrived = False

        if count < 2.0:
            vel_msg = Float32()
            vel_msg.data = 2.5 - count
            velo_pub.publish(vel_msg)
        count += 0.5
    

    # Return to Home (ensure LOS with GCS)
    log_info("Setting target to initial point: " + str(init_pos))
    while not arrived:              #(euclidean_distance(point,position) >= area_details.resolution.data):
        #log_info(euclidean_distance(point,position))
        target_pub.publish(init_pos)
        rate.sleep()
    arrived = False

    
    while not rospy.is_shutdown():
        log_info("Finished")
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