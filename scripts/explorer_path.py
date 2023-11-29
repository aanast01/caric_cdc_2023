### Explorer Path Planning Code ###
#### Created By Kios ####
##### 16 Nov 2023 #####
__author__ = "Andreas Anastasiou, Angelos Zacharia"
__copyright__ = "Copyright (C) 2023 Kios Center of Excellence"
__version__ = "7.0"

import sys
import rospy
from std_msgs.msg import String, Bool, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointCloud
import sensor_msgs.point_cloud2
from kios_solution.msg import area, norms
from caric_mission.srv import CreatePPComTopic
from visualization_msgs.msg import MarkerArray
from scipy.spatial import Delaunay
import numpy as np, pandas as pd
import math
import time

build_map = True
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

def check_point_inside_cuboid(vertices, point):
    DT = Delaunay(vertices)

    if DT.find_simplex(point) >= 0:
        return True
    return False

def calculateCircuits(positions, num_of_nodes, TravellingCost):
    UAVs = len(positions)
    # positions = index where each uav is located

    # Initialization of Set S matrices and CircuitX, CircuitY.
    Set_S_source = [[] for i in range(0, UAVs)]
    Set_S_destination = [[] for i in range(0, UAVs)]
    Set_S_cost = [[] for i in range(0, UAVs)]
    listV1 = [0 for i in range(0, num_of_nodes)]
    for z in range(0, UAVs):
        listV1[positions[z]] = 1
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
            minx = point.x
        if maxx < point.x:
            maxx = point.x

        if miny > point.y:
            miny = point.y
        if maxy < point.y:
            maxy = point.y

        if minz > point.z:
            minz = point.z
        if maxz < point.z:
            maxz = point.z

    return [minx, maxx, miny, maxy, minz, maxz]

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
    # norm pub
    norm_pub = rospy.Publisher("/"+namespace+"/norms", norms, queue_size=1)

    # Get Bounding Box Verticies
    bboxes = rospy.wait_for_message("/gcs/bounding_box_vertices/", PointCloud)
    bbox_points = np.zeros((int(len(bboxes.points)/8),8,3))
    counter = 0
    for i in range(0,int(len(bboxes.points)/8)):
        for j in range(8):
            bbox_points[i,j,0] = bboxes.points[counter].x
            bbox_points[i,j,1] = bboxes.points[counter].y
            bbox_points[i,j,2] = bboxes.points[counter].z
            counter += 1

    # Get inspection area details
    log_info("Waiting for area details")
    area_details = rospy.wait_for_message("/world_coords/"+namespace, area)

    min_x = area_details.minPoint.x + 10.0
    max_x = int(area_details.minPoint.x + area_details.size.x * area_details.resolution.data) - 10.0
    mid_x = (min_x + max_x)/2.0

    min_y = area_details.minPoint.y + 10.0
    max_y = int(area_details.minPoint.y + area_details.size.y * area_details.resolution.data) - 10.0
    mid_y = (min_y + max_y)/2.0

    min_z = area_details.minPoint.z + 10.0
    max_z = int(area_details.minPoint.z + area_details.size.z * area_details.resolution.data)
    mid_z = ((min_z + max_z)/2.0)

    rospy.wait_for_message("/"+namespace+"/ground_truth/odometry", Odometry)
    init_pos = position

    if scenario != 'hangar':
        # if x dimension is longest
        if (max_x >= max_y) and (max_x >= max_z):
            target_points_jurong = np.array([[min_x, max_y, mid_z],
                                             [max_x, max_y, mid_z],
                                             [mid_x, max_y, max_z]])
            
            target_points_raffles = np.array([[min_x, min_y, mid_z],
                                              [max_x, min_y, mid_z],
                                              [mid_x, min_y, max_z]])
        # if y dimension is longest
        elif (max_y >= max_x) and (max_y >= max_z):
            target_points_jurong = np.array([[min_x, min_y, mid_z],
                                             [min_x, max_y, mid_z],
                                             [min_x, mid_y, max_z]])
            
            target_points_raffles = np.array([[max_x, min_y, mid_z],
                                              [max_x, max_y, mid_z],
                                              [max_x, mid_y, max_z]])
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
                
        # points = np.concatenate((target_points_jurong, target_points_raffles))

        # adjacency = np.zeros((points.shape[0],points.shape[0]))
        # for i in range(points.shape[0]):
        #     for j in range(points.shape[0]):
        #         adjacency[i,j] = euclidean_distance_3d(points[i],points[j])

        # log_info("Running mTSP")
        # waypointsMatrix = calculateCircuits([i for i in range(num_of_agents)], num_of_nodes, adjacency)
    else:
        max_y = max_y -10
        min_y = min_y-5
        p1 = np.array([mid_x-10, max_y, mid_z])
        p2 = np.array([mid_x+10, max_y, mid_z])
        p3 = np.array([mid_x-10, min_y, mid_z])
        p4 = np.array([mid_x+10, min_y, mid_z])
        d1 = euclidean_distance(p1, init_pos)
        d2 = euclidean_distance(p2, init_pos)
        d3 = euclidean_distance(p3, init_pos)
        d4 = euclidean_distance(p4, init_pos)
        ind = np.where(np.array([d1, d2, d3, d4])==min(d1, d2, d3, d4))[0]
        if ind==0:
            target_points_jurong = np.array([p1,p3,p4,p2])
        elif ind==1:
            target_points_jurong = np.array([p2,p4,p3,p1])
        elif ind==2:
            target_points_jurong = np.array([p3,p1,p2,p4])
        elif ind==3:
            target_points_jurong = np.array([p4,p2,p1,p3])


    
    if drone_IDs[namespace] == drone_IDs['jurong']:
        # log_info("TSP Path length: " + str(len(waypointsMatrix[0])))
        # if scenario != 'hangar':
        #     target_points = waypointsMatrix[0]
        # else:
        target_points = target_points_jurong
    else:
        # log_info("TSP Path length: " + str(len(waypointsMatrix[1])))
        # target_points = waypointsMatrix[1]
        target_points = target_points_raffles

    log_info("Waiting for adjacency build")
    rospy.wait_for_message("/"+namespace+"/adjacency_viz", MarkerArray)
    rate.sleep()

    # log_info("Waiting for waypoints")
    # filename_msg = rospy.wait_for_message("/waypoints/"+namespace, String)

    if build_map:
        # go to initial points for map building
        for point in target_points:
            target_msg = Point()
            target_msg.x = point[0]
            target_msg.y = point[1]
            target_msg.z = point[2]
            log_info("Setting target to point: " + str(point))
            while not arrived:
                target_pub.publish(target_msg)
                rate.sleep()
            arrived = False

        bool_msg = Bool()
        bool_msg.data = True
        flag_pub.publish(bool_msg)
        rate.sleep()
        update_done = False
        log_info("Waiting for map merge to complete")
        # wait for neighbor update flag
        if namespace == 'jurong':
            if scenario != 'hangar':
                while not update_done:
                    try:
                        flag_pub.publish(bool_msg)
                        msg = rospy.wait_for_message("/"+namespace+"/command/update_done", Bool,1)
                        update_done = msg.data
                    except:
                        pass
                    rate.sleep()
        else:
            while not update_done:
                try:
                    flag_pub.publish(bool_msg)
                    msg = rospy.wait_for_message("/"+namespace+"/command/update_done", Bool,1)
                    update_done = msg.data
                except:
                    pass
                rate.sleep()

    vel_msg = Float32()
    vel_msg.data = 3.0
    velo_pub.publish(vel_msg)


    # Generate and go to TSP points
    log_info("Loading map")
    # cleared_inspect_points = np.loadtxt(filename_msg.data, delimiter=",")
    filename_msg = rospy.wait_for_message("/"+namespace+"/adjacency/", String)
    # start = time.time()
    adjacency = np.loadtxt(filename_msg.data, delimiter=",")
    # duration = time.time() - start
    # log_info("Numpy Load Map Time: " +  str(duration) + "s")
    # start = time.time()
    # adjacency = pd.read_csv(filename_msg.data, delimiter=",").values
    # duration = time.time() - start
    # log_info("Pandas Load Map Time: " +  str(duration) + "s")

    coordinates = np.loadtxt("./"+namespace+"_coordinates.csv", delimiter=",")
    # coordinates = pd.read_csv("./"+namespace+"_coordinates.csv", delimiter=",").values
    # print(adjacency.shape)
    # print(coordinates.shape)


    # start = time.time()
    arr = np.sum(adjacency, axis=0)
    valid_dist_indices = np.where(arr == 0)[0]
    log_info("Calculating waypoints")
    targeted_points = np.empty((0,1))
    inspect_points = np.empty((0,1))
    for index in valid_dist_indices:
            for box_i in range(0,int(len(bboxes.points)/8)):
                if check_point_inside_cuboid(bbox_points[box_i], coordinates[index]):
                    targeted_points = np.append(targeted_points, index)
                    break
    # duration = time.time() - start
    # log_info("Find Target Voxels Time: " +  str(duration) + "s")

    # start = time.time()
    targeted_points = targeted_points.astype(int)
    all_norms = np.empty((0,3))
    for target_point in targeted_points:
        # target_point = int(target_point)
        points = np.where(adjacency[target_point]>0)[0]
        inspect_points = np.append(inspect_points, points)
        for point in points:
            norm = coordinates[point] - coordinates[target_point]
            norm /= np.linalg.norm(norm)
            all_norms = np.append(all_norms, [norm], axis=0)
    # duration = time.time() - start
    # log_info("Calculate Norms Time: " +  str(duration) + "s")

    log_info("Running unique")
    inspect_points, ind = np.unique(inspect_points,axis=0, return_index=True)
    all_norms = all_norms[ind]
    inspect_points = inspect_points.astype(int)

    # np.savetxt("./"+namespace+"_newPoints.csv", coordinates[inspect_points], delimiter=',')

    log_info("Constructing norms message")
    # start = time.time()
    norm_msg = norms()
    for i, point in enumerate(inspect_points):
        facet_mid = Point()
        facet_mid.x = coordinates[point,0]
        facet_mid.y = coordinates[point,1]
        facet_mid.z = coordinates[point,2]
        norm_msg.facet_mids.append(facet_mid)
        norm_point = Point()
        norm_point.x = all_norms[i,0]
        norm_point.y = all_norms[i,1]
        norm_point.z = all_norms[i,2]
        norm_msg.normals.append(norm_point)
    # duration = time.time() - start
    # log_info("Build Norms Message Time: " +  str(duration) + "s")

    norm_pub.publish(norm_msg)
    count = 0
    while repeat:
        log_info("Constructing TSP matrix")
        neighbors = PointCloud2()
        try:
            neighbors = rospy.wait_for_message("/"+namespace+"/nbr_odom_cloud", PointCloud2,2)
        except:
            pass
        uav_positions = np.empty((0,3))
        uav_indices = np.array([])
        for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
            if point[3] != drone_IDs['gcs']:
                # log_info(point[3])
                uav_positions = np.append(uav_positions, [[point[0], point[1], point[2]]], axis=0)
                uav_indices = np.append(uav_indices, point[3])

        pos = 0
        if(uav_positions.shape[0]!=0):
            while uav_indices[pos] < drone_IDs[namespace]:
                pos += 1
        
        uav_positions = np.insert(uav_positions, pos, [position.x, position.y, position.z], axis=0)
        uav_indices = np.insert(uav_indices, pos, drone_IDs[namespace])

        num_of_agents = uav_positions.shape[0]

        points = np.concatenate((uav_positions, coordinates[inspect_points]))
        num_of_nodes = points.shape[0]

        adjacency = np.zeros((num_of_nodes,num_of_nodes))
        for i in range(num_of_nodes):
            for j in range(num_of_nodes):
                adjacency[i,j] = euclidean_distance_3d(points[i],points[j])

        log_info("Running mTSP")
        waypointsMatrix = calculateCircuits([i for i in range(num_of_agents)], num_of_nodes, adjacency)
        log_info("TSP Path length: " + str(len(waypointsMatrix[pos])))
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
            vel_msg.data = 3.0 - count
            velo_pub.publish(vel_msg)
        count += 1
    

    # Return to Home (ensure LOS with GCS)
    log_info("Setting target to initial point: " + str(init_pos))
    while not arrived:
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