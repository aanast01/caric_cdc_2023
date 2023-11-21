### Photographer Trajectory Code ###
#### Created By Kios ####
##### 21 Nov 2023 #####
import sys
import rospy
from std_msgs.msg import Header, Float32, Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3, Quaternion, Twist, Point
from caric_mission.srv import CreatePPComTopic
from kios_solution.msg import area
from octomap_msgs.srv import BoundingBoxQuery
from visualization_msgs.msg import MarkerArray, Marker
import math
import numpy as np
import heapq
import threading
import time


maxVel = 2.0
debug = False
TAG = ""
odom = Odometry()
target = 0
neighbors = PointCloud2()
command_thread = None
cmdPub = None
coordinates = None
target_yaw = 0.0
grid_resolution = 6
namespace = "sentosa"
adjacency = np.zeros((2,2))

offsets_cross = [
    (0,-1,0), (1,0,0), (0,1,0), (-1,0,0), (0,0,1), (0,0,-1)
]

def dijkstra(g, arrival_pub, s, t):
    
    if (s==t):
        # init arrival message
        arrived_msg = Bool()
        arrived_msg.data = True
        arrival_pub.publish(arrived_msg)
        # log_info("ARRIVED")
        return [s,s]
    
    if (len(np.nonzero(g[s,:])[0]) == 0): 
        log_info("Source " + str(t) + " blocked")
        return [s,s]
    
    if (len(np.nonzero(g[:,t])[0]) == 0):
        log_info("Target " + str(t) + " not reachable")
        return [s,s]
    
    q = []
    d = {n: float('inf') for n in range(len(g))}
    p = {}

    d[s] = 0
    heapq.heappush(q, (0, s))

    last_w, curr_v = heapq.heappop(q)

    # log_info(g[:,t])
    while  curr_v != t:

        neighbor_indices = np.nonzero(g[curr_v,:])
        #log_info(neighbor_indices)
        #for n, n_w in zip(nodes,g[curr_v]):
        #log_info("kokos")
        for n in neighbor_indices[0]:
            #log_info("kokos")
            n_w = g[curr_v,n]
            #if n_w == 0:
            #    continue

            cand_w = last_w + n_w # equivalent to d[curr_v] + n_w 
            # print d # uncomment to see how deltas are updated
            if cand_w < d[n]:
                d[n] = cand_w
                p[n] = curr_v
                heapq.heappush(q, (cand_w, n))
        last_w, curr_v = heapq.heappop(q)  

    return generate_path(p, s, t)

def generate_path(parents, start, end):
        path = [end]
        #log_info("Recreating path")
        while True:
            key = parents[path[0]]
            path.insert(0, key)
            if key == start:
                break
        #log_info("Returning path")
        return path

def set_tag(tag):
    global TAG
    TAG = tag

def log_info(info):
    global TAG, debug
    if debug:
        rospy.loginfo(TAG + f"{info}")
    #print(TAG)

def odomCallback(msg):
    global odom
    odom = msg

def targetCallback(msg):
    global target, coordinates
    target_point = msg
    target =  closest_node_index((target_point.x,target_point.y,target_point.z),coordinates)

def yawCallback(msg):
    global target_yaw
    target_yaw = msg.data

def neighCallback(msg):
    global neighbors
    neighbors = msg

def update_adjacency_with_neighbors(adjacency):
    global neighbors, grid_resolution, coordinates, area_details

    adjacency_temp = np.copy(adjacency)
    for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
        if point[3] != 0: 
            index = closest_node_index_1((point[0], point[1], point[2]), coordinates)
            adjacency_temp[:,index]=0
            for _, offset in enumerate(offsets_cross):
                neighbor_x = coordinates[index,0]+(offset[0] * grid_resolution)
                neighbor_y = coordinates[index,1]+(offset[1] * grid_resolution)
                neighbor_z = coordinates[index,2]+(offset[2] * grid_resolution)
            
                
                gone_too_far_x = (neighbor_x < area_details.minPoint.x) or (neighbor_x > (area_details.minPoint.x + area_details.size.x*area_details.resolution.data))
                gone_too_far_y = (neighbor_y < area_details.minPoint.y) or (neighbor_y > (area_details.minPoint.y + area_details.size.y*area_details.resolution.data))
                gone_too_far_z = (neighbor_z < area_details.minPoint.z) or (neighbor_z > (area_details.minPoint.z + area_details.size.z*area_details.resolution.data))
                if gone_too_far_x or gone_too_far_y or gone_too_far_z:
                    #log_info(f"{gone_too_far_x}, {gone_too_far_y}, {gone_too_far_z}")
                    continue
                
                
                neighbor_index = closest_node_index_1((neighbor_x, neighbor_y, neighbor_z),coordinates)
                adjacency_temp[:,neighbor_index]=0

    arr = np.sum(adjacency, axis=1)
    isolated_indicies = np.where(arr <= grid_resolution)[0]
    for _, index in enumerate(isolated_indicies):
        #log_info("ISOLATED NODE: " + str(coordinates[index]))
        adjacency_temp[:,index] = 0

    return adjacency_temp

def euclidean_distance_3d(p1,p2):
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def go_to_point():
    global cmd_pub, odom, waypoint, target_yaw, maxVel
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #print(waypoint)
        if waypoint[0] != -3000:
            header_msg = Header()
            header_msg.frame_id = 'world'
            trajset_msg = MultiDOFJointTrajectory()
            trajpt_msg = MultiDOFJointTrajectoryPoint()
            transform_msgs = Transform()
            translation_msg = Vector3()
            rotation_msg = Quaternion()
            zero_vector_msg = Vector3()
            velocities_msg = Twist()
            acceleration_msg = Twist()
            
            
            translation_msg.x = 0.0
            translation_msg.y = 0.0
            translation_msg.z = 0.0
            rotation_msg.x = 0.0
            rotation_msg.y = 0.0
            rotation_msg.z = np.sin(target_yaw/2.0)
            rotation_msg.w = np.cos(target_yaw/2.0)
            
            
            velocities_msg.linear.x = max(min((waypoint[0]-odom.pose.pose.position.x) * 1.0,maxVel), -maxVel)
            velocities_msg.linear.y = max(min((waypoint[1]-odom.pose.pose.position.y) * 1.0,maxVel), -maxVel)
            velocities_msg.linear.z = max(min((waypoint[2]-odom.pose.pose.position.z) * 1.0,2.0), -2.0)
            

            
            #velocities_msg.linear = zero_vector_msg
            #q = odom.pose.pose.orientation
            #agent_yaw = np.degrees(np.arctan2(2.0 * (q.y * q.z + q.w *q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z))

            velocities_msg.angular.x = 0.0
            velocities_msg.angular.y = 0.0
            velocities_msg.angular.z = 0.0#max(min((target_yaw - agent_yaw) * 0.5, 5.0), -5.0)
            
            acceleration_msg.linear.x = 0.0
            acceleration_msg.linear.y = 0.0
            acceleration_msg.linear.z = 0.0

            acceleration_msg.angular.x = 0.0
            acceleration_msg.angular.y = 0.0
            acceleration_msg.angular.z = 0.0
            
            transform_msgs.translation = translation_msg
            transform_msgs.rotation = rotation_msg
            
            trajpt_msg.transforms.append(transform_msgs)
            trajpt_msg.velocities.append(velocities_msg)
            trajpt_msg.accelerations.append(acceleration_msg)
            
            trajset_msg.points.append(trajpt_msg)
            
            header_msg.stamp = rospy.Time.now()
            trajset_msg.header = header_msg

            cmdPub.publish(trajset_msg)
        rate.sleep()

def closest_node_index_1(node, nodes):
    nodes = np.asarray(nodes)
    deltas = nodes - node
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)

    return np.argmin(dist_2)

def closest_node_index(node, nodes):
    global adjacency
    arr = np.sum(adjacency, axis=0)
    valid_dist_indices = np.nonzero(arr)[0]
    nodes = np.asarray(nodes)
    deltas = nodes[valid_dist_indices] - node
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    
    return valid_dist_indices[np.argmin(dist_2)]

def main():
    # init
    global cmdPub, waypoint, command_thread, coordinates, target, grid_resolution, namespace, debug, adjacency, adjacency_final, area_details
    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_resolution = rospy.get_param('grid_resolution')
        set_tag("[" + namespace.upper() + " TRAJ SCRIPT]: ")
        #rospy.loginfo(TAG + namespace)
    except Exception as e:
        print(e)
        namespace = "sentosa"
        scenario = 'mbs'
        debug = True
        set_tag("[" + namespace.upper() + " TRAJ SCRIPT]: ")
		
    rospy.init_node(namespace, anonymous=True)
    log_info(namespace)

    rate = rospy.Rate(10)

    # subscribe to self topics
    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", Odometry, odomCallback)
    rospy.Subscriber("/"+namespace+"/command/targetPoint", Point, targetCallback)
    rospy.Subscriber("/"+namespace+"/command/yaw", Float32, yawCallback)
    

    # create command publisher
    cmdPub = rospy.Publisher("/"+namespace+"/command/trajectory", MultiDOFJointTrajectory, queue_size=1)
    # occupied coordinates publisher
    arrival_pub = rospy.Publisher('/'+namespace+'/arrived_at_target', Bool, queue_size=1)

    area_details = rospy.wait_for_message("/world_coords/"+namespace, area)

    # create thread
    waypoint = (-3000,-3000,-3000)
    command_thread = threading.Thread(target=go_to_point)
    command_thread.start()

    filename_msg = String()
    log_info("Waiting for map from explorers")
    explorer_name = "jurong"
    while len(filename_msg.data) == 0:
        try:
            filename_msg = rospy.wait_for_message("/jurong/adjacency/"+namespace, String, 1)
            log_info("Receivied map from Jurong")
            explorer_name = "jurong"
        except rospy.exceptions.ROSException as e:
            try:
                filename_msg = rospy.wait_for_message("/raffles/adjacency/"+namespace, String, 1)
                log_info("Receivied map from Raffles")
                explorer_name = "raffles"
            except rospy.exceptions.ROSException as e:
                pass
                #log_info("Waiting for map from explorers")
        rate.sleep() 
    
    log_info("Loading map")
    adjacency_final = np.loadtxt(filename_msg.data, delimiter=",")
    coordinates = np.loadtxt("./"+explorer_name+"_coordinates.csv", delimiter=",")
    adjacency = update_adjacency_with_neighbors(adjacency_final)
    

    log_info("Waiting for target point")
    arrival_pub.publish(True)
    try:
        rospy.wait_for_message("/"+namespace+"/command/targetPoint", Point)
    except rospy.exceptions.ROSException as e:
        log_info("Waiting for target point TIMEOUT")
        target =  closest_node_index((odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z),coordinates)

    while not rospy.is_shutdown():
            agent_index = closest_node_index_1(([odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z]),coordinates)
            #log_info("Generating path. Starting Point: " + str(agent_index) + " Target Point: " + str(target))
            path = dijkstra(adjacency, arrival_pub, agent_index, target)
            #log_info("Going to point: " + str(coordinates[path[1]]))
            waypoint = coordinates[path[1]]

            adjacency = update_adjacency_with_neighbors(adjacency_final)



if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("terminating...")
        command_thread.terminate()
    except Exception as e:
        print(e)
    finally:
        exit()