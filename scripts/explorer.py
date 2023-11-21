### Explorer Trajectory Code ###
#### Created By Kios ####
##### 13 Nov 2023 #####
import rospy
from std_msgs.msg import Header, Float32, Bool
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

def dijkstra(g, arrival_pub, s, t):
    
    if (s==t):
        # init arrival message
        arrived_msg = Bool()
        arrived_msg.data = True
        arrival_pub.publish(arrived_msg)
        log_info("ARRIVED")
        return [s,s]
    
    if (len(np.nonzero(g[s,:])[0]) == 0) or (len(np.nonzero(g[:,t])[0]) == 0):
        log_info("Target" + str(t) + " not reachable")
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
namespace = "jurong"
adjacency = np.zeros((2,2))
adjacency_final = np.zeros((2,2))
update=True
mutex = threading.Lock()

#neighbors' offsets starting from same z plane counter-clockwise
offsets_all = [
    (-1,-1,0), (0,-1,0), (1,-1,0), (1,0,0), (1,1,0), (0,1,0), (-1,1,0), (-1,0,0), 
    (-1,-1,1), (0,-1,1), (1,-1,1), (1,0,1), (1,1,1), (0,1,1), (-1,1,1), (-1,0,1),(0,0,1),
    (-1,-1,-1), (0,-1,-1), (1,-1,-1), (1,0,-1), (1,1,-1), (0,1,-1), (-1,1,-1), (-1,0,-1),(0,0,-1)
]
offsets_cross = [
    (0,-1,0), (1,0,0), (0,1,0), (-1,0,0), (0,0,1), (0,0,-1)
]

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

def euclidean_distance_3d(p1,p2):
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def constuct_adjacency(area_details, coordinates):
    global offsets_cross
    num_of_nodes = int(area_details.size.x * area_details.size.y * area_details.size.z)
    adjacency_1 = np.zeros((num_of_nodes,num_of_nodes))
    log_info("Starting Adjacency calculation. Please wait... ")
    #'''
    for _,coord in enumerate(coordinates):
        # if i%2!=0:
        #     continue
        #start = rospy.Time.now().nsecs
        for _, offset in enumerate(offsets_cross):
            neighbor_x = coord[0]+(offset[0] * area_details.resolution.data)
            neighbor_y = coord[1]+(offset[1] * area_details.resolution.data)
            neighbor_z = coord[2]+(offset[2] * area_details.resolution.data)
           
            
            gone_too_far_x = (neighbor_x < area_details.minPoint.x) or (neighbor_x > (area_details.minPoint.x + area_details.size.x*area_details.resolution.data))
            gone_too_far_y = (neighbor_y < area_details.minPoint.y) or (neighbor_y > (area_details.minPoint.y + area_details.size.y*area_details.resolution.data))
            gone_too_far_z = (neighbor_z < area_details.minPoint.z) or (neighbor_z > (area_details.minPoint.z + area_details.size.z*area_details.resolution.data))
            if gone_too_far_x or gone_too_far_y or gone_too_far_z:
                #log_info(f"{gone_too_far_x}, {gone_too_far_y}, {gone_too_far_z}")
                continue
            
            
            neighbor_index = closest_node_index_1((neighbor_x, neighbor_y, neighbor_z),coordinates)
            my_index = closest_node_index_1((coord[0], coord[1], coord[2]),coordinates)
            
            
            cost = euclidean_distance_3d(coord, coordinates[neighbor_index])
            # if(cost <= area_details.resolution.data*2):
            try:
                adjacency_1[my_index,neighbor_index] = cost
                adjacency_1[neighbor_index,my_index] = cost
            except:
                pass

    return adjacency_1

def update_adjacency(adjacency, coordinates, obstacle_coordinates):    
    global grid_resolution
    adjacency_temp = np.copy(adjacency)
    # for _, obstacle in enumerate(obstacle_coordinates.points):
    for _, obstacle in enumerate(sensor_msgs.point_cloud2.read_points(obstacle_coordinates, skip_nans=True)):
        index = closest_node_index_1((obstacle[0], obstacle[1], obstacle[2]), coordinates)
        # delta_x = coordinates[index][0] - obstacle[0]#.x
        # delta_y = coordinates[index][1] - obstacle[1]#.y
        # delta_z = coordinates[index][2] - obstacle[2]#.z
        # if np.linalg.norm([delta_x,delta_y,delta_z]) < grid_resolution:
        adjacency_temp[:,index] = 0

    update_adjacency_with_neighbors(adjacency)

    return adjacency_temp

def update_adjacency_with_neighbors(adjacency):
    global neighbors, grid_resolution

    adjacency_temp = np.copy(adjacency)
    for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
        index = closest_node_index_1((point[0], point[1], point[2]), coordinates)
        adjacency_temp[:,index]=0

    arr = np.sum(adjacency, axis=1)
    isolated_indicies = np.where(arr <= grid_resolution*2)[0]
    for _, index in enumerate(isolated_indicies):
        #log_info("ISOLATED NODE: " + str(coordinates[index]))
        adjacency_temp[:,index] = 0

    return adjacency_temp

def update_from_neighbor(coordinates):
    global adjacency, update, namespace, mutex, adjacency_final
    log_info("waiting for update command")
    flag_pub = rospy.Publisher("/"+namespace+"/command/update", Bool, queue_size=1, latch=False)
    
    rospy.wait_for_message("/"+namespace+"/command/update", Bool)
    rate = rospy.Rate(1)
    # clear_agent_box(4, namespace)
    occupancy_coords = PointCloud2()
    log_info("Waiting for neighbor map")
    while len(occupancy_coords.data) == 0:
        try:
            if namespace == 'jurong':
                occupancy_coords = rospy.wait_for_message('/raffles/occupancy_coords/'+namespace, PointCloud2, 1)
            else:
                occupancy_coords = rospy.wait_for_message('/jurong/occupancy_coords/'+namespace, PointCloud2, 1)
        except rospy.exceptions.ROSException as e:
            log_info("Waiting for neighbor map")
        rate.sleep()

    update = False
    log_info("Acquiring mutex")
    mutex.acquire()
    log_info("Merging map")
    adjacency_final = update_adjacency(adjacency, coordinates, occupancy_coords)
    mutex.release()
    log_info("Merging DONE")
    while not rospy.is_shutdown():
        flag_pub.publish(True)
        rate.sleep

def get_node_index(coordinates, x,y,z):
    return coordinates.index((x, y, z))

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

def clear_agent_box(size, namespace):
    global odom, neighbors
    clear_bbox = rospy.ServiceProxy('/'+namespace+'/octomap_server_'+namespace+'/clear_bbx', BoundingBoxQuery)
    min = Point()
    max = Point()
    min.x = odom.pose.pose.position.x - size/4
    min.y = odom.pose.pose.position.y - size/4
    min.z = odom.pose.pose.position.z - size/4
    max.x = odom.pose.pose.position.x + size/4
    max.y = odom.pose.pose.position.y + size/4
    max.z = odom.pose.pose.position.z + size/4
    clear_bbox(max=max, min=min)

    for point in sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True):
        min.x = point[0] - size/4
        min.y = point[1] - size/4
        min.z = point[2] - size/4
        max.x = point[0] + size/4
        max.y = point[1] + size/4
        max.z = point[2] + size/4
        clear_bbox(max=max, min=min)

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

def take_off():
    global cmd_pub, odom, waypoint
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
    

    translation_msg.x = odom.pose.pose.position.x
    translation_msg.y = odom.pose.pose.position.y
    translation_msg.z = 1.5
    rotation_msg.z = 3.14/4.0
    rotation_msg.w = 3.14/4.0
    
    
    # velocities_msg.linear.x = min((waypoint[0]-odom.pose.pose.position.x) * 1.0,3.0)
    # velocities_msg.linear.y = min((waypoint[1]-odom.pose.pose.position.y) * 1.0,3.0)
    # velocities_msg.linear.z = min((waypoint[2]-odom.pose.pose.position.z) * 1.0,2.0)
    
    
    velocities_msg.linear = zero_vector_msg
    velocities_msg.angular = zero_vector_msg
    
    acceleration_msg.linear = zero_vector_msg
    acceleration_msg.angular = zero_vector_msg
    
    transform_msgs.translation = translation_msg
    transform_msgs.rotation = rotation_msg
    
    trajpt_msg.transforms.append(transform_msgs)
    trajpt_msg.velocities.append(velocities_msg)
    trajpt_msg.accelerations.append(acceleration_msg)
    
    trajset_msg.points.append(trajpt_msg)
    
    header_msg.stamp = rospy.Time.now()
    trajset_msg.header = header_msg

    rate = rospy.Rate(10)
    while odom.pose.pose.position.z < 1.2:
        cmdPub.publish(trajset_msg)

def publish_graph_viz(pub, coords, adj):
    global waypoint, namespace
    marker_array = MarkerArray()

    for indx, coord in enumerate(coords):
        if sum(adj[:,indx]) == 0:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = grid_resolution
            marker.scale.y = grid_resolution
            marker.scale.z = grid_resolution
            #marker.color.a = 0.05
            #index = closest_node_index(coord,coords)
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            # else:
            #     marker.color.r = 1.0
            #     marker.color.g = 1.0
            #     marker.color.b = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.pose.position.z = coord[2]
            marker_array.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = grid_resolution/2.0
    marker.scale.y = grid_resolution/2.0
    marker.scale.z = grid_resolution/2.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    if namespace == "jurong":
        marker.color.g = 0.0
        marker.color.b = 1.0
    else:
        marker.color.g = 1.0
        marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = coords[target][0]
    marker.pose.position.y = coords[target][1]
    marker.pose.position.z = coords[target][2]
    marker_array.markers.append(marker)

    id = 0
    for m in marker_array.markers:
        m.id = id
        id += 1

    pub.publish(marker_array)

def main():
    # init
    global cmdPub, waypoint, command_thread, coordinates, target, grid_resolution, namespace, debug, adjacency, update, mutex, adjacency_final
    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_resolution = rospy.get_param('grid_resolution')
        set_tag("[" + namespace.upper() + " TRAJ SCRIPT]: ")
        #rospy.loginfo(TAG + namespace)
    except Exception as e:
        print(e)
        namespace = "jurong"
        scenario = 'mbs'
        debug = True
        set_tag("[" + namespace.upper() + " SCRIPT]: ")
		
    rospy.init_node(namespace, anonymous=True)
    log_info(namespace)

    rate = rospy.Rate(10)

    # subscribe to self topics
    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", Odometry, odomCallback)
    rospy.Subscriber("/"+namespace+"/command/targetPoint", Point, targetCallback)
    rospy.Subscriber("/"+namespace+"/command/yaw", Float32, yawCallback)
    

    # create command publisher
    cmdPub = rospy.Publisher("/"+namespace+"/command/trajectory", MultiDOFJointTrajectory, queue_size=1)
    # adjacency vis pub
    viz_pub = rospy.Publisher("/"+namespace+"/adjacency_viz", MarkerArray, queue_size=1)
    # occupied coordinates publisher
    occ_pub = rospy.Publisher('/'+namespace+'/occupancy_coords', PointCloud2, queue_size=1)
    # occupied coordinates publisher
    arrival_pub = rospy.Publisher('/'+namespace+'/arrived_at_target', Bool, queue_size=1)

    # Get Neighbor Positions
    rospy.Subscriber("/"+namespace+"/nbr_odom_cloud", PointCloud2, neighCallback)

    # Create a ppcom publisher
    # Wait for service to appear
    log_info("Waiting for ppcom")
    rospy.wait_for_service('/create_ppcom_topic')
    # Create a service proxy
    create_ppcom_topic = rospy.ServiceProxy('/create_ppcom_topic', CreatePPComTopic)
    # Register the topic with ppcom router
    response = create_ppcom_topic(namespace, ['all'], '/'+namespace+'/occupancy_coords', 'sensor_msgs', 'PointCloud2')
    if namespace == 'jurong':
        response = create_ppcom_topic(namespace, ['raffles'], '/'+namespace+'/command/update', 'std_msgs', 'Bool')
    else:
        response = create_ppcom_topic(namespace, ['jurong'], '/'+namespace+'/command/update', 'std_msgs', 'Bool')


    # log_info(response)
    #response = create_ppcom_topic(namespace, ['all'], '/octomap_binary', 'kios_solution', 'area')
    # Create the publisher
    #msg_pub = rospy.Publisher('/world_coords', area, queue_size=1)


    # Get inspection area details
    log_info("Waiting for area details")
    area_details = rospy.wait_for_message("/world_coords/"+namespace, area)
    num_of_nodes = int(area_details.size.x * area_details.size.y * area_details.size.z)

    xrange = range(int(area_details.minPoint.x + area_details.resolution.data/2), int(area_details.minPoint.x + area_details.size.x * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    yrange = range(int(area_details.minPoint.y + area_details.resolution.data/2), int(area_details.minPoint.y + area_details.size.y * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    zrange = range(int(area_details.minPoint.z + area_details.resolution.data/2), int(area_details.minPoint.z + area_details.size.z * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 

    # Constructing the graph
    log_info("Constructing initial graph")
    nodes = [i for i in range(num_of_nodes)]
    #nodes_int = [i for i in range(num_of_nodes)]
    coordinates = [(x,y,z) for x in xrange for y in yrange for z in zrange]
    start = time.time()
    adjacency_org = constuct_adjacency(area_details, coordinates)
    duration = time.time() - start
    log_info("Adjacency Build Time: " +  str(duration) + "s")

    

    # Get obstacles' coordinates
    # log_info("Clearing agent position")
    # try:
    #     clear_agent_box(area_details.resolution.data, namespace)
    # except:
    #     pass
    log_info("Waiting for map")
    # binary_map = rospy.wait_for_message('/'+namespace+'/octomap_binary', Octomap)
    # log_info("Translating map")
    # get_coords = rospy.ServiceProxy('/octomap_to_coords', OctomapToCoords)
    # occupancy_coords = get_coords(octomap=binary_map)
    occupancy_coords = rospy.wait_for_message('/'+namespace+'/octomap_point_cloud_centers', PointCloud2)


    adjacency = update_adjacency(adjacency_org, coordinates, occupancy_coords)
    publish_graph_viz(viz_pub, coordinates, adjacency)

    waypoint = (odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z)
    #publish_graph_viz(viz_pub, coordinates, adjacency_org)
    # path planning and execution
    log_info("Waiting for target point")
    try:
        rospy.wait_for_message("/"+namespace+"/command/targetPoint", Point)
        #take_off()
        # one_sec = rospy.Duration(5)
        # rospy.sleep(one_sec)
        #target =  closest_node_index((target_point.x,target_point.y,target_point.z),coordinates)
    except rospy.exceptions.ROSException as e:
        log_info("Waiting for target point TIMEOUT")
        target =  closest_node_index((odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z),coordinates)
        
    # create thread
    waypoint = (-3000,-3000,-3000)
    command_thread = threading.Thread(target=go_to_point)
    command_thread.start()

    update_from_neighbor_thread = threading.Thread(target=update_from_neighbor, args=(coordinates,))
    update_from_neighbor_thread.start()

    
    #np.savetxt("/home/dronesteam/ws_caric/"+namespace+"_cost_matrix.csv", adjacency, delimiter=",")

    while not rospy.is_shutdown():
        agent_index = closest_node_index_1(([odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z]),coordinates)
        #log_info("Generating path. Starting Point: " + str(agent_index) + " Target Point: " + str(target))
        path = dijkstra(adjacency, arrival_pub, agent_index, target)
        #log_info("Going to point: " + str(coordinates[path[1]]))
        waypoint = coordinates[path[1]]
        #while (euclidean_distance_3d([odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z],coordinates[path[1]]) > area_details.resolution.data/4):
        #        go_to_point(coordinates[path[1]])
        #        rate.sleep()

        
        # Get obstacles' coordinates
        # try:
        #     clear_agent_box(area_details.resolution.data, namespace)
        # except:
        #     pass

        
        # binary_map = rospy.wait_for_message('/'+namespace+'/octomap_binary', Octomap)
        # get_coords = rospy.ServiceProxy('/octomap_to_coords', OctomapToCoords)
        # occupancy_coords = get_coords(octomap=binary_map)
        occupancy_coords = rospy.wait_for_message('/'+namespace+'/octomap_point_cloud_centers', PointCloud2)
        occ_pub.publish(occupancy_coords)

        if update:
            log_info("Updating map")
            mutex.acquire()
            adjacency = update_adjacency(adjacency_org,coordinates, occupancy_coords)
            mutex.release()
        else:
            mutex.acquire()
            # adjacency = update_adjacency(adjacency_final,coordinates, occupancy_coords)
            adjacency = update_adjacency_with_neighbors(adjacency_final)
            mutex.release()

        publish_graph_viz(viz_pub, coordinates, adjacency)
        
        
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