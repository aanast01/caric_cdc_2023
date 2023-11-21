### GCS LOGIC CODE ###
#### Created By Kios ####
##### 10 Nov 2023 #####
import rospy
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2
from nav_msgs.msg import Odometry
from caric_mission.srv import CreatePPComTopic
from kios_solution.msg import area, norms, multiPoint
from geometry_msgs.msg import Point
from scipy.spatial import Delaunay
import numpy as np
import sys
import math

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

    
    return Set_S_destination

debug = False
TAG = ""
grid_res = 6
odom = Odometry()

def set_tag(tag):
    global TAG
    TAG = tag

def log_info(info):
    global TAG, debug
    if debug:
        rospy.loginfo(TAG + info)
    #print(TAG)

def odomCallback(msg):
    global odom
    odom = msg

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

def find_world_min_max(msg, min_max):
    minx = min_max[0]
    miny = min_max[2]
    minz = min_max[4]
    maxx = min_max[1]
    maxy = min_max[3]
    maxz = min_max[5]

    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
        if minx > point[0]:
            minx = point[0]
        if maxx < point[0]:
            maxx = point[0]

        if miny > point[1]:
            miny = point[1]
        if maxy < point[1]:
            maxy = point[1]

        if minz > point[2]:
            minz = point[2]
        if maxz < point[2]:
            maxz = point[2]

    if minx > odom.pose.pose.position.x:
        minx = odom.pose.pose.position.x
    if maxx < odom.pose.pose.position.x:
        maxx = odom.pose.pose.position.x

    if miny > odom.pose.pose.position.y:
        miny = odom.pose.pose.position.y
    if maxy < odom.pose.pose.position.y:
        maxy = odom.pose.pose.position.y

    if minz > odom.pose.pose.position.z:
        minz = odom.pose.pose.position.z
    if maxz < odom.pose.pose.position.z:
        maxz = odom.pose.pose.position.z

    relax = 5
    minx = round(minx-relax)
    maxx = round(maxx+relax)
    if (abs(minx) + abs(maxx))%grid_res!=0:
        maxx += grid_res-(abs(minx) + abs(maxx))%grid_res

    miny = round(miny-relax)
    maxy = round(maxy+relax)
    if (abs(miny) + abs(maxy))%grid_res!=0:
            maxy += grid_res-(abs(miny) + abs(maxy))%grid_res

    minz = round(max(2.0,minz-relax))
    maxz = round(maxz+relax)
    if (abs(minz) + abs(maxz))%grid_res!=0:
            maxz += grid_res-(abs(minz) + abs(maxz))%grid_res
    #if debug:
        #print("\nneighbor x: " + str(x) + " y: " + str(y) + " z: " + str(z))
    return [minx, maxx, miny, maxy, minz, maxz]

def find_norms(vertices):
    # 3D vertices
    # vertices = np.zeros((len(bboxes.points),3))
    # for i, point in enumerate(reversed(bboxes.points)):
    #     vertices[i,0] = point.x
    #     vertices[i,1] = point.y
    #     vertices[i,2] = point.z
        
    #print(vertices)
    # Create Delaunay triangulation
    DT = Delaunay(vertices)
    
    # Extract free boundary
    edges = DT.convex_hull  # Convex hull represents the free boundary in Delaunay triangulation

    # Calculate incenter of triangles
    #print(DT.simplices)
    triangles = vertices[edges]

    P = np.mean(triangles, axis=1)
    
    # Calculate face normals manually
    normals = []
    for i, triangle in enumerate(triangles):
        A, B, C = triangle
        AB = B - A
        AC = C - A
        normal = np.cross(AB, AC)
        normal /= np.linalg.norm(normal)
        if DT.find_simplex(P[i]+normal) >= 0:
            normals.append(-normal)
        else:
            normals.append(normal)
    
    F = np.array(normals)
 
    # Now P contains the triangle incenters and F contains the face normals

    # Display facets and their normals
    # print("Facets:")
    # for facet in facets:
    #     print(facet)
    
    # print("\nFacet Normals:")
    # for normal in facet_normals:
    #     print(normal)

    return [P, F]

def euclidean_distance_3d(p1,p2):
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def main():
    global debug
    # init
    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_res = rospy.get_param('grid_resolution')
        set_tag("[" + namespace.upper() + " SCRIPT]: ")
        #rospy.loginfo(TAG + namespace)
    except Exception as e:
        print(e)
        namespace = "gcs"
        scenario = 'mbs'
        debug = True
        set_tag("[" + namespace.upper() + " SCRIPT]: ")

    rospy.init_node(namespace, anonymous=True)
    log_info(namespace)

    rate = rospy.Rate(1)

    # wait for gazebo
    rospy.wait_for_service("/gazebo/get_model_state")
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rate.sleep()

    state = get_state(model_name=scenario)
    while state.status_message != "GetModelState: got properties":
        if debug:
            rospy.loginfo("[TESTING FLIGHT SCRIPT: GET STATE]: " + namespace)
        state = get_state(model_name=namespace)
        rate.sleep()
    
    #rospy.sleep(rospy.Duration(10))
        
    # subscribe to self topics
    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", Odometry, odomCallback)
   
    
    # Create a ppcom publisher
    # Wait for service to appear
    rospy.wait_for_service('/create_ppcom_topic')

    # Create a service proxy
    create_ppcom_topic = rospy.ServiceProxy('/create_ppcom_topic', CreatePPComTopic)
    # Register the topic with ppcom router
    response = create_ppcom_topic('gcs', ['all'], '/world_coords', 'kios_solution', 'area')
    response = create_ppcom_topic('gcs', ['all'], '/norms', 'kios_solution', 'norms')
    response = create_ppcom_topic('gcs', ['jurong'], '/jurong_waypoints', 'kios_solution', 'multiPoint')
    response = create_ppcom_topic('gcs', ['raffles'], '/raffles_waypoints', 'kios_solution', 'multiPoint')
    # Create the publisher
    # coords pub
    msg_pub = rospy.Publisher('/world_coords', area, queue_size=1)
    # norm pub
    norm_pub = rospy.Publisher('/norms', norms, queue_size=1)
    # Explorers' Waypoints
    jurong_waypoints_pub = rospy.Publisher('/jurong_waypoints',multiPoint, queue_size=1)
    raffles_waypoints_pub = rospy.Publisher('/raffles_waypoints',multiPoint, queue_size=1)

    # Get Bounding Box Verticies
    bboxes = rospy.wait_for_message("/gcs/bounding_box_vertices/", PointCloud)
    min_max = process_boxes(bboxes)

    # Get Neighbor Positions
    log_info("Waiting for neighbors positions")
    neighbors = rospy.wait_for_message("/"+namespace+"/nbr_odom_cloud", PointCloud2)
    
    #print(min_max)
    # Calculate discretized world
    log_info("Calculating discretized world size")
    min_max = find_world_min_max(neighbors, min_max)
    size_x = (abs(min_max[0]) + abs(min_max[1]))/grid_res
    size_y = (abs(min_max[2]) + abs(min_max[3]))/grid_res
    size_z = (abs(min_max[4]) + abs(min_max[5]))/grid_res

    area_msg = area()
    area_msg.minPoint.x = min_max[0]
    area_msg.minPoint.y = min_max[2]
    area_msg.minPoint.z = min_max[4]
    area_msg.size.x = size_x
    area_msg.size.y = size_y
    area_msg.size.z = size_z
    area_msg.resolution.data = grid_res
    msg_pub.publish(area_msg)
    #log_info(f"{area_msg}")



    # find inspection target points
    log_info("Calculating waypoints based on bounding boxes")
    neighbor_positions = sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)
    num_of_agents = 2
    if scenario == 'hangar':
        num_of_agents = 1

    num_of_nodes = len(bboxes.points) + num_of_agents

    bbox_points = np.zeros((int(len(bboxes.points)/8),8,3))
    facet_mids = []
    normal_vectors = []
    counter = 0
    for i in range(0,int(len(bboxes.points)/8)):
        for j in range(8):
            bbox_points[i,j,0] = bboxes.points[counter].x
            bbox_points[i,j,1] = bboxes.points[counter].y
            bbox_points[i,j,2] = bboxes.points[counter].z
            counter += 1

        # calculate norms
        [facets, normls] = find_norms(bbox_points[i])
        facet_mids.append(facets)
        normal_vectors.append(normls)

    # np.savetxt("/home/dronesteam/ws_caric/"+namespace+"_norms.csv",normls, delimiter=",")
    # np.savetxt("/home/dronesteam/ws_caric/"+namespace+"_facet_mids.csv",facets, delimiter=",")
    norm_msg = norms()
    for facet, norm in zip(facet_mids, normal_vectors):
        for i in range(12):
            facet_mid = Point()
            facet_mid.x = facet[i,0]
            facet_mid.y = facet[i,1]
            facet_mid.z = facet[i,2]
            norm_msg.facet_mids.append(facet_mid)
            norm_point = Point()
            norm_point.x = norm[i,0]
            norm_point.y = norm[i,1]
            norm_point.z = norm[i,2]
            norm_msg.normals.append(norm_point)
    
    
    #np.savetxt("/home/dronesteam/ws_caric/"+namespace+"_points.csv",bbox_points.reshape(int(len(bboxes.points)),3), delimiter=",")

    inspect_points = np.zeros((int(len(bboxes.points)/8),28,3))
    cleared_inspect_points = np.zeros((int(len(bboxes.points)/8),19,3))
    for box_i in range(0,int(len(bboxes.points)/8)):
        counter = 0
        for i in range(8):
            for j in range(i+1,8):
                inspect_points[box_i,counter,0] = round((bbox_points[box_i,i,0] + bbox_points[box_i,j,0])/2,4)
                inspect_points[box_i,counter,1] = round((bbox_points[box_i,i,1] + bbox_points[box_i,j,1])/2,4)
                inspect_points[box_i,counter,2] = round((bbox_points[box_i,i,2] + bbox_points[box_i,j,2])/2,4)
                counter += 1
        cleared_inspect_points[box_i] = np.unique(inspect_points[box_i], axis=0)

    cleared_inspect_points = np.delete(cleared_inspect_points,9, axis=1)
    # np.savetxt("/home/dronesteam/ws_caric/"+namespace+"_points.csv",cleared_inspect_points.reshape(int(len(bboxes.points)/8)*18,3), delimiter=",")
    # print(inspect_points)

    neighbor_points = np.zeros((num_of_agents,3))
    for i, point in enumerate(neighbor_positions):
        neighbor_points[i,0] = point[0]
        neighbor_points[i,1] = point[1]
        neighbor_points[i,2] = point[2]
        if i == num_of_agents-1:
            break
        
    points = np.concatenate((neighbor_points, cleared_inspect_points.reshape(int(len(bboxes.points)/8)*18,3)))

    log_info("Calculating cost matrix")
    adjacency = np.zeros((num_of_nodes,num_of_nodes))
    for i in range(num_of_nodes):
        for j in range(num_of_nodes):
            adjacency[i,j] = euclidean_distance_3d(points[i],points[j])

    # np.savetxt("/home/dronesteam/ws_caric/"+namespace+"_cost_matrix.csv", adjacency, delimiter=",")
    # np.savetxt("/home/dronesteam/ws_caric/"+namespace+"_drones.csv", [i for i in range(num_of_agents)], delimiter=",")

    log_info("Running mTSP")
    waypointsMatrix = calculateCircuits([i for i in range(num_of_agents)], num_of_nodes, adjacency)

    jurong_waypoints_msg = multiPoint()
    raffles_waypoints_msg = multiPoint()
    for i in range(num_of_agents):
        for waypoint in waypointsMatrix[i]:
            point = Point()
            point.x = points[waypoint,0]
            point.y = points[waypoint,1]
            point.z = points[waypoint,2]

            if i == 0:
                jurong_waypoints_msg.points.append(point)
            else:
                raffles_waypoints_msg.points.append(point)
            

    log_info("DONE")
    while not rospy.is_shutdown():
        rate.sleep()
        msg_pub.publish(area_msg)
        norm_pub.publish(norm_msg)
        jurong_waypoints_pub.publish(jurong_waypoints_msg)
        raffles_waypoints_pub.publish(raffles_waypoints_msg)
    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("terminating...")
    except Exception as e:
        print(e)
    finally:
        exit()