### GCS LOGIC CODE ###
#### Created By Kios ####
##### 10 Nov 2023 #####
import rospy
from std_msgs.msg import String
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2
from nav_msgs.msg import Odometry
from caric_mission.srv import CreatePPComTopic
from kios_solution.msg import area, norms, multiPoint
from geometry_msgs.msg import Point
from scipy.spatial import Delaunay, ConvexHull
import numpy as np
import sys
import math


def generate_points_on_cuboid_faces(vertices, num_points_per_face=10):
    # Convert vertices to a NumPy array for easier manipulation
    vertices = np.array(vertices)
 
    # Find the convex hull of the vertices
    hull = ConvexHull(vertices)
    DT = Delaunay(vertices)
    faces = hull.simplices  # Get the vertices of each face
 
    all_points = []
    all_norms = []

    for face in faces:
        # Extract the vertices of the current face
        current_face = vertices[face]
 
        # Calculate two edges of the face
        edge1 = current_face[1] - current_face[0]
        edge2 = current_face[2] - current_face[0]
 
        # Generate points on the current face using parametric equations
      
        u = np.linspace(0, 1, num_points_per_face)
        v = np.linspace(0, 1, num_points_per_face)
        u, v = np.meshgrid(u, v)
        
        normal = find_norm_from_face(DT, current_face)
        face_points = []
        face_norms = []
        for i in range(1, num_points_per_face-1):
            for j in range(1, num_points_per_face-i-1):
                # Calculate the point on the face using parametric equations
                point_on_face = current_face[0] + u[i][j] * edge2 + v[i][j] * edge1
                face_points.append(point_on_face)
                face_norms.append(normal)
        

        all_points.extend(face_points)
        all_norms.extend(face_norms)

    all_points = np.array(all_points)
    all_points, ind = np.unique(all_points,axis=0, return_index=True)
    all_norms = np.array(all_norms)
    all_norms = all_norms[ind]
    # np.savetxt("/home/dronesteam/ws_caric/_points.csv",all_points, delimiter=",")
    # np.savetxt("/home/dronesteam/ws_caric/_norms.csv",all_norms, delimiter=",")
 
    return all_points, all_norms

def find_norm_from_face(DT, current_face):

    # Calculate face normals manually
    A, B, C = current_face
    AB = B - A
    AC = C - A
    center = np.mean(current_face, axis=0)
    normal = np.cross(AB, AC)
    normal /= np.linalg.norm(normal)
    if DT.find_simplex(center+normal) >= 0:
        normal = (-normal)
    else:
        normal = normal

    # if scenario != 'hangar':
    #     remove_indicies = np.where(P[:,2] <= 5)[0]
    #     P = np.delete(P,remove_indicies, axis=0)
    #     F = np.delete(F,remove_indicies, axis=0)


    return normal

def check_point_inside_cuboid(vertices, point):
    # Extract x, y, z coordinates of the vertices
    x_coords, y_coords, z_coords = zip(*vertices)
    # Determine the boundaries
    min_x = min(x_coords)+5
    max_x = max(x_coords)-5
    min_y = min(y_coords)+5
    max_y = max(y_coords)-5
    min_z = min(z_coords)+5
    max_z = max(z_coords)-5
    # Check if the point is inside the cuboid
    x_p, y_p, z_p = point
    if min_x < x_p < max_x and min_y < y_p < max_y and min_z < z_p < max_z:
        return True
    else:
        return False

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

    # if minx > odom.pose.pose.position.x:
    #     minx = odom.pose.pose.position.x
    # if maxx < odom.pose.pose.position.x:
    #     maxx = odom.pose.pose.position.x

    # if miny > odom.pose.pose.position.y:
    #     miny = odom.pose.pose.position.y
    # if maxy < odom.pose.pose.position.y:
    #     maxy = odom.pose.pose.position.y

    # if minz > odom.pose.pose.position.z:
    #     minz = odom.pose.pose.position.z
    # if maxz < odom.pose.pose.position.z:
    #     maxz = odom.pose.pose.position.z

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

    if scenario != 'hangar':
        remove_indicies = np.where(P[:,2] <= 5)[0]
        P = np.delete(P,remove_indicies, axis=0)
        F = np.delete(F,remove_indicies, axis=0)


    return [P, F]

def euclidean_distance_3d(p1,p2):
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def main():
    global debug, scenario
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
    response = create_ppcom_topic('gcs', ['all'], '/waypoints', 'std_msgs', 'String')
    # response = create_ppcom_topic('gcs', ['jurong'], '/jurong_waypoints', 'kios_solution', 'multiPoint')
    # response = create_ppcom_topic('gcs', ['raffles'], '/raffles_waypoints', 'kios_solution', 'multiPoint')
    # Create the publisher
    # coords pub
    msg_pub = rospy.Publisher('/world_coords', area, queue_size=1)
    # norm pub
    norm_pub = rospy.Publisher('/norms', norms, queue_size=1)
    # Explorers' Waypoints
    # jurong_waypoints_pub = rospy.Publisher('/jurong_waypoints',multiPoint, queue_size=1)
    # raffles_waypoints_pub = rospy.Publisher('/raffles_waypoints',multiPoint, queue_size=1)
    waypoints_pub = rospy.Publisher('/waypoints',String, queue_size=1)

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

    # find inspection target points
    log_info("Calculating waypoints based on bounding boxes")

    bbox_points = np.zeros((int(len(bboxes.points)/8),8,3))
    counter = 0
    for i in range(0,int(len(bboxes.points)/8)):
        for j in range(8):
            bbox_points[i,j,0] = bboxes.points[counter].x
            bbox_points[i,j,1] = bboxes.points[counter].y
            bbox_points[i,j,2] = bboxes.points[counter].z
            counter += 1

    #create interest points
    all_points = np.empty((0,3))
    all_norms = np.empty((0,3))
    for box_i in range(0,int(len(bboxes.points)/8)):
        all_box_points,all_box_norms = generate_points_on_cuboid_faces(bbox_points[box_i],8)

        all_points = np.append(all_points, all_box_points, axis=0)
        all_norms = np.append(all_norms, all_box_norms, axis=0)

    delete_index = np.empty((0,1))
    for box_i in range(0,int(len(bboxes.points)/8)):
        DT = Delaunay(bbox_points[box_i])
        for i in range(all_points.shape[0]):
            if DT.find_simplex(all_points[i] + all_norms[i]) >= 0:
                delete_index = np.append(delete_index, [i])
        delete_index = delete_index.astype(int)
    if delete_index.size > 0:
        all_points = np.delete(all_points, delete_index, axis=0)
        all_norms = np.delete(all_norms, delete_index, axis=0)
    
    np.savetxt("/home/dronesteam/ws_caric/_points.csv",all_points, delimiter=",")
    np.savetxt("/home/dronesteam/ws_caric/_norms.csv",all_norms, delimiter=",")

    norm_msg = norms()
    for i in range(all_points.shape[0]):
        facet_mid = Point()
        facet_mid.x = all_points[i,0]
        facet_mid.y = all_points[i,1]
        facet_mid.z = all_points[i,2]
        norm_msg.facet_mids.append(facet_mid)
        norm_point = Point()
        norm_point.x = all_norms[i,0]
        norm_point.y = all_norms[i,1]
        norm_point.z = all_norms[i,2]
        norm_msg.normals.append(norm_point)
    
    # #np.savetxt("/home/dronesteam/ws_caric/"+namespace+"_facet_mids_pruned.csv",facet_mids, delimiter=",")
    # log_info("Saving file")
    # np.savetxt("/home/dronesteam/ws_caric/"+namespace+"_points.csv",all_points, delimiter=",")
 
    log_info("Sending waypoints")
 
    filename = "./"+namespace+"_waypoints.csv"
    np.savetxt(filename, all_points, delimiter=",")

    # # TODO: delete this
    # filename = "./"+scenario+"_waypoints.csv"
    # np.savetxt(filename, cleared_inspect_points.reshape(int(len(bboxes.points)/8)*18,3), delimiter=",")

    str_msg = String()
    str_msg.data = filename

    log_info("DONE")
    while not rospy.is_shutdown():
        rate.sleep()
        msg_pub.publish(area_msg)
        norm_pub.publish(norm_msg)
        waypoints_pub.publish(str_msg)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        filename = "./gcs_waypoints.csv"
        import os
        os.remove(filename)
        print("terminating...")
    except Exception as e:
        print(e)
    finally:
        exit()