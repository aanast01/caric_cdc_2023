### GCS LOGIC CODE ###
#### Created By Kios ####
##### 10 Nov 2023 #####
import rospy
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2
from nav_msgs.msg import Odometry
from caric_mission.srv import CreatePPComTopic
from kios_solution.msg import area, norms
from geometry_msgs.msg import Point
from scipy.spatial import Delaunay, ConvexHull
import numpy as np

debug = False
TAG = ""
grid_res = 6
odom = Odometry()

def set_tag(tag):
    global TAG
    TAG = tag

def log_info(info):
    global TAG
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

    minz = round(max(0,minz-relax))
    maxz = round(maxz+relax)
    if (abs(minz) + abs(maxz))%grid_res!=0:
            maxz += grid_res-(abs(minz) + abs(maxz))%grid_res
    #if debug:
        #print("\nneighbor x: " + str(x) + " y: " + str(y) + " z: " + str(z))
    return [minx, maxx, miny, maxy, minz, maxz]

def find_norms(bboxes):
    # 3D vertices
    vertices = np.zeros((len(bboxes.points),3))
    for i, point in enumerate(reversed(bboxes.points)):
        vertices[i,0] = point.x
        vertices[i,1] = point.y
        vertices[i,2] = point.z
        
    # Calculate Convex Hull
    # hull = ConvexHull(vertices)
    
    # # Extract facets and their normals
    # facets = []
    # facet_normals = []
    
    # for simplex in hull.simplices:
    #     facet = vertices[simplex]
    #     mid_x = np.mean([facet[0][0], facet[1][0], facet[2][0]])
    #     mid_y = np.mean([facet[0][1], facet[1][1], facet[2][1]])
    #     mid_z = np.mean([facet[0][2], facet[1][2], facet[2][2]])
    #     facet_mid = [mid_x, mid_y, mid_z]
    #     facets.append(facet_mid)
    #     # Calculate normal vector for each facet
    #     normal = np.cross(facet[1] - facet[0], facet[2] - facet[0])
    #     normal /= np.linalg.norm(normal)

    #     facet_normals.append(normal)

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

if __name__ == '__main__':
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
    # Create the publisher
    # coords pub
    msg_pub = rospy.Publisher('/world_coords', area, queue_size=1)
    # norm pub
    norm_pub = rospy.Publisher('/norms', norms, queue_size=1)

    # Get Bounding Box Verticies
    bboxes = rospy.wait_for_message("/gcs/bounding_box_vertices/", PointCloud)
    min_max = process_boxes(bboxes)

    [facets, normls] = find_norms(bboxes)
    norm_msg = norms()
    for facet, norm in zip(facets, normls):
        facet_mid = Point()
        facet_mid.x = facet[0]
        facet_mid.y = facet[1]
        facet_mid.z = facet[2]
        norm_msg.facet_mids.append(facet_mid)
        norm_point = Point()
        norm_point.x = norm[0]
        norm_point.y = norm[1]
        norm_point.z = norm[2]
        norm_msg.normals.append(norm_point)


    # Get Neighbor Positions
    neighbors = rospy.wait_for_message("/"+namespace+"/nbr_odom_cloud", PointCloud2)
    
    
    #print(min_max)
    # Calculate discretized world
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
    
    while not rospy.is_shutdown():
        rate.sleep()
        msg_pub.publish(area_msg)
        norm_pub.publish(norm_msg)
    #rospy.spin()


