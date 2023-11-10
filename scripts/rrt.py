import random
import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Header
from octomap_msgs.msg import Octomap
from caric_mission.srv import OctomapToCoords
from nav_msgs.msg import Odometry as OdomMsg
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion, Twist, Point32, Point
import gurobipy as gp
from gurobipy import GRB
import numpy as np
import scipy.sparse as sp
import math
import threading


odom = OdomMsg()

# Function to find the nearest node in the tree to a random sample
def nearest_node(random_sample, tree):
    min_distance = float("inf")
    nearest = None
    for node in tree:
        distance = np.linalg.norm(node - random_sample)
        if distance < min_distance:
            min_distance = distance
            nearest = node
    return nearest
 
# Function to generate a new node in the direction of the random sample
def new_node(nearest, random_sample, step_size):
    direction = random_sample - nearest
    magnitude = np.linalg.norm(direction)
    if magnitude <= step_size:
        return random_sample
    else:
        unit_vector = direction / magnitude
        return nearest + step_size * unit_vector

def rrt(occupancy_coords, x_d):
    global odom
    # Define your environment: point A, point B, and a list of 3D obstacles
    point_A = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
    point_B = point_A + np.array([10,10,5])#np.array([x_d[0], x_d[1], x_d[2]])
    #obstacles = [np.array([2, 2, 2]), np.array([3, 3, 3]), np.array([4, 4, 4])]
     
    # Define RRT parameters
    num_nodes = 2500
    step_size = 1.0
     
    # Initialize the tree with the starting point
    tree = [point_A]
    
    print("ksekino rrt")
    # RRT algorithm
    for _ in range(num_nodes):
        random_sample = np.array([random.uniform(0, 50), random.uniform(0, 50), random.uniform(0, 20)])
        nearest = nearest_node(random_sample, tree)
        new_node_candidate = new_node(nearest, random_sample, step_size)
        # Check if the new node is collision-free
        collision_free = True
        for obstacle in occupancy_coords.points:
            obstacle_position = np.array([obstacle.x, obstacle.y, obstacle.z])
            if np.linalg.norm(new_node_candidate - obstacle_position) < 1.0:  # Assuming obstacle radius
                collision_free = False
                break
        if collision_free:
            tree.append(new_node_candidate)
    
    print("Etelepsa rrt")
    # Find the path from point A to the nearest node to point B
    nearest_to_B = nearest_node(point_B, tree)
    path = [nearest_to_B]
    print("ivra ton proto")

    while not np.array_equal(path[-1], point_A):
        parent = None
        for node in tree:
            if np.array_equal(node, path[-1]):
                parent = node
                break
        path.append(parent)
     
    # Reverse the path for plotting
    path = path[::-1]

    print("Pempo lisi")
    return [path[0][0],path[0][1],path[0][2]]
    '''     
    # Plot the environment, tree, and path
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(point_A[0], point_A[1], point_A[2], c='g', marker='o', label='Start')
    ax.scatter(point_B[0], point_B[1], point_B[2], c='r', marker='o', label='Goal')
    for obstacle in obstacles:
        ax.scatter(obstacle[0], obstacle[1], obstacle[2], c='k', marker='x', label='Obstacle')
    for node in tree:
        ax.scatter(node[0], node[1], node[2], c='b', marker='.', label='Tree Node')
    for i in range(len(path) - 1):
        ax.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], [path[i][2], path[i + 1][2]], c='b')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.legend()
    plt.show()
    '''

def euclidean_distance(p1,p2):
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def go_to_point():
    global cmd_pub, odom, u, rate
    print("pao sto:", u)
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
    rotation_msg.z = 0.0
    rotation_msg.w = 0.0
    
    
    velocities_msg.linear.x = min((u[0]-odom.pose.pose.position.x) * 2.0,5.0)
    velocities_msg.linear.y = min((u[1]-odom.pose.pose.position.y) * 2.0,5.0)
    velocities_msg.linear.z = min((u[2]-odom.pose.pose.position.z) * 2.0,5.0)
    
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
    
    #print(u)
    if ( abs(u[0]) + abs(u[1]) + abs(u[2]))!=0:
        cmdPub.publish(trajset_msg)
    rate.sleep()
    #rospy.loginfo(trajset_msg)


def odomCallback(msg):
    global odom
    odom = msg

def desCallback(msg):
    global x_d
    x_d = [msg.x, msg.y, msg.z]

def main():
    global cmdPub, debug, u, rate, command_thread, x_d, currVels, send_count

    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        print(namespace)
        rospy.loginfo("[MPC FLIGHT SCRIPT: NAMESPACE]: " + namespace)
    except Exception as e:
        print("error: " + str(e))
        namespace = "jurong"
        scenario = 'mbs'
        debug = True

    rospy.init_node(namespace+'_rrt_controller', anonymous=True)
    rate = rospy.Rate(10)

    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", OdomMsg, odomCallback)

    #x_d = [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z]
    rospy.Subscriber("/"+namespace+"/command/custom", Point, desCallback)
    rospy.wait_for_message("/"+namespace+"/command/custom", Point)
    print("Epiasa command")

    # Publisher for command
    cmdPub = rospy.Publisher("/"+namespace+"/command/trajectory", MultiDOFJointTrajectory, queue_size=1)

    #rospy.Subscriber('/jurong/octomap_binary', OctomapBinary, octomap_callback)
    binary_map = rospy.wait_for_message('/'+namespace+'/octomap_binary', Octomap)
    get_coords = rospy.ServiceProxy('/octomap_to_coords', OctomapToCoords)
    occupancy_coords = get_coords(octomap=binary_map)

    while (euclidean_distance(x_d,[odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z]) >= 1.5):
        binary_map = rospy.wait_for_message('/'+namespace+'/octomap_binary', Octomap)
        get_coords = rospy.ServiceProxy('/octomap_to_coords', OctomapToCoords)
        occupancy_coords = get_coords(octomap=binary_map)
        print("Epiasa xarti: ", len(occupancy_coords.points))

        start = rospy.Time.now().secs
        u = rrt(occupancy_coords, x_d)
        elapsed = (rospy.Time.now().secs-start)
        print("elaplsed time: ", elapsed)
        #go_to_point()
        rate.sleep()


if __name__ == '__main__':
    try:
        print("Ksekino")
        main()
    except KeyboardInterrupt:
        print("terminating...")
    except Exception as e:
        print(e)
    finally:
        exit()