### UAV FoV Code ###
#### Created By Kios ####
##### 16 Nov 2023 #####
import rospy
from std_msgs.msg import Header, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from kios_solution.msg import norms
import numpy as np

debug = False
TAG = ""
odom = Odometry()
position = Point()
target = 0
command_thread = None
cmdPub = None
normals_msg = None
grid_resolution = 6
namespace = "jurong"
target = 0


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

def closest_node_index(node, nodes):
    agent_pos = np.asarray((node.x, node.y, node.z))
    verticies = np.zeros((len(nodes),3))
    for i, point in enumerate(nodes):
        verticies[i,0] = point.x
        verticies[i,1] = point.y
        verticies[i,2] = point.z
    deltas = np.subtract(verticies, agent_pos)
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    return np.argmin(dist_2)

def vec_to_eurler(vector):
    global odom
    
    #vector = np.asarray((vector.x, vector.y, vector.z))
    #vector /= np.linalg.norm(vector)

    #print(agent_yaw)
    yaw = np.arctan2(-vector.y, -vector.x)

    pitch = np.arcsin(vector.z)



    # # calculate pitch
    # pitch = np.arcsin(-vector[2])
    #print(pitch)

    # # calculate yaw
    # yaw = np.arctan2(-vector[1], -vector[0])
    #print(yaw)


    return pitch, yaw

def targetCallback(msg):
    global target, normals_msg
    target_point = msg
    target =  closest_node_index(target_point,normals_msg.facet_mids)


def main():
    # init
    global grid_resolution, namespace, debug, odom, position, target, normals_msg
    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_resolution = rospy.get_param('grid_resolution')
        set_tag("[" + namespace.upper() + " FOV SCRIPT]: ")
        #rospy.loginfo(TAG + namespace)
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
    # subscribe to target point
    rospy.Subscriber("/"+namespace+"/command/targetPoint", Point, targetCallback)


    # Get inspection norms
    log_info("Waiting for normals details")
    normals_msg = rospy.wait_for_message("/norms/"+namespace, norms)#
    #log_info(normals_msg)


    # Create the publisher
    gmb_pub = rospy.Publisher('/'+namespace+'/command/gimbal', Twist, queue_size=1)
    yaw_pub = rospy.Publisher('/'+namespace+'/command/yaw', Float32, queue_size=1)
    gmb_cmd = Twist()
    gmb_cmd.linear.x = 1.0 # set gimbal to target position

    while not rospy.is_shutdown():
        ind = closest_node_index(position, normals_msg.facet_mids)
        #print(normals_msg.facet_mids[ind])
        normal_direction_position = normals_msg.normals[ind]
        normal_direction_target = normals_msg.normals[target]

        #print(normal_direction)
        pitch_pos, yaw_pos  = vec_to_eurler(normal_direction_position)
        pitch_tar, yaw_tar  = vec_to_eurler(normal_direction_target)


        pitch = (pitch_pos + pitch_tar) / 2

        yaw = (yaw_pos + yaw_tar) / 2
        #print("pos: ", yaw_pos, "  tar: ", yaw_tar, "  fin: ", yaw)

        # if abs(pitch) <= 0.5:
        #     pitch = 0.0
        # elif pitch > 0:
        #     pitch = 1.57
        # else:
        #     pitch = -1.57

        gmb_cmd.linear.y = pitch # pitch
        gmb_cmd.linear.z = 0.0
        yaw_msg = Float32()
        yaw_msg.data = yaw # yaw
    
        gmb_pub.publish(gmb_cmd)
        yaw_pub.publish(yaw_msg)
        #log_info(gmb_cmd)
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