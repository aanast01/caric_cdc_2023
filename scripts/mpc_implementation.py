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

dt = 3

A = np.array([
    [1, 0, 0, dt,  0,  0],
    [0, 1, 0,  0, dt,  0],
    [0, 0, 1,  0,  0, dt],
    [0, 0, 0,  1,  0,  0],
    [0, 0, 0,  0,  1,  0],
    [0, 0, 0,  0,  0,  1]
    ])

B = np.array([
    [dt,  0,  0],
    [ 0, dt,  0],
    [ 0,  0, dt],
    [ 0,  0,  0],
    [ 0,  0,  0],
    [ 0,  0,  0]
    ])


def euclidean_distance(p1,p2):
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def go_to_point():
    global cmd_pub, odom, u, rate
    while not rospy.is_shutdown():
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

def send_command():
    global cmd_pub, odom, u, send_count
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
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
        
        velocities_msg.linear.x = u[0] #min((x-odom.pose.pose.position.x) * 2.0,5.0)
        velocities_msg.linear.y = u[1] #min((y-odom.pose.pose.position.y) * 2.0,5.0)
        velocities_msg.linear.z = u[2] #min((z-odom.pose.pose.position.z) * 2.0,5.0)
        
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
        

        if ((abs(u[0]) + abs(u[1]) + abs(u[2]))!=0):# and (send_count <= 2)):
            cmdPub.publish(trajset_msg)
            send_count = send_count+1
        rate.sleep()
        #rospy.loginfo(trajset_msg)

def mpc_control(occupancy_coords, desired_p):
    global A, B, dt, odom, currVels
    print("Start MPC")
    # horizon
    N=2
    # safety threshold
    d_s = 5.0
    # control bounds
    u_lb = [-1.0,-1.0,-1.0]
    u_ub = [1.0,1.0,1.0]

    # state bounds
    p_lb = [-gp.GRB.INFINITY,-gp.GRB.INFINITY,0.16,-2.0,-2.0,-1.0]
    p_ub = [gp.GRB.INFINITY,gp.GRB.INFINITY,100,2.0,2.0,1.0]

    # create new model
    model = gp.Model("mpc")
    model.setParam("OutputFlag",1)
    model.setParam("LogToConsole",1)

    # Create variables
    u = {}          # Control inputs
    p = {}          # State variables
    voxel = {}      # Obstacles variable
    #desired_p = {}  # Desired variable
    dist = {}       # Distance variable
    abs_dist = {}   # Absolute distance variable
    d_dist = {}       # Distance variable for desired
    d_abs_dist = {}   # Absolute distance variable for desired

    print("Start Adding Vars")
    for k in range(0,N):
        u[k] = model.addMVar(3,lb=u_lb, ub=u_ub, name=f"u_{k}")
        p[k] = model.addMVar(6,lb=p_lb, ub=p_ub, name=f"p_{k}")
    print("Update 1: " + rospy.get_param('namespace'))
    model.update()

    #for k in range(0,len(occupancy_coords.points)):
    #    voxel[k] = model.addVars(3,lb=[-gp.GRB.INFINITY,-gp.GRB.INFINITY,-gp.GRB.INFINITY], ub=[gp.GRB.INFINITY,gp.GRB.INFINITY,gp.GRB.INFINITY], name=f"voxel_{k}")
    #    voxel[k] = [occupancy_coords.points[k].x,occupancy_coords.points[k].y,occupancy_coords.points[k].z]
    '''
    for v in range(0,len(occupancy_coords.points)*(N-1)):
        dist[v] = model.addMVar(3,lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY, name=f"dist{v}")
        abs_dist[v] = model.addMVar(3,lb=0, ub=gp.GRB.INFINITY, name=f"abs_dist{v}")

    print("Update 2: " + rospy.get_param('namespace'))
    model.update()
    '''
    #desired_p = model.addVar(lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY, name="desired_p")

    #print(p[0])

    # Add system dynamics constraints
    for k in range(0, N-1):
        model.addConstr((p[k+1] ==  A @ p[k] + B @ u[k]), name=f"dyn_{k}")

    '''
            #for i in range(0,6):
                #model.addConstr((p[k+1][i] ==  A[i] * p[k] + B[i] * u[k]), name=f"dyn_{k}_y")

                #Ap[i] = Ap[i] + (A[i,j] * p[k][j])
                #print(Ap[i])
                #Bu[i] = sum([B[i,j] * u[k][j] for j in range(3)])
                #model.addConstr((p[k+1][i] ==  sum([Ap[i], Bu[i]])), name=f"dyn_{k}_{i}") # + B[0,i] @ u[k]
                #model.addConstr((p[k+1][i] ==  sum([A[i,j] * p[k][j] for j in range(6)]) + sum([B[i,j] * u[k][j] for j in range(3)])), name=f"dyn_{k}_{i}")
            #model.addConstr((p[k+1][1] ==  Ap[1] + Bu[1]), name=f"dyn_{k}_y") # + B[0,i] @ u[k]
            #model.addConstr((p[k+1][2] ==  Ap[2] + Bu[2]), name=f"dyn_{k}_z") # + B[0,i] @ u[k]
            #model.addConstr((p[k+1][3] ==  Ap[3] + Bu[3]), name=f"dyn_{k}_vx") # + B[0,i] @ u[k]
            #model.addConstr((p[k+1][4] ==  Ap[4] + Bu[4]), name=f"dyn_{k}_vy") # + B[0,i] @ u[k]
            #model.addConstr((p[k+1][5] ==  Ap[5] + Bu[5]), name=f"dyn_{k}_vz") # + B[0,i] @ u[k]
    '''
    print("Update 3: " + rospy.get_param('namespace'))
    model.update()

    index = 0
    for voxel in occupancy_coords.points:
        for k in range(0,N-1):
            dist[index] = model.addMVar(3,lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY, name=f"dist{index}")
            abs_dist[index] = model.addMVar(3,lb=0, ub=gp.GRB.INFINITY, name=f"abs_dist{index}")
            model.addConstr(dist[index][0] == (p[k][0] - voxel.x), name=f"safety_voxel_x_{index}_{k}")
            model.addConstr(dist[index][1] == (p[k][1] - voxel.y), name=f"safety_voxel_y_{index}_{k}")
            model.addConstr(dist[index][2] == (p[k][2] - voxel.z), name=f"safety_voxel_z_{index}_{k}")
            model.addGenConstrAbs(abs_dist[index][0], dist[index][0], name=f"absdist_x_{index}_{k}")
            model.addGenConstrAbs(abs_dist[index][1], dist[index][1], name=f"absdist_y_{index}_{k}")
            model.addGenConstrAbs(abs_dist[index][2], dist[index][2], name=f"absdist_z_{index}_{k}")
            model.addConstr( d_s <= abs_dist[index][0] + abs_dist[index][1] + abs_dist[index][2], name=f"abs_safety_voxel_{index}_{k}")
            index = index + 1
            #print(index)
    '''
                #model.addConstr(dist[index][0] == (p[k][0] - occupancy_coords.points[v].x), name=f"safety_voxel_x_{v}_{k}") #voxel[v][0]
                #model.addConstr(dist[index][1] == (p[k][1] - occupancy_coords.points[v].y), name=f"safety_voxel_y_{v}_{k}")
                #model.addConstr(dist[index][2] == (p[k][2] - occupancy_coords.points[v].z), name=f"safety_voxel_z_{v}_{k}")
                #model.addGenConstrAbs(abs_dist[index], dist[index], name=f"absdist_{v}_{k}")
                #model.addGenConstrAbs(abs_dist[index][1], dist[index][1], name=f"absdist_y_{v}_{k}")
                #model.addGenConstrAbs(abs_dist[index][2], dist[index][2], name=f"absdist_z_{v}_{k}")
                #model.addConstr( d_s <= sum([abs_dist[index][i] for i in range(3)]), name=f"abs_safety_voxel_{v}_{k}")
    '''         
    print("Update 4: " + rospy.get_param('namespace'))
    model.update()

    index = 0
    #obj = 0
    print(desired_p)
    for k in range(0,N-1):
        d_dist[index] = model.addMVar(3,lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY, name=f"d_dist{index}")
        d_abs_dist[index] = model.addMVar(3,lb=0, ub=gp.GRB.INFINITY, name=f"d_abs_dist{index}")
        model.addConstr(d_dist[index][0] == (p[k][0] - desired_p[0]), name=f"d_des_x_{index}_{k}")
        model.addConstr(d_dist[index][1] == (p[k][1] - desired_p[1]), name=f"d_des_y_{index}_{k}")
        model.addConstr(d_dist[index][2] == (p[k][2] - desired_p[2]), name=f"d_des_z_{index}_{k}")
        model.addGenConstrAbs(d_abs_dist[index][0], d_dist[index][0], name=f"absd_des_x_{index}_{k}")
        model.addGenConstrAbs(d_abs_dist[index][1], d_dist[index][1], name=f"absd_des_y_{index}_{k}")
        model.addGenConstrAbs(d_abs_dist[index][2], d_dist[index][2], name=f"absd_des_z_{index}_{k}")
        #model.addConstr( d_s <= abs_dist[index][0] + abs_dist[index][1] + abs_dist[index][2], name=f"abs_safety_voxel_{index}_{k}")
        index = index + 1

    obj = model.addVar(lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY, name=f"obj")
    model.addConstr(obj == sum([sum([d_abs_dist[i][0] for i in range(N-1)]),sum([d_abs_dist[i][1] for i in range(N-1)]),sum([d_abs_dist[i][2] for i in range(N-1)])]), name=f"obj_const")

    model.setObjective(obj, GRB.MINIMIZE)

    print("Update 5: " + rospy.get_param('namespace'))
    model.update()

    print(model.getConstrByName("obj_const"))

    currVels = [odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z]
    model.addConstr( p[0][0] == odom.pose.pose.position.x, name=f"init_pos_0")
    model.addConstr( p[0][1] == odom.pose.pose.position.y, name=f"init_pos_1")
    model.addConstr( p[0][2] == odom.pose.pose.position.z, name=f"init_pos_2")
    model.addConstr( p[0][3] == odom.twist.twist.linear.x, name=f"init_pos_3")
    model.addConstr( p[0][4] == odom.twist.twist.linear.y, name=f"init_pos_4")
    model.addConstr( p[0][5] == odom.twist.twist.linear.z, name=f"init_pos_5")

    print("Update 6: " + rospy.get_param('namespace'))
    model.update()

    print("OPTIMIZE: " + rospy.get_param('namespace'))
    model.optimize()

    # Access the solution
    if model.status == gp.GRB.OPTIMAL:
        #print("Optimal solution found")
        #print("0: ",[model.getVarByName("u_0[0]").x,model.getVarByName("u_0[1]").x,model.getVarByName("u_0[2]").x])
        #print("1: ",[model.getVarByName("u_1[0]").x,model.getVarByName("u_1[1]").x,model.getVarByName("u_1[2]").x])
        #print("2: ",[model.getVarByName("u_2[0]").x,model.getVarByName("u_2[1]").x,model.getVarByName("u_2[2]").x])

        return [model.getVarByName("u_0[0]").x,model.getVarByName("u_0[1]").x,model.getVarByName("u_0[2]").x]
    else:
        iis = model.computeIIS()
        constr_index = model.IISConstr.index(1)
        print(model.getConstrs()[constr_index])
        print("Optimal solution not found.")
        return [0,0,0]
    
def odomCallback(msg):
    global odom
    odom = msg

def desCallback(msg):
    global x_d
    x_d = [msg.x, msg.y, msg.z]

def octomap_callback(msg):
    pass

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

    rospy.init_node(namespace+'_mpc_controller', anonymous=True)
    rate = rospy.Rate(30)

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

    # create thread
    command_thread = threading.Thread(target=send_command)
    u = [0,0,0]
    currVels = [1,1,1]
    #x_d = [20, -20, 35]
    command_thread.start()


    while True:#(euclidean_distance(x_d,[odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z]) >= 1.5):
        binary_map = rospy.wait_for_message('/'+namespace+'/octomap_binary', Octomap)
        get_coords = rospy.ServiceProxy('/octomap_to_coords', OctomapToCoords)
        occupancy_coords = get_coords(octomap=binary_map)
        start = rospy.Time.now().secs
        u = mpc_control(occupancy_coords, x_d)
        send_count = 0 
        #u = [currVels[0] + temp_u[0], currVels[1] + temp_u[1], currVels[2] + temp_u[2]]
        elapsed = (rospy.Time.now().secs-start)
        print("elaplsed time: ", elapsed)
        rate.sleep()

    u = [0,0,0]
    command_thread.terminate()
    #rospy.spin()

if __name__ == '__main__':
    try:
        print("Ksekino")
        main()
    except KeyboardInterrupt:
        print("terminating...")
        command_thread.terminate()
    except Exception as e:
        print(e)
    finally:
        exit()