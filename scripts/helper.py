import rospy
import tf2_ros
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2
from nav_msgs.msg import Odometry as OdomMsg
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion, Twist, Point32, Point
from octomap_msgs.msg import Octomap
from gazebo_msgs.srv import GetModelState
import math

#debug = False
TAG = ""

def set_tag(tag):
	global TAG
	TAG = tag

def log_info(info):
	global TAG
	rospy.loginfo(TAG + info)
	#print(TAG)

def find_world_min_max(msg, min_max):
	global debug
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

		#if debug:
			#print("\nneighbor x: " + str(x) + " y: " + str(y) + " z: " + str(z))
	return [minx, maxx, miny, maxy, minz, maxz]

def euclidean_distance(p1,p2):
	return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def go_to_point(x,y,z):
	global cmd_pub, odom
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
	rotation_msg.z = 3.14#math.tan(target_yaw/2.0)+.78
	rotation_msg.w = 3.14#math.tan(target_yaw/2.0)+.78
	
	
	velocities_msg.linear.x = min((x-odom.pose.pose.position.x) * 2.0,5.0)
	velocities_msg.linear.y = min((y-odom.pose.pose.position.y) * 2.0,5.0)
	velocities_msg.linear.z = min((z-odom.pose.pose.position.z) * 2.0,5.0)
	
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
	
	cmdPub.publish(trajset_msg)
	#rospy.loginfo(trajset_msg)

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

	if debug:
		print("min x: " + str(minx) + " max x: " + str(maxx) + "\nmin y: " + str(miny) + " max y: " + str(maxy) + "\nmin z: " + str(minz) + " max z: " + str(maxz))

	return [minx, maxx, miny, maxy, minz, maxz]
		#point_str = "x: " + str(point.x) + " y: " + str(point.y) + " z: " + str(point.z)
		#rospy.loginfo("[TESTING FLIGHT SCRIPT: BBOX]: " + point_str)
