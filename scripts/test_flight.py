from helper import *

odom = OdomMsg()

def odomCallback(msg):
	global odom
	odom = msg

def mapCallback(msg):
	global debug

	#if debug:
		#print(len(msg.data))

if __name__ == '__main__':

	global cmdPub
	try:
		namespace = rospy.get_param('namespace') # node_name/argsname
		scenario = rospy.get_param('scenario')
		debug = rospy.get_param('debug')
		print(namespace)
		rospy.loginfo("[TESTING FLIGHT SCRIPT: NAMESPACE]: " + namespace)
	except Exception as e:
		print("error: " + str(e))
		namespace = "jurong"
		scenario = 'mbs'
		debug = True
	
	rospy.init_node(namespace+'_testflight', anonymous=True)
	rate = rospy.Rate(10)
	
	#wait for simulation
	if debug:
		rospy.loginfo("[TESTING FLIGHT SCRIPT: WAITING FOR SERVICE]: " + namespace)
	rospy.wait_for_service("/gazebo/get_model_state")
	get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	rate.sleep()
	
	state = get_state(model_name=scenario)

	while state.status_message != "GetModelState: got properties":
		if debug:
			rospy.loginfo("[TESTING FLIGHT SCRIPT: GET STATE]: " + namespace)
		state = get_state(model_name=namespace)
		rate.sleep()

	# Subscribe to the ppcom topics
	rospy.Subscriber("/"+namespace+"/ground_truth/odometry", OdomMsg, odomCallback)
	rospy.Subscriber("/"+namespace+"/octomap_binary", Octomap, mapCallback)

	# Get Bounding Box Verticies
	bboxes = rospy.wait_for_message("/gcs/bounding_box_vertices/", PointCloud)
	min_max = process_boxes(bboxes)

	# Get Neighbor Positions
	neighbors = rospy.wait_for_message("/"+namespace+"/nbr_odom_cloud", PointCloud2)
	min_max = find_world_min_max(neighbors, min_max)
	if debug:
		print(min_max)
	
	# Publisher for command
	cmdPub = rospy.Publisher("/"+namespace+"/command/trajectory", MultiDOFJointTrajectory, queue_size=1)
	cmdPubCust = rospy.Publisher("/"+namespace+"/command/custom", Point, queue_size=1)
	
	# Wait before take-off
	for i in range(0,60):
		rate.sleep()

	first=True
	flag=True
	x=odom.pose.pose.position.x
	y=odom.pose.pose.position.y
	z=odom.pose.pose.position.z
	while flag:
		if namespace=="jurong":
			if first:
				x = min_max[0]
				y = min_max[3]
				z = min_max[5]/2.0
			else:
				x = min_max[1]
				y = min_max[3]
				z = min_max[5]/2.0
				#flag=False
		else:
			'''
			if first:
				x = min_max[1]
				y = min_max[3]
				z = min_max[5]/2.0
			else:
				x = min_max[0]
				y = min_max[3]
				z = min_max[5]/2.0
				#flag=False
			'''
			if first:
				x = min_max[1]
				y = min_max[2]
				z = min_max[5]+10
			else:
				x = min_max[0]
				y = min_max[2]
				z = min_max[5]/2.0
				#flag=False
			

		while(euclidean_distance([x,y,z],[odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z]) >= 1.5):
			#go_to_point(x=x,y=y, z=z)

			des = Point()
			des.x = x
			des.y = y
			des.z = z
			cmdPubCust.publish(des)
			
			rate.sleep()
		first = False
		
		
		
