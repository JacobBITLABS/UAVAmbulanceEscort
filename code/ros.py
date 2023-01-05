#!/bin/python3
import rospy, time
import mavros
from enum import Enum
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
	global current_state
	current_state = msg


#Lat,long,alt
class Point:
	x = 0.0
	y = 0.0
	z = 0.0
	def __init__(self, x,y,z):
		self.x = x
		self.y = y
		self.z = z

#A waypoint
class Waypoint:
	pos = Point(0,0,0)
	stop = False
	deadline = 0
	wait = 30
	def __init__(self, point, stop, deadline=time.time() + 60, wait=30):
		self.point = point
		self.stop = stop
		self.deadline = deadline
		self.wait = 30

#State of the drone
class DState(Enum):
	Start = 1 #Startup sequence
	Travel = 2 #Normal travel
	TravelWait = 3 #Is decending to waypoint
	Wait = 4 #Is at a waypoint
	End = 5 #At the end

def set_follow_me_target(target_position, target_velocity):
	"""
	Set the "follow me" target for the drone.
	
	Parameters:
	target_position -- the position of the target (tuple of 3 floats: (x, y, z))
	target_velocity -- the velocity of the target (tuple of 3 floats: (vx, vy, vz))
	"""
	# Create a PositionTarget message
	msg = PositionTarget()
	
	# Set the target position and velocity
	msg.position.x = target_position[0]
	msg.position.y = target_position[1]
	msg.position.z = target_position[2]
	msg.velocity.x = target_velocity[0]
	msg.velocity.y = target_velocity[1]
	msg.velocity.z = target_velocity[2]
	
	# Set the coordinate frame and type mask
	msg.coordinate_frame = mavros.setpoint.FRAME_LOCAL_NED
	msg.type_mask = mavros.setpoint.IGNORE_VX + mavros.setpoint.IGNORE_VY + mavros.setpoint.IGNORE_VZ + mavros.setpoint.IGNORE_AFX + mavros.setpoint.IGNORE_AFY + mavros.setpoint.IGNORE_AFZ + mavros.setpoint.FORCE
	
	# Publish the message
	return msg

#TODO: Beautify this. This is derived from an example, move it to a class and make it nicer that way
#TODO: MAVROS cross drone communication
#TODO: Load waypoints
#TODO: Reset waypoints (update if times change etc.)
#TODO: Follow mode if behind another drone
if __name__ == "__main__":
	rospy.init_node("offb_node_py")

	state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

	local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
	
	rospy.wait_for_service("/mavros/cmd/arming")
	arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

	rospy.wait_for_service("/mavros/set_mode")
	set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
	

	# Setpoint publishing MUST be faster than 2Hz
	rate = rospy.Rate(20)

	# Wait for Flight Controller connection
	while(not rospy.is_shutdown() and not current_state.connected):
		rate.sleep()

	pose = PoseStamped()

	#
	state = DState.Travel
	path = [
		Waypoint(Point(0,0,2), True, deadline=time.time()+60, wait=5),
		Waypoint(Point(5,5,5), True, deadline=time.time()+120)
	]

	offset_height = 5
	#TODO: Supports long,lat,alt
	pose.pose.position.x = path[0].pos.x
	pose.pose.position.y = path[0].pos.y
	pose.pose.position.z = path[0].pos.z + offset_height

	# Send a few setpoints before starting
	for i in range(100):
		if(rospy.is_shutdown()):
			break

		local_pos_pub.publish(pose)
		rate.sleep()

	offb_set_mode = SetModeRequest()
	offb_set_mode.custom_mode = 'OFFBOARD'

	followme_set_mode = SetModeRequest()
	followme_set_mode.custom_mode = 'AUTO.FOLLOW_TARGET'

	land_set_mode = SetModeRequest()
	land_set_mode.custom_mode = 'AUTO.LAND'

	arm_cmd = CommandBoolRequest()
	arm_cmd.value = True

	last_req = rospy.Time.now()

	def setState(custom_mode, cmd, last_req):
		if(current_state.mode != custom_mode and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
			if(set_mode_client.call(cmd).mode_sent == True):
				rospy.loginfo(f"{custom_mode} enabled")

			last_req = rospy.Time.now()
		else:
			if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
				if(arming_client.call(arm_cmd).success == True):
					rospy.loginfo("Vehicle armed")

				last_req = rospy.Time.now()
		return last_req

	rospy.loginfo("Start mission")
	while(not rospy.is_shutdown()):
		if (state == DState.Travel):
			last_req = setState(offb_set_mode.custom_mode, offb_set_mode, last_req)

			if (path[0].deadline + path[0].wait <= time.time()):
				path.pop(0)
				if (len(path) > 0):
					rospy.loginfo("Next mission")
					pose.pose.position.x = path[0].point.x
					pose.pose.position.y = path[0].point.y
					pose.pose.position.z = path[0].point.z + offset_height
				else:
					rospy.loginfo("End mission")
					state = DState.End
			elif (path[0].deadline <= time.time() - 15): #15 Seconds before it needs to be there
				pose.pose.position.x = path[0].point.x
				pose.pose.position.y = path[0].point.y
				pose.pose.position.z = path[0].point.z

			local_pos_pub.publish(pose)
		if (state == DState.End):
			last_req = setState(land_set_mode.custom_mode, land_set_mode, last_req)

		rate.sleep()