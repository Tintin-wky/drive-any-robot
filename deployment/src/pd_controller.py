import numpy as np
import yaml
from typing import Tuple
import math

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Bool

vel_msg = Twist()
path = Path()
path.header.frame_id = 'odom'
reached_goal = False
CONFIG_PATH = "../config/robot.yaml"
with open(CONFIG_PATH, "r") as f:
	robot_config = yaml.safe_load(f)
MAX_V = robot_config["max_v"]
MAX_W = robot_config["max_w"]
VEL_TOPIC = robot_config["vel_navi_topic"]
DT = 1/robot_config["frame_rate"]
RATE = 9
EPS = 1e-8


def clip_angle(theta) -> float:
	"""Clip angle to [-pi, pi]"""
	theta %= 2 * np.pi
	if -np.pi < theta < np.pi:
		return theta
	return theta - 2 * np.pi
      

def pd_controller(waypoint: np.ndarray) -> Tuple[float]:
	"""PD controller for the robot"""
	assert len(waypoint) == 2 or len(waypoint) == 4, "waypoint must be a 2D or 4D vector"
	if len(waypoint) == 2:
		dx, dy = waypoint
	else:
		dx, dy, hx, hy = waypoint
	# this controller only uses the predicted heading if dx and dy near zero
	if len(waypoint) == 4 and np.abs(dx) < EPS and np.abs(dy) < EPS:
		v = 0
		w = clip_angle(np.arctan2(hy, hx))/DT		
	elif np.abs(dx) < EPS:
		v =  0
		w = np.sign(dy) * np.pi/(2*DT)
	else:
		v = dx / DT
		w = np.arctan(dy/dx) / DT
	v = np.clip(v, 0, MAX_V)
	w = np.clip(w, -MAX_W, MAX_W)
	return v, w

def callback_drive(waypoint_msg: Float32MultiArray):
	"""Callback function for the waypoint subscriber"""
	global vel_msg,path
	waypoint = waypoint_msg.data
	v, w = pd_controller(waypoint)
	vel_msg = Twist()
	vel_msg.linear.x = v
	vel_msg.angular.z = w


def callback_reached_goal(reached_goal_msg: Bool):
	"""Callback function for the reached goal subscriber"""
	global reached_goal
	reached_goal = reached_goal_msg.data


def main():
	rospy.init_node("PD_CONTROLLER", anonymous=False)
	waypoint_sub = rospy.Subscriber("/waypoint", Float32MultiArray, callback_drive, queue_size=1)
	reached_goal_sub = rospy.Subscriber("/topoplan/reached_goal", Bool, callback_reached_goal, queue_size=1)
	path_pub = rospy.Publisher('/path', Path, queue_size=10)
	vel_out = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
	rate = rospy.Rate(RATE)
	rospy.loginfo("Registered with master node. Waiting for waypoints...")
	while not rospy.is_shutdown():
		global vel_msg,path
		path_pub.publish(path)
		if reached_goal:
			vel_out.publish(Twist())
			rospy.loginfo("Reached goal! Stopping...")
			# return
		else:
			vel_out.publish(vel_msg)
			rospy.loginfo(f"publishing new vel: v:{vel_msg.linear.x} w:{vel_msg.angular.z}")
		rate.sleep()
	

if __name__ == '__main__':
	main()
