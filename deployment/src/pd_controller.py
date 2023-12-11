import numpy as np
import yaml
from typing import Tuple

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseStamped
import tf.transformations as tf_trans
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

	global x,y,yaw,t
	if len(path.poses) == 0:
		yaw = 0
		x = 0
		y = 0
		t = rospy.Time.now()
	else:
		# 根据收到的twist消息更新姿势
		dt = (rospy.Time.now() - t).to_sec()
		yaw += 0.5 * vel_msg.angular.z * dt
		x += vel_msg.linear.x * np.cos(yaw) * dt
		y += vel_msg.linear.x * np.sin(yaw) * dt
		yaw += 0.5 * vel_msg.angular.z * dt
		t = rospy.Time.now()

	pose = PoseStamped()
	pose.header.stamp =rospy.Time.now()
	pose.header.frame_id = path.header.frame_id
	pose.pose.position.x = x
	pose.pose.position.y = y
	orientation = tf_trans.quaternion_from_euler(0, 0, yaw)
	pose.pose.orientation.x = orientation[0]
	pose.pose.orientation.y = orientation[1]
	pose.pose.orientation.z = orientation[2]
	pose.pose.orientation.w = orientation[3]

	# 将新的PoseStamped添加到Path消息中
	path.poses.append(pose)
	
	waypoint = waypoint_msg.data
	v, w = pd_controller(waypoint)
	vel_msg = Twist()
	vel_msg.linear.x = v
	vel_msg.angular.z = w
	rospy.loginfo(f"publishing new vel: v:{v} w:{w}")


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
		vel_out.publish(vel_msg)
		path_pub.publish(path)
		if reached_goal:
			vel_msg = Twist()
			vel_out.publish(vel_msg)
			rospy.loginfo("Reached goal! Stopping...")
			return
		rate.sleep()
	

if __name__ == '__main__':
	main()
