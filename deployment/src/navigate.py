# ROS
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from gps_common.msg import GPSFix
from geometry_msgs.msg import Pose

import torch
from PIL import Image
import numpy as np
import os
import pickle
import argparse
import yaml
import io

# UTILS
from utils import to_numpy, transform_images, load_model
from gps import get_gps

import sys
sys.path.append('/home/classlab/drive-any-robot/train')

MODEL_WEIGHTS_PATH = "../model_weights"
ROBOT_CONFIG_PATH ="../config/robot.yaml"
MODEL_CONFIG_PATH = "../config/models.yaml"
with open(ROBOT_CONFIG_PATH, "r") as f:
    robot_config = yaml.safe_load(f)
MAX_V = robot_config["max_v"]
MAX_W = robot_config["max_w"]
RATE = robot_config["frame_rate"] 
IMAGE_TOPIC = "/camera/left/image_raw/compressed"
GPS_TOPIC = "/gps/gps"
DESTINATION_DIR="../destination"

# DEFAULT MODEL PARAMETERS (can be overwritten by model.yaml)
model_params = {
    "path": "large_gnm.pth", # path of the model in ../model_weights
    "model_type": "gnm", # gnm (conditioned), stacked, or siamese
    "context": 5, # number of images to use as context
    "len_traj_pred": 5, # number of waypoints to predict
    "normalize": True, # bool to determine whether or not normalize images
    "image_size": [85, 64], # (width, height)
    "normalize": True, # bool to determine whether or not normalize the waypoints
    "learn_angle": True, # bool to determine whether or not to learn/predict heading of the robot
    "obs_encoding_size": 1024, # size of the encoding of the observation [only used by gnm and siamese]
    "goal_encoding_size": 1024, # size of the encoding of the goal [only used by gnm and siamese]
    "obsgoal_encoding_size": 2048, # size of the encoding of the observation and goal [only used by stacked model]
}

# GLOBALS
context_queue = []
odom = Pose()
latlon = None

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)

def callback_obs(msg):
    obs_img = Image.open(io.BytesIO(msg.data))
    global context_queue
    if len(context_queue) < model_params["context"] + 1:
        context_queue.append(obs_img)
    else:
        context_queue.pop(0)
        context_queue.append(obs_img)

def callback_gps(msg: GPSFix):
    global latlon
    latlon={'latitude':msg.latitude,'longitude':msg.longitude}

def get_closest_node(model,topomap,image_queue,latlon=None):
    transf_image = transform_images(image_queue, model_params["image_size"])
    check_distances = []
    check_nodes = [] 
    for node in topomap.nodes():
        if latlon is not None:
            if topomap.nodes[node].get('gps') is not None:
                if abs(latlon['latitude']-topomap.nodes()[node]['gps'].latitude) > 0.0001 or abs(latlon['longitude']-topomap.nodes()[node]['gps'].longitude) > 0.0001:
                    continue
        check_img= topomap.nodes[node]['image']
        transf_check_img = transform_images(check_img, model_params["image_size"])
        dist, _ = model(transf_image.to(device), transf_check_img.to(device)) 
        check_distances.append(to_numpy(dist[0]))
        check_nodes.append(node)
    if latlon is not None:
        rospy.loginfo(f"latitude:{latlon['latitude']} longitude:{latlon['longitude']}")
    rospy.loginfo(f"check nodes:{check_nodes}")
    closest_node = check_nodes[np.argmin(check_distances)]
    closest_distance = check_distances[np.argmin(check_distances)]
    return closest_node, closest_distance

def main(args: argparse.Namespace):
    # load model parameters
    with open(MODEL_CONFIG_PATH, "r") as f:
        model_config = yaml.safe_load(f)
    for param in model_config:
        model_params[param] = model_config[param]

    # load model weights
    model_filename = model_config[args.model]["path"]
    model_path = os.path.join(MODEL_WEIGHTS_PATH, model_filename)
    if os.path.exists(model_path):
        print(f"Loading model from {model_path}")
    else:
        raise FileNotFoundError(f"Model weights not found at {model_path}")
    model = load_model(
        model_path,
        model_params["model_type"],
        model_params["context"],
        model_params["len_traj_pred"],
        model_params["learn_angle"], 
        model_params["obs_encoding_size"], 
        model_params["goal_encoding_size"],
        model_params["obsgoal_encoding_size"],
        device,
    )
    model.eval()

    # load topomaps
    topomap_path = f"../topomaps/{args.name}.pkl"
    with open(topomap_path, 'rb') as file:
        topomap = pickle.load(file)
        topomap.reset()

    # load destination
    goal_img=Image.open(os.path.join(DESTINATION_DIR, f"{args.destination}.jpg"))
    goal_latlon = get_gps(goal_img)
    # goal_latlon={'latitude':22.5883,'longitude':113.9670}

    # ROS
    rospy.init_node("TOPOPLAN", anonymous=False)
    rate = rospy.Rate(RATE)
    image_curr_msg = rospy.Subscriber(IMAGE_TOPIC, CompressedImage, callback_obs, queue_size=1)
    gps_curr_msg = rospy.Subscriber(GPS_TOPIC, GPSFix, callback_gps, queue_size=1)
    waypoint_pub = rospy.Publisher("/waypoint", Float32MultiArray, queue_size=1)
    goal_pub = rospy.Publisher("/topoplan/reached_goal", Bool, queue_size=1)
    rospy.loginfo("Registered with master node. Waiting for image observations...")

    i = 0
    path = []
    complete_path = False
    reached_goal = False
    forward_node = None
    forward_point_count = 0
    # navigation loop
    while not rospy.is_shutdown():
        if len(context_queue) > model_params["context"]:
            if not path:
                global latlon
                start_node, distance_start = get_closest_node(model, topomap, context_queue, latlon)
                goal_queue = []
                while len(goal_queue) < model_params["context"] + 1:
                    goal_queue.append(goal_img)
                goal_node, distance_end = get_closest_node(model, topomap, goal_queue, goal_latlon)
                path = topomap.shortest_path(start_node, goal_node)
                rospy.loginfo(f"path: {path}")
                rospy.loginfo(f"start at node {path[0]} distance:{distance_start}")
                rospy.loginfo(f"end at node {path[-1]} distance:{distance_end}")

            transf_obs_img = transform_images(context_queue, model_params["image_size"])
            if complete_path:
                transf_sg_img = transform_images(goal_img, model_params["image_size"])
                dist, waypoint = model(transf_obs_img.to(device), transf_sg_img.to(device)) 
                distance=to_numpy(dist[0])
                waypoint=to_numpy(waypoint[0][args.waypoint])
                rospy.loginfo(f"Target: goal Estimate distance:{distance.item():.2f}")
                closest_distance = distance
            else:
                check_distances = []
                check_nodes = [] 
                waypoints = []
                start = i
                end = min(i + args.radius, len(path)-1)
                for node in path[start:end+1]:
                    check_img= topomap.nodes[node]['image']
                    transf_check_img = transform_images(check_img, model_params["image_size"])
                    dist, waypoint = model(transf_obs_img.to(device), transf_check_img.to(device)) 
                    check_distances.append(to_numpy(dist[0]))
                    waypoints.append(to_numpy(waypoint[0]))
                    check_nodes.append(node)
                rospy.loginfo(f"Target node: {path[i]} nearby_nodes:{check_nodes}")
                closest_index = np.argmin(check_distances)
                closest_node = check_nodes[closest_index]
                closest_distance = check_distances[closest_index]
                rospy.loginfo(f"closest node: {closest_node} distance: {closest_distance.item():.2f}")
                if closest_distance < args.far_threshold:
                    if forward_node is None:
                        forward_node = closest_node
                    elif forward_node == closest_node and forward_node != path[i]:
                        forward_point_count += 1
                        rospy.loginfo(f"forward node: {forward_node} count:{forward_point_count}")
                    else:
                        forward_node = closest_node
                        forward_point_count = 0
                if forward_point_count >= 3:
                    forward_point_count = 0
                    forward_node = None
                    chosen_waypoint = waypoints[closest_index][args.waypoint]
                    i += closest_index
                else:
                    chosen_waypoint = waypoints[0][args.waypoint]

            if closest_distance < args.close_threshold:
                if complete_path:
                    reached_goal = True
                    rospy.loginfo(f"reach goal!")
                elif path[i] == path[-1]: # goal_node
                    complete_path = True
                    rospy.loginfo(f"end at node {path[-1]}")
                    rospy.loginfo(f"complete the path")
                    if distance_end < args.close_threshold:
                        reached_goal = True
                        rospy.loginfo(f"reach goal!")
                else:
                    rospy.loginfo(f"arrive at node {path[i]} ({i}/{len(path)-1})")
                    i = i + 1
            goal_pub.publish(reached_goal)
            if reached_goal:
                return

            waypoint_msg = Float32MultiArray()
            if model_params["normalize"]:
                chosen_waypoint[:2] *= (MAX_V / RATE)
            waypoint_msg.data = chosen_waypoint
            waypoint_pub.publish(waypoint_msg)

            rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Code to run GNMs on the classbot")
    parser.add_argument(
        "--name",
        "-n",
        default="topomap",
        type=str,
        help="name of your topomap (default: topomap)",
    )
    parser.add_argument(
        "--destination",
        "-d",
        default="8",
        type=str,
        help="name of destination",
    )
    parser.add_argument(
        "--model",
        "-m",
        default="gnm_large",
        type=str,
        help="model name (hint: check ../config/models.yaml) (default: large_gnm)",
    )
    parser.add_argument(
        "--close_threshold",
        "-t",
        default=3.,
        type=float,
        help="""temporal distance within the next node in the topomap before 
        localizing to it (default: 3)""",
    )
    parser.add_argument(
        "--far_threshold",
        default=10.,
        type=float,
        help="""temporal distance far away from the nodes in the topomap before 
        localizing to it (default: 18)""",
    )
    parser.add_argument(
        "--radius",
        "-r",
        default=2,
        type=int,
        help="""temporal number of locobal nodes to look at in the topopmap for
        localization (default: 2)""",
    )
    parser.add_argument(
        "--waypoint",
        "-w",
        default=2, # close waypoints exihibit straight line motion (the middle waypoint is a good default)
        type=int,
        help=f"""index of the waypoint used for navigation (between 0 and 4 or 
        how many waypoints your model predicts) (default: 2)""",
    )
    args = parser.parse_args()
    print(f"Using {device}")
    main(args)

