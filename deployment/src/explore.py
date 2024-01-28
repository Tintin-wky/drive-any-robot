# ROS
import rospy
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

import torch
from PIL import Image as PILImage
import numpy as np
import os
import argparse
import yaml
import random

# UTILS
from utils import to_numpy, transform_images, load_model,pil_to_msg
import pickle
import shutil
import io
from topomap import Topomap

import sys
sys.path.append('/home/classlab/drive-any-robot/train')

TOPOMAP_IMAGES_DIR = "../topomaps/images"
MODEL_WEIGHTS_PATH = "../model_weights"
ROBOT_CONFIG_PATH ="../config/robot.yaml"
MODEL_CONFIG_PATH = "../config/models.yaml"
with open(ROBOT_CONFIG_PATH, "r") as f:
    robot_config = yaml.safe_load(f)
MAX_V = robot_config["max_v"]
MAX_W = robot_config["max_w"]
RATE = robot_config["frame_rate"] 
IMAGE_TOPIC = "/camera/left/image_raw/compressed"
ODOM_TOPIC = "/odom_chassis"
DESTINATION_DIR = "../destination"
TOPOMAP_IMAGES_DIR = "../topomaps/images"

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
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)

# GLOBALS
context_queue = []
obs_img = PILImage.Image()
pose = Pose()

def callback_obs(msg):
    global last_message_time
    last_message_time = rospy.get_time()
    global obs_img
    obs_img = PILImage.open(io.BytesIO(msg.data))
    if len(context_queue) < model_params["context"] + 1:
        context_queue.append(obs_img)
    else:
        context_queue.pop(0)
        context_queue.append(obs_img)

def callback_odom(msg: Odometry):
    global pose
    pose=msg.pose.pose

def remove_files_in_dir(dir_path: str):
    for f in os.listdir(dir_path):
        file_path = os.path.join(dir_path, f)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print("Failed to delete %s. Reason: %s" % (file_path, e)) 

def explore(args: argparse.Namespace):
    # load destination
    goal_img=PILImage.open(os.path.join(DESTINATION_DIR, f"{args.destination}.png"))
    transf_goal_img = transform_images(goal_img, model_params["image_size"])

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
    if args.restart:
        topomap=Topomap()
    else:
        topomap_path = f"../topomaps/{args.name}.pkl"
        with open(topomap_path, 'rb') as file:
            topomap = pickle.load(file)
        topomap.reset()

    # ROS
    rospy.init_node("TOPOPLAN", anonymous=False)
    rate = rospy.Rate(RATE)
    image_curr_msg = rospy.Subscriber(IMAGE_TOPIC, CompressedImage, callback_obs, queue_size=1)
    odom_msg = rospy.Subscriber(ODOM_TOPIC, Odometry, callback_odom, queue_size=1)
    waypoint_pub = rospy.Publisher("/waypoint", Float32MultiArray, queue_size=1)
    goal_pub = rospy.Publisher("/topoplan/reached_goal", Bool, queue_size=1)
    closest_node_pub = rospy.Publisher("/topoplan/closest_node", Image, queue_size=1)
    last_node_pub = rospy.Publisher("/topoplan/last_node", Image, queue_size=1)
    rospy.loginfo("Registered with master node. Waiting for image observations...")
    
    reached_goal = False
    i = topomap.number_of_nodes()
    temporal_count = 0
    path = []
    last_node = None
    
    # exploration loop
    while not rospy.is_shutdown():
        if len(context_queue) > model_params["context"]:
            global obs_img
            transf_obs_img = transform_images(context_queue, model_params["image_size"])   

            dist, waypoints = model(transf_obs_img.to(device), transf_goal_img.to(device)) 
            distance=to_numpy(dist[0])
            waypoint=to_numpy(waypoints[0][args.waypoint])

            waypoint_msg = Float32MultiArray()
            if model_params["normalize"]:
                waypoint[:2] *= (MAX_V / RATE)
            waypoint_msg.data = waypoint
            waypoint_pub.publish(waypoint_msg)

            if i == 0:
                closest_node = i
                path.append(closest_node)
                topomap.add_node(closest_node, image=obs_img, pose=pose)
                rospy.loginfo("start create a new topomap")
                last_node = i
                i += 1
                temporal_count = 0
            else:
                if last_node is None:
                    check_distances = []
                    for node in range(topomap.number_of_nodes()):
                        check_img= topomap.nodes[node]['image']
                        transf_check_img = transform_images(check_img, model_params["image_size"])
                        dist, _ = model(transf_obs_img.to(device), transf_check_img.to(device)) 
                        check_distances.append(to_numpy(dist[0]))
                    closest_node = np.argmin(check_distances)
                    closest_distance = check_distances[closest_node]
                    rospy.loginfo(f"closest node: {closest_node} distance: {closest_distance.item():.2f}")
                    if closest_distance < args.close_threshold:
                        last_node = closest_node
                        topomap.nodes[closest_node]['count'] += 1
                        path.append(closest_node)
                        topomap.loopback(closest_node,pose)
                        rospy.loginfo("arrive at nodes on the former topomap")
                    else:
                        closest_node = i
                        path.append(closest_node)
                        topomap.add_node(closest_node, image=obs_img, pose=pose)
                        rospy.loginfo(f"start create a new topomap at node {i}")
                        last_node = closest_node
                        i += 1
                        temporal_count = 0

                if topomap.loop_back is False:
                    check_distances = []
                    check_nodes = [] 
                    for node in random.sample(range(topomap.last_node_ID),5):
                        check_img= topomap.nodes[node]['image']
                        transf_check_img = transform_images(check_img, model_params["image_size"])
                        dist, _ = model(transf_obs_img.to(device), transf_check_img.to(device)) 
                        check_distances.append(to_numpy(dist[0]))
                        check_nodes.append(node)
                    closest_node = check_nodes[np.argmin(check_distances)]
                    closest_distance = check_distances[np.argmin(check_distances)]
                    rospy.loginfo(f"closest node: {closest_node} distance: {closest_distance.item():.2f} sampled_nodes:{check_nodes}")
                    if closest_distance < args.close_threshold:
                        topomap.nodes[closest_node]['count'] += 1
                        path.append(closest_node)
                        topomap.loopback(closest_node,pose)
                        topomap.add_edge(last_node,closest_node,weight=temporal_count / RATE)
                        rospy.loginfo(f"from {last_node}[{topomap.nodes[last_node]['count']}] reach {closest_node}[{topomap.nodes[closest_node]['count']}]")
                        rospy.loginfo("arrive at nodes on the former topomap")
                        last_node = closest_node
                        temporal_count = 0
                        
                check_distances = []
                check_nodes = [] 
                for node in topomap.neighbors(last_node,args.area):
                    check_img= topomap.nodes[node]['image']
                    transf_check_img = transform_images(check_img, model_params["image_size"])
                    dist, _ = model(transf_obs_img.to(device), transf_check_img.to(device)) 
                    check_distances.append(to_numpy(dist[0]))
                    check_nodes.append(node)
                closest_node = check_nodes[np.argmin(check_distances)]
                closest_distance = check_distances[np.argmin(check_distances)]
                rospy.loginfo(f"closest node: {closest_node} distance: {closest_distance.item():.2f} nearby_nodes:{check_nodes}")
                    
                if  closest_distance < args.close_threshold and last_node != closest_node:
                    topomap.nodes[closest_node]['count'] += 1
                    path.append(closest_node)
                    topomap.update_node(closest_node,image=obs_img, pose=pose)
                    topomap.add_edge(last_node,closest_node,weight=temporal_count / RATE)
                    rospy.loginfo(f"from {last_node}[{topomap.nodes[last_node]['count']}] reach {closest_node}[{topomap.nodes[closest_node]['count']}]")
                    topomap.pruning(closest_node,check_nodes,check_distances,args.close_threshold)
                    last_node = closest_node
                    temporal_count = 0
                elif closest_distance > args.far_threshold and temporal_count != 0:
                    closest_node = i
                    path.append(closest_node)
                    topomap.add_node(closest_node, image=obs_img, pose=pose)
                    topomap.add_edge(last_node,closest_node,weight=temporal_count / RATE)
                    rospy.loginfo(f"from {last_node}[{topomap.nodes[last_node]['count']}] reach {closest_node}[{topomap.nodes[closest_node]['count']}]")
                    last_node = i
                    i += 1
                    temporal_count = 0
                elif temporal_count % (args.dt*RATE) == 0 and last_node != closest_node: 
                    closest_node = i
                    path.append(closest_node)
                    topomap.add_node(closest_node, image=obs_img, pose=pose)
                    topomap.add_edge(last_node,closest_node,weight=temporal_count / RATE)
                    rospy.loginfo(f"from {last_node}[{topomap.nodes[last_node]['count']}] reach {closest_node}[{topomap.nodes[closest_node]['count']}]")
                    last_node = i
                    i += 1
                    temporal_count = 0
            temporal_count += 1
        
            closet_node_image = topomap.nodes[closest_node]['image']
            last_node_image = topomap.nodes[last_node]['image']
            closest_node_pub.publish(pil_to_msg(closet_node_image))
            last_node_pub.publish(pil_to_msg(last_node_image))
            reached_goal = bool(distance < args.close_threshold)
            goal_pub.publish(reached_goal)

            if reached_goal:
                rospy.loginfo("Reached goal Stopping...")
                closest_node = i
                topomap.add_node(closest_node, image=obs_img, pose=pose)
                path.append(closest_node)
                topomap.add_edge(last_node,closest_node,weight=temporal_count / RATE)
                last_node = i
                i += 1
                temporal_count = 0

                print(f"path:{path}")
                topomap.path.append(path)
                if args.save:
                    topomap.save(args.name)
                topomap.visualize()
                return
            
            global last_message_time
            if rospy.get_time() - last_message_time > 1:
                rospy.loginfo("No message received for {} seconds. Exiting...".format(1))
                print(f"path:{path}")
                topomap.path.append(path)
                if args.save:
                    topomap.save(args.name)
                topomap.visualize()
                return
            
            rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Code to run GNMs on the locobot")
    parser.add_argument(
        "--destination",
        "-d",
        default="8",
        type=str,
        help="path to destination image",
    )
    parser.add_argument(
        "--name",
        "-n",
        default="topomap",
        type=str,
        help="name of your topomap (default: topomap)",
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
        default=3.,
        type=int,
        help="""temporal distance within the next node in the topomap before 
        localizing to it (default: 3)""",
    )
    parser.add_argument(
        "--dt",
        "-t",
        default=5,
        type=float,
        help=f"time between images sampled from the {IMAGE_TOPIC} topic (default: 3 seconds)",
    )
    parser.add_argument(
        "--area",
        "-a",
        default=10.,
        type=float,
        help="""area range of nearby nodes checked in odom(default: 10)""",
    )
    parser.add_argument(
        "--restart",
        "-r",
        default=False,
        type=bool,
        help="""whether remake the topomap or not (default: Flase)""",
    )
    parser.add_argument(
        "--save",
        "-s",
        default=False,
        type=bool,
        help="""whether save the topomap or not (default: True)""",
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
    print(args)
    print(f"Using {device}")
    explore(args)

