# ROS
import rospy
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import Bool, Float32MultiArray

import torch
from PIL import Image as PILImage
import numpy as np
import os
import time
import argparse
import yaml

# UTILS
from utils import to_numpy, transform_images, load_model
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
DESTINATION_DOMAIN = 3
DESTINATION_PATH = "../topomaps/destination"
TOPOMAP_IMAGES_DIR = "../topomaps/images"
TOPOMAP_MATRIX = "../topomaps/matrix.pkl"

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

def main(args: argparse.Namespace):
    # load destination
    destination = args.destination
    goal_img=PILImage.open(destination)
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

    topomap_name_dir = os.path.join(TOPOMAP_IMAGES_DIR, args.name)
    if not os.path.isdir(topomap_name_dir):
        os.makedirs(topomap_name_dir)
    else:
        print(f"{topomap_name_dir} already exists. Removing previous images...")
        remove_files_in_dir(topomap_name_dir)

    # ROS
    rospy.init_node("TOPOPLAN", anonymous=False)
    rate = rospy.Rate(RATE)
    image_curr_msg = rospy.Subscriber(
        IMAGE_TOPIC, CompressedImage, callback_obs, queue_size=1)
    waypoint_pub = rospy.Publisher(
        "/waypoint", Float32MultiArray, queue_size=1)
    goal_pub = rospy.Publisher("/topoplan/reached_goal", Bool, queue_size=1)
    rospy.loginfo("Registered with master node. Waiting for image observations...")
    
    reached_goal = False
    i = 0
    temporal_count = 0
    topomap=Topomap()
    
    # exploration loop
    while not rospy.is_shutdown():
        if len(context_queue) > model_params["context"]:
            global obs_img
            transf_obs_img = transform_images(context_queue, model_params["image_size"])
            dist, waypoints = model(transf_obs_img, transf_goal_img) 
            distance=to_numpy(dist[0])
            waypoint=to_numpy(waypoints[0][args.waypoint])
 
            if temporal_count % (args.dt*RATE) == 0:
                last_time = time.time()    
                obs_img.save(os.path.join(topomap_name_dir, f"{i}.png"))
                rospy.loginfo(f"saved image {i}")
                topomap.update(i,obs_img,args.dt)
                i += 1
                temporal_count = 0
            temporal_count += 1
            waypoint_msg = Float32MultiArray()
            if model_params["normalize"]:
                waypoint[:2] *= (MAX_V / RATE)
            waypoint_msg.data = waypoint
            waypoint_pub.publish(waypoint_msg)
            rospy.loginfo(f"Estimate distance:{distance} Next waypoint: dx:{waypoint[0]} dy:{waypoint[1]} ")

            reached_goal = bool(distance < args.close_threshold)
            goal_pub.publish(reached_goal)
            if reached_goal:
                rospy.loginfo("Reached goal Stopping...")
                obs_img.save(os.path.join(topomap_name_dir, f"{i}.png"))
                rospy.loginfo(f"saved image {i}")
                topomap.update(i,obs_img,temporal_count / RATE)
                i += 1
                with open(TOPOMAP_MATRIX, 'wb') as file:
                    pickle.dump(dict([(args.name,topomap.get_adjacency_matrix())]), file)
                return
            global last_message_time
            if rospy.get_time() - last_message_time > 1:
                rospy.loginfo("No message received for {} seconds. Exiting...".format(1))
                rospy.loginfo("Reached goal Stopping...")
                obs_img.save(os.path.join(topomap_name_dir, f"{i}.png"))
                rospy.loginfo(f"saved image {i}")
                topomap.update(i,obs_img,temporal_count / RATE)
                i += 1
                with open(TOPOMAP_MATRIX, 'wb') as file:
                    pickle.dump(dict([(args.name,topomap.get_adjacency_matrix())]), file)
                rospy.signal_shutdown("Timeout reached")
            
            rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Code to run GNMs on the locobot")
    parser.add_argument(
        "--destination",
        "-d",
        default="../topomaps/destination/8.png",
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
        default=2.,
        type=int,
        help="""temporal distance within the next node in the topomap before 
        localizing to it (default: 3)""",
    )
    parser.add_argument(
        "--dt",
        "-t",
        default=5,
        type=float,
        help=f"time between images sampled from the {IMAGE_TOPIC} topic (default: 5 seconds)",
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

