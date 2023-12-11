# ROS
import rospy
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import Bool, Float32MultiArray

import torch
from PIL import Image as PILImage
import numpy as np
import os
import argparse
import yaml
import io

# UTILS
from utils import msg_to_pil, to_numpy, transform_images, load_model

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

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)

def callback_obs(msg):
    obs_img = PILImage.open(io.BytesIO(msg.data))
    if len(context_queue) < model_params["context"] + 1:
        context_queue.append(obs_img)
    else:
        context_queue.pop(0)
        context_queue.append(obs_img)
    

def main(args: argparse.Namespace):
    # load topomap
    topomap_filenames = sorted(os.listdir(os.path.join(
        TOPOMAP_IMAGES_DIR, args.name)), key=lambda x: int(x.split(".")[0]))
    topomap_dir = f"{TOPOMAP_IMAGES_DIR}/{args.name}"
    num_nodes = len(os.listdir(topomap_dir))
    topomap = []
    for i in range(num_nodes):
        image_path = os.path.join(topomap_dir, topomap_filenames[i])
        topomap.append(PILImage.open(image_path))

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

    # ROS
    rospy.init_node("TOPOPLAN", anonymous=False)
    rate = rospy.Rate(RATE)
    image_curr_msg = rospy.Subscriber(
        IMAGE_TOPIC, CompressedImage, callback_obs, queue_size=1)
    waypoint_pub = rospy.Publisher(
        "/waypoint", Float32MultiArray, queue_size=1)
    goal_pub = rospy.Publisher("/topoplan/reached_goal", Bool, queue_size=1)
    rospy.loginfo("Registered with master node. Waiting for image observations...")

    closest_node = 0
    assert -1 <= args.goal_node < len(topomap), "Invalid goal index"
    if args.goal_node == -1:
        goal_node = len(topomap) - 1
    else:
        goal_node = args.goal_node
    reached_goal = False

    # navigation loop
    while not rospy.is_shutdown():
        if len(context_queue) > model_params["context"]:
            start = max(closest_node - args.radius, 0)
            end = min(closest_node + args.radius, goal_node)
            distances = []
            waypoints = []
            for sg_img in topomap[start: end + 1]:
                transf_obs_img = transform_images(context_queue, model_params["image_size"])
                transf_sg_img = transform_images(sg_img, model_params["image_size"])
                dist, waypoint = model(transf_obs_img.to(device), transf_sg_img.to(device)) 
                # transf_goal_img = transform_images(topomap[-1], model_params["image_size"])
                # dist, waypoint = model(transf_obs_img, transf_goal_img) 
                distances.append(to_numpy(dist[0]))
                waypoints.append(to_numpy(waypoint[0]))
            # look for closest node
            closest_node = np.argmin(distances)
            # chose subgoal and output waypoints
            if distances[closest_node] > args.close_threshold:
                chosen_waypoint = waypoints[closest_node][args.waypoint]
            else:
                chosen_waypoint = waypoints[min(
                    closest_node + 1, len(waypoints) - 1)][args.waypoint]
            waypoint_msg = Float32MultiArray()
            if model_params["normalize"]:
                chosen_waypoint[:2] *= (MAX_V / RATE)
            waypoint_msg.data = chosen_waypoint
            waypoint_pub.publish(waypoint_msg)
            rospy.loginfo(f"Closest node: {closest_node + start} Estimate distance:{distances[closest_node]}")
            # rospy.loginfo(f"Next waypoint: dx:{chosen_waypoint[0]} dy:{chosen_waypoint[1]}")
            closest_node += start
            reached_goal = closest_node == goal_node
            goal_pub.publish(reached_goal)
            if reached_goal:
                rospy.loginfo("Reached goal Stopping...")
                return
            rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Code to run GNMs on the locobot")
    parser.add_argument(
        "--name",
        "-n",
        default="test",
        type=str,
        help="topomap name",
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
        default=3,
        type=int,
        help="""temporal distance within the next node in the topomap before 
        localizing to it (default: 3)""",
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
    parser.add_argument(
        "--goal-node",
        "-g",
        default=-1,
        type=int,
        help="""goal node index in the topomap (if -1, then the goal node is 
        the last node in the topomap) (default: -1)""",
    )
    args = parser.parse_args()
    print(f"Using {device}")
    main(args)

