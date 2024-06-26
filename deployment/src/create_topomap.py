# ROS
import rospy
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from gps_common.msg import GPSFix
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
from latlon import LatLon

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
ODOM_TOPIC = "/robot_pose_ekf/odom_combined"
# ODOM_TOPIC = "/odom_chassis"
GPS_TOPIC = "/gps/gps"

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
latlon_queue = []
obs_img = None
pose = None
gps = None

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

def callback_gps(msg: GPSFix):
    latlon=None
    if(msg.latitude!=0 and msg.longitude!=0):
        latlon=LatLon(latitude=msg.latitude,longitude=msg.longitude)
    global latlon_queue
    if len(latlon_queue) < 100:
        latlon_queue.append(latlon)
    else:
        latlon_queue.pop(0)
        latlon_queue.append(latlon)
    global gps
    gps=None
    for i in range(-1,-len(latlon_queue)-1,-1):
        if latlon_queue[i] is not None:
            gps=latlon_queue[i]
            break

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
    gps_msg = rospy.Subscriber(GPS_TOPIC, GPSFix, callback_gps)
    closest_node_pub = rospy.Publisher("/topoplan/closest_node", Image, queue_size=1)
    last_node_pub = rospy.Publisher("/topoplan/last_node", Image, queue_size=1)
    rospy.loginfo("Registered with master node. Waiting for image observations...")
    
    ID = topomap.last_node_ID + 1
    temporal_count = 0
    path = []
    last_node = None
    
    while not rospy.is_shutdown():
        if len(context_queue) > model_params["context"]:
            transf_obs_img = transform_images(context_queue, model_params["image_size"])   
            if ID == 0:
                closest_node = ID
                path.append(closest_node)
                topomap.add_node(closest_node, image=obs_img, pose=pose, gps=gps)
                rospy.loginfo("start create a new topomap")
                last_node = ID
                ID += 1
                temporal_count = 0
            else:
                if last_node is None:
                    check_distances = []
                    check_nodes = [] 
                    for node in topomap.nodes():
                        check_img= topomap.nodes[node]['image']
                        transf_check_img = transform_images(check_img, model_params["image_size"])
                        dist, _ = model(transf_obs_img.to(device), transf_check_img.to(device)) 
                        check_distances.append(to_numpy(dist[0]))
                        check_nodes.append(node)
                    closest_node = check_nodes[np.argmin(check_distances)]
                    closest_distance = check_distances[np.argmin(check_distances)]
                    rospy.loginfo(f"closest node: {closest_node} distance: {closest_distance.item():.2f}")
                    if closest_distance < args.close_threshold:
                        last_node = closest_node
                        topomap.nodes[closest_node]['count'] += 1
                        path.append(closest_node)
                        topomap.loopback(closest_node,pose)
                        rospy.loginfo("arrive at nodes on the former topomap")
                    else:
                        closest_node = ID
                        path.append(closest_node)
                        topomap.add_node(closest_node, image=obs_img, pose=pose, gps=gps)
                        rospy.loginfo(f"start create a new topomap at node {ID}")
                        last_node = closest_node
                        ID += 1
                        temporal_count = 0

                if topomap.loop_back is False:
                    check_distances = []
                    check_nodes = [] 
                    for node in random.sample([n for n in topomap.nodes() if  n <= topomap.last_node_ID],8):
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
                    
                if  closest_distance < args.close_threshold:
                    if last_node != closest_node:
                        topomap.nodes[closest_node]['count'] += 1
                        path.append(closest_node)
                        topomap.update_node(closest_node,image=obs_img, pose=pose)
                        topomap.add_edge(last_node,closest_node,weight=temporal_count / RATE)
                        rospy.loginfo(f"from {last_node}[{topomap.nodes[last_node]['count']}] reach {closest_node}[{topomap.nodes[closest_node]['count']}]")
                        # pruning
                        for i in range(len(check_nodes)):
                            if check_nodes[i] != closest_node and check_distances[i] < args.close_threshold:
                                topomap.merge(closest_node,check_nodes[i])
                                rospy.loginfo(f"replace node {check_nodes[i]} with node {closest_node}")
                        last_node = closest_node
                        temporal_count = 0
                elif closest_distance > args.far_threshold:
                    closest_node = ID
                    path.append(closest_node)
                    topomap.add_node(closest_node, image=obs_img, pose=pose, gps=gps)
                    topomap.add_edge(last_node,closest_node,weight=temporal_count / RATE)
                    rospy.loginfo(f"from {last_node}[{topomap.nodes[last_node]['count']}] reach {closest_node}[{topomap.nodes[closest_node]['count']}]")
                    last_node = ID
                    ID += 1
                    temporal_count = 0
                elif temporal_count >= (args.dt*RATE): 
                    closest_node = ID
                    path.append(closest_node)
                    topomap.add_node(closest_node, image=obs_img, pose=pose, gps=gps)
                    topomap.add_edge(last_node,closest_node,weight=temporal_count / RATE)
                    rospy.loginfo(f"from {last_node}[{topomap.nodes[last_node]['count']}] reach {closest_node}[{topomap.nodes[closest_node]['count']}]")
                    last_node = ID
                    ID += 1
                    temporal_count = 0
            temporal_count += 1
        
            closet_node_image = topomap.nodes[closest_node]['image']
            last_node_image = topomap.nodes[last_node]['image']
            closest_node_pub.publish(pil_to_msg(closet_node_image))
            last_node_pub.publish(pil_to_msg(last_node_image))
            
            global last_message_time
            if rospy.get_time() - last_message_time > args.dt:
                rospy.loginfo("No message received for {} seconds. Exiting...".format(args.dt))
                print(f"path:{path}")
                topomap.path.append(path)
                if args.save:
                    if topomap.loop_back:
                        topomap.save(args.name)
                    else:
                        rospy.loginfo("Not loopback!")
                        old_nodes = [n for n in topomap.nodes() if n <= topomap.last_node_ID]
                        for n in old_nodes:
                            topomap.remove_node(n)
                        topomap.save(f"{args.name}_part")
                else:
                    topomap.visualize()
                return
            
            if temporal_count > RATE * args.dt * 2:
                rospy.loginfo("Having stopped for {} seconds. Exiting...".format(args.dt*2))
                print(f"path:{path}")
                topomap.path.append(path)
                if args.save:
                    if topomap.loop_back:
                        topomap.save(args.name)
                    else:
                        rospy.loginfo("Not loopback!")
                        old_nodes = [n for n in topomap.nodes() if n <= topomap.last_node_ID]
                        for n in old_nodes:
                            topomap.remove_node(n)
                        topomap.save(f"{args.name}_part")
                else:
                    topomap.visualize()
                return
            
            rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Code to run GNMs on the locobot")
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
        "--far_threshold",
        default=15.,
        type=int,
        help="""temporal distance far away from the nodes in the topomap before 
        localizing to it (default: 18)""",
    )
    parser.add_argument(
        "--dt",
        "-t",
        default=5,
        type=float,
        help=f"time between images sampled from the {IMAGE_TOPIC} topic (default: 5 seconds)",
    )
    parser.add_argument(
        "--area",
        "-a",
        default=5.,
        type=float,
        help="""area range of nearby nodes checked in odom(default: 5)""",
    )
    parser.add_argument(
        "--restart",
        "-r",
        action='store_true',
        help="""whether remake the topomap or not (default: Flase)""",
    )
    parser.add_argument(
        "--save",
        "-s",
        action='store_true',
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
    main(args)

