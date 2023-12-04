import argparse
import os
from utils import msg_to_pil 
import time
import shutil

# ROS
import rospy
from sensor_msgs.msg import Image

from topomap import Topomap
import pickle

IMAGE_TOPIC = "/camera/left/image_raw"
TOPOMAP_IMAGES_DIR = "../topomaps/images"
TOPOMAP_MATRIX = "../topomaps/matrix.pkl"
obs_img = None


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

def not_green_image(image):
    green_pixels = 0
    total_pixels = image.width * image.height

    for pixel in image.getdata():
        if pixel[0] == 0 and pixel[1] != 0 and pixel[2] == 0:  # (R, G, B)，检查是否为绿色
            green_pixels += 1

    # 如果所有像素都是绿色，则图像是完全绿色的
    return green_pixels != total_pixels

def callback_obs(msg: Image):
    global obs_img
    temp_img = msg_to_pil(msg)
    if not_green_image(temp_img):
        obs_img = temp_img

def main(args: argparse.Namespace):
    global obs_img
    rospy.init_node("CREATE_TOPOMAP", anonymous=False)
    image_curr_msg = rospy.Subscriber(
        IMAGE_TOPIC, Image, callback_obs, queue_size=1)

    topomap_name_dir = os.path.join(TOPOMAP_IMAGES_DIR, args.name)
    if not os.path.isdir(topomap_name_dir):
        os.makedirs(topomap_name_dir)
    else:
        print(f"{topomap_name_dir} already exists. Removing previous images...")
        remove_files_in_dir(topomap_name_dir)
    

    assert args.dt > 0, "dt must be positive"
    rate = rospy.Rate(1/args.dt)
    print("Registered with master node. Waiting for images...")
    i = 0
    start_time = float("inf")
    topomap=Topomap()
    while not rospy.is_shutdown():
        if obs_img is not None:
            obs_img.save(os.path.join(topomap_name_dir, f"{i}.png"))
            topomap.update(i,obs_img,args.dt)
            print("published image", i)
            i += 1
            rate.sleep()
            start_time = time.time()
            obs_img = None
        if time.time() - start_time > 2 * args.dt:
            print(f"Topic {IMAGE_TOPIC} not publishing anymore. Shutting down...")
            rospy.signal_shutdown("shutdown")
            with open(TOPOMAP_MATRIX, 'wb') as file:
                pickle.dump(dict([(args.name,topomap.get_adjacency_matrix())]), file)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=f"Code to generate topomaps from the {IMAGE_TOPIC} topic"
    )
    parser.add_argument(
        "--name",
        "-n",
        default="topomap",
        type=str,
        help="name of your topomap (default: topomap)",
    )
    parser.add_argument(
        "--dt",
        "-t",
        default=3.,
        type=float,
        help=f"time between images sampled from the {IMAGE_TOPIC} topic (default: 3.0)",
    )
    args = parser.parse_args()

    main(args)
