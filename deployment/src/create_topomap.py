import rospy
import rosbag
import argparse
import shutil
import os
import pickle
from topomap import Topomap
from PIL import Image
import io

IMAGE_TOPIC = "/camera/left/image_raw/compressed"
TOPOMAP_IMAGES_DIR = "../topomaps/images"
TOPOMAP_MATRIX = "../topomaps/matrix.pkl"

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
    rospy.init_node("CREATE_TOPOMAP", anonymous=False)

    topomap_name_dir = os.path.join(TOPOMAP_IMAGES_DIR, args.name)
    if not os.path.isdir(topomap_name_dir):
        os.makedirs(topomap_name_dir)
    else:
        print(f"{topomap_name_dir} already exists. Removing previous images...")
        remove_files_in_dir(topomap_name_dir)

    bag = rosbag.Bag(args.rosbag)
    i = 0
    start_time = float("inf")
    topomap=Topomap()
    for topic, msg, t in bag.read_messages(topics=[IMAGE_TOPIC]):
        if t.to_sec() < start_time:
            start_time = t.to_sec()
        if t.to_sec() - start_time >= args.dt:
            start_time = t.to_sec()
            obs_img = Image.open(io.BytesIO(msg.data))
            obs_img.save(os.path.join(topomap_name_dir, f"{i}.png"))
            topomap.update(i,obs_img,args.dt)
            i+=1
            print("saved image", i)

    print(f"Topic {IMAGE_TOPIC} not publishing anymore. Shutting down...")
    rospy.signal_shutdown("shutdown")
    with open(TOPOMAP_MATRIX, 'wb') as file:
        pickle.dump(dict([(args.name,topomap.get_adjacency_matrix())]), file)
    bag.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=f"Code to generate topomaps from the {IMAGE_TOPIC} topic"
    )
    parser.add_argument(
        "--rosbag",
        "-r",
        default="../topomaps/bags/raw/test.bag",
        type=str,
        help="path to your rosbag (default: topomap)",
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
        default=5.,
        type=float,
        help=f"time between images sampled from the {IMAGE_TOPIC} topic (default: 3.0)",
    )
    args = parser.parse_args()

    main(args)
