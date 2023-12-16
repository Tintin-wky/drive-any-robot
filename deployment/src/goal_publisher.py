import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from PIL import Image as PILImage
from utils import pil_to_msg
import os
import argparse

TOPOMAP_IMAGES_DIR="../topomaps/images"

reached_goal = False
def callback_reached_goal(reached_goal_msg: Bool):
	"""Callback function for the reached goal subscriber"""
	global reached_goal
	reached_goal = reached_goal_msg.data
     
def main(args: argparse.Namespace):
    rospy.init_node('goal_publisher', anonymous=True)
    pub = rospy.Publisher('/goal/image', Image, queue_size=1)
    reached_goal_sub = rospy.Subscriber("/topoplan/reached_goal", Bool, callback_reached_goal, queue_size=1)
    rate = rospy.Rate(1/args.duration) if args.ID == -1 else rospy.Rate(1)

    # load topomap
    topomap_filenames = sorted(os.listdir(os.path.join(
        TOPOMAP_IMAGES_DIR, args.name)), key=lambda x: int(x.split(".")[0]))
    topomap_dir = f"{TOPOMAP_IMAGES_DIR}/{args.name}"
    num_nodes = len(os.listdir(topomap_dir))
    topomap = []
    for i in range(num_nodes):
        image_path = os.path.join(topomap_dir, topomap_filenames[i])
        topomap.append(PILImage.open(image_path))

    i = 0 if args.ID == -1 else args.ID
    while not rospy.is_shutdown():
        pub.publish(pil_to_msg(topomap[i]))
        rospy.loginfo(f'Goal:{i}.png')
        if i < num_nodes - 1:
            i = i + 1 if args.ID == -1 else args.ID
        else:
            return
        rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Code to run GNMs on the locobot")
    parser.add_argument(
        "--name",
        "-n",
        default="topomap",
        type=str,
        help="topomap name",
    )
    parser.add_argument(
        "--duration",
        "-dt",
        default=5.,
        type=float,
        help="duration between goals",
    )
    parser.add_argument(
        "--ID",
        "-i",
        default=-1,
        type=int,
        help="the id of chosen goal",
    )
    args = parser.parse_args()
    main(args)
