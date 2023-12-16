import rospy
import rosbag
import argparse
import matplotlib.pyplot as plt

ODOM_DISTANCE_TOPIC = "/odom_distance"

def main(args:argparse.Namespace):
    rospy.init_node("distance_visualization", anonymous=False)

    x = []
    y = []
    distance = []
    bag = rosbag.Bag(args.bagpath)
    for topic, msg, t in bag.read_messages(topics=[ODOM_DISTANCE_TOPIC]):
        distance.append(msg.data[0])
        x.append(msg.data[1])
        y.append(msg.data[2])

    plt.scatter(x, y, c=distance)
    plt.colorbar()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Height Distribution on XY plane')
    plt.show()
    bag.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Code to run GNMs on the locobot")
    parser.add_argument(
        "--bagpath",
        "-i",
        default="../topomaps/bags/raw/odom_distance_2023-12-16-23-01-13.bag",
        type=str,
        help="path to rosbag",
    )
    args = parser.parse_args()
    main(args)
