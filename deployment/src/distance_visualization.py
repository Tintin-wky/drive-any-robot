import rospy
import rosbag
import argparse
import pickle
import matplotlib.pyplot as plt

ODOM_DISTANCE_TOPIC = "/odom_distance"
TOPOMAP_IMAGES_DIR = "../topomaps/images"

def main(args:argparse.Namespace):
    rospy.init_node("distance_visualization", anonymous=False)

    topomap_path = f"../topomaps/{args.name}.pkl"
    with open(topomap_path, 'rb') as file:
        topomap = pickle.load(file)
    path=topomap.path[0]
    x=[]
    y=[]
    i=0
    flag =True
    x_ = []
    y_ = []
    distance = []
    node = []
    bag = rosbag.Bag(args.bagpath)
    for topic, msg, t in bag.read_messages(topics=[ODOM_DISTANCE_TOPIC]):
        x.append(msg.data[1])
        y.append(msg.data[2])
        if msg.data[3]==path[i]:
            distance.append(msg.data[0])
            x_.append(msg.data[1])
            y_.append(msg.data[2])
            flag = False
        else:
            if flag is False:
                if i < len(path)-1:
                    i=i+1
                flag = True
    plt.scatter(x_, y_, c=distance)
    plt.colorbar()
    plt.plot(x, y)
    x_nodes=[]
    y_nodes=[]
    for i in range(topomap.number_of_nodes()):
        x_nodes.append(topomap.nodes[i]['pose'].position.x)
        y_nodes.append(topomap.nodes[i]['pose'].position.y)
        if i == 10:
            plt.scatter(x_nodes,y_nodes,c='yellow')
            x_nodes.clear()
            y_nodes.clear()
        if i == 20:
            plt.scatter(x_nodes,y_nodes,c='orange')
            x_nodes.clear()
            y_nodes.clear()
    plt.scatter(x_nodes,y_nodes,c='red')
    x_nodes.clear()
    y_nodes.clear()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Distance Distribution on XY plane')
    plt.show()
    bag.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Code to run GNMs on the locobot")
    parser.add_argument(
        "--bagpath",
        "-i",
        default="odom_distance_2023-12-19-21-07-02.bag",
        type=str,
        help="path to rosbag",
    )
    parser.add_argument(
        "--name",
        "-n",
        default="topomap",
        type=str,
        help="name of your topomap (default: topomap)",
    )
    args = parser.parse_args()
    main(args)
