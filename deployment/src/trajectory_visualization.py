import rospy
import rosbag
import matplotlib.pyplot as plt
from tqdm import tqdm
import pickle

TOPOMAPS="../topomaps/topomaps.pkl"
ODOM_TOPIC = "/odom_chassis"
NAME='outdoor'
BAG_PATH_LIST = ["../bags/explore/outdoor1_1703408954_2023-12-24-17-09-15.bag"]

def main():
    rospy.init_node("trajectory_visualization", anonymous=False)

    plt.figure()
    i = 0
    for bag_path in BAG_PATH_LIST:
        bag = rosbag.Bag(bag_path)
        x = []
        y = []
        for topic, msg, t in tqdm(bag.read_messages(topics=[ODOM_TOPIC]),total=bag.get_message_count(topic_filters=[ODOM_TOPIC]), desc="Rosbag " + str(i)):
            x.append(msg.pose.pose.position.x)
            y.append(msg.pose.pose.position.y)
        plt.plot(x, y, label='Trajectory ' + str(i))
        i += 1
    
    with open(TOPOMAPS, 'rb') as file:
        topomap = pickle.load(file)[NAME]
    x_nodes=[]
    y_nodes=[]
    for i in topomap.nodes():
        x_nodes.append(topomap.nodes[i]['pose'].position.x)
        y_nodes.append(topomap.nodes[i]['pose'].position.y)
    plt.scatter(x_nodes,y_nodes,c='red')
    
    # 添加图例
    plt.legend()

    # 添加标题和轴标签
    plt.title("Multiple Trajectory Plots")
    plt.xlabel("X Axis")
    plt.ylabel("Y Axis")

    # 显示图表
    plt.show()

if __name__ == "__main__":
    main()