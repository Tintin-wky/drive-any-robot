import rospy
import rosbag
import matplotlib.pyplot as plt
from tqdm import tqdm

ODOM_TOPIC = "/odom_chassis"
# indoor
BAG_PATH_LIST = ["../topomaps/bags/raw/test1_2023-12-14-17-06-24.bag", \
                 "../topomaps/bags/navigate/gnm_classbot_1702544973_2023-12-14-17-09-34.bag", \
                 "../topomaps/bags/navigate/gnm_classbot_1702545079_2023-12-14-17-11-20.bag", \
                 "../topomaps/bags/navigate/gnm_classbot_1702545144_2023-12-14-17-12-25.bag", \
           ] 
# outdoor
# BAG_PATH_LIST = ["../topomaps/bags/raw/test2_2023-12-14-17-31-54.bag", \
#                  "../topomaps/bags/navigate/gnm_classbot_1702546523_2023-12-14-17-35-24.bag", \
#                  "../topomaps/bags/navigate/gnm_classbot_1702546737_2023-12-14-17-38-58.bag", \
#                  "../topomaps/bags/navigate/gnm_classbot_1702546849_2023-12-14-17-40-50.bag", \
#                  "../topomaps/bags/navigate/gnm_classbot_1702547003_2023-12-14-17-43-24.bag", \
#                  "../topomaps/bags/navigate/gnm_classbot_1702547137_2023-12-14-17-45-38.bag", \
#            ] 

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