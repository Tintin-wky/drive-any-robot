import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def publish_path():
    # 初始化 ROS 节点
    rospy.init_node('path_publisher_node')

    # 创建一个发布者，发布 Path 消息到指定话题
    path_publisher = rospy.Publisher('/path', Path, queue_size=10)

    # 创建一个 Path 消息
    path_msg = Path()
    path_msg.header.frame_id = 'odom'  # 设置适当的坐标系

    # 创建一些 PoseStamped 消息并将它们添加到 Path 中
    for i in range(10):  # 假设添加了10个路径点
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()  # 时间戳
        pose.header.frame_id = 'odom'  # 设置适当的坐标系
        pose.pose.position.x = i  # 设置 x 坐标
        pose.pose.position.y = 2 * i  # 设置 y 坐标
        pose.pose.orientation.w = 1.0  # 设置四元数的 w 分量为 1.0，表示无旋转
        path_msg.poses.append(pose)

    rate = rospy.Rate(1)  # 设置发布频率为 1 Hz

    while not rospy.is_shutdown():
        # 发布 Path 消息
        path_publisher.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass