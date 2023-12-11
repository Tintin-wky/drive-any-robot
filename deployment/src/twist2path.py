import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import tf.transformations as tf_trans
import numpy as np

class TwistToPathNode:
    def __init__(self):
        rospy.init_node('twist_to_path_node')
        self.path_publisher = rospy.Publisher('/path', Path, queue_size=10)
        self.twist_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.twist_callback)
        self.path = Path()
        self.path.header.frame_id = 'odom'  # 设置适当的坐标系

    def twist_callback(self, twist_msg):
        global v,w,yaw,x,y,t
        if len(self.path.poses) == 0:
            v = 0
            w = 0
            yaw = 0
            x = 0
            y = 0
            t = rospy.Time.now()

        # 根据收到的twist消息更新姿势
        dt = (rospy.Time.now() - t).to_sec()
        x += v * np.cos(yaw) * dt
        y += v * np.sin(yaw) * dt
        v = twist_msg.linear.x
        w = twist_msg.angular.z
        yaw += twist_msg.angular.z * dt
        t = rospy.Time.now()

        pose = PoseStamped()
        pose.header.stamp =rospy.Time.now()
        pose.header.frame_id = self.path.header.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        orientation = tf_trans.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]


        # 将新的PoseStamped添加到Path消息中
        self.path.poses.append(pose)

        # 发布更新后的Path消息
        self.path_publisher.publish(self.path)
        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TwistToPathNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
