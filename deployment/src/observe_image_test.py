#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():
    rospy.init_node('observe_image_test', anonymous=True)
    pub = rospy.Publisher('/camera/left/image_raw', Image, queue_size=10)
    rate = rospy.Rate(1)  # 发布频率为1Hz

    bridge = CvBridge()
    image_path = 'test.png'  # 替换为你的PNG图片的路径

    while not rospy.is_shutdown():
        try:
            # 读取PNG图片
            image = cv2.imread(image_path)
            if image is not None:
                # 将OpenCV图像转换为ROS图像消息
                ros_image = bridge.cv2_to_imgmsg(image, encoding='bgr8')
                pub.publish(ros_image)
                rospy.loginfo('Publishing image...')
            else:
                rospy.logwarn('Image not found!')
        except Exception as e:
            rospy.logerr(str(e))

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass