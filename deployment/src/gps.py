import rospy
import math
import cv2 as cv
from gps_common.msg import GPSFix
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

img = cv.imread("map.png")
bridge = CvBridge()
def callback(data):
    rospy.loginfo("[longitude: %s latitude: %s]", data.longitude,data.latitude)
    cv.circle(img, center=trans(data), radius=1, color=[255, 0, 0], thickness=3)
    pub = rospy.Publisher('BirdEyeView', Image, queue_size=10)
    pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
    cv.imshow("Display window", cv.resize(img,(1024,1024)))
    cv.waitKey(3)
        

def trans(data,offset=(113.966,22.592),ratio=(187*1000,202*1000),theta=0):
    offset_x, offset_y = offset
    ratio_x, ratio_y = ratio
    x=1023+(data.longitude-offset_x)*ratio_x
    y=1023-(data.latitude-offset_y)*ratio_y
    x_new = int(round(x * math.cos(theta) - y * math.sin(theta)))
    y_new = int(round(x * math.sin(theta) + y * math.cos(theta)))
    return  (x_new,y_new)

def main():
    rospy.init_node('gps_visualization', anonymous=True)
    rospy.Subscriber("/gps/gps", GPSFix, callback)
    rospy.spin()

if __name__ == '__main__':
    main()