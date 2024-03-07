import rospy
import math
import cv2 as cv
from gps_common.msg import GPSFix
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage
from PIL.ExifTags import TAGS, GPSTAGS

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

def get_gps(image):
    def convert_to_degrees(value):
        degrees = value[0].numerator / value[0].denominator
        minutes = value[1].numerator / value[1].denominator
        seconds = value[2].numerator / value[2].denominator
        return degrees + (minutes / 60.0) + (seconds / 3600.0)
    exif_data = {}
    exif_info = image._getexif()
    # print(exif_info)
    if exif_info:
        for tag, value in exif_info.items():
            tag_name = TAGS.get(tag, tag)
            exif_data[tag_name] = value
    else:
        return None
    gps_info = {}
    if 'GPSInfo' in exif_data:
        for key in exif_data['GPSInfo'].keys():
            tag_name = GPSTAGS.get(key,key)
            gps_info[tag_name] = exif_data['GPSInfo'][key]
    else:
        return None
    # print(gps_info)
    latitude = gps_info.get('GPSLatitude')
    longitude = gps_info.get('GPSLongitude')
    if latitude and longitude:
        latitude_ref = gps_info.get('GPSLatitudeRef', 'N')
        longitude_ref = gps_info.get('GPSLongitudeRef', 'E')
        latitude = convert_to_degrees(latitude)
        longitude = convert_to_degrees(longitude)
        if latitude_ref != 'N':
            latitude = 0 - latitude
        if longitude_ref != 'E':
            longitude = 0 - longitude
        return {'latitude':latitude,'longitude':longitude}
    else:
        return None

def draw():
    rospy.init_node('gps_visualization', anonymous=True)
    rospy.Subscriber("/gps/gps", GPSFix, callback)
    rospy.spin()

if __name__ == '__main__':
    image = PILImage.open('test.jpg')
    print(get_gps(image))