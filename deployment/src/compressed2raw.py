import rosbag
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import argparse
from tqdm import tqdm

def main(args:argparse.Namespace):
    rospy.init_node('compressed_to_raw_converter')
    bridge = CvBridge()
    
    with rosbag.Bag(args.output_bag, 'w') as outbag:
        for topic, msg, t in tqdm(rosbag.Bag(args.input_bag).read_messages(topics=[args.raw_topic]), desc="Converting"):
            if topic == args.compressed_topic:
                # Convert CompressedImage to Image
                try:
                    raw_image = bridge.compressed_imgmsg_to_cv2(msg)
                    image_msg = bridge.cv2_to_imgmsg(raw_image, encoding="bgr8")
                    image_msg.header = msg.header
                    outbag.write(args.raw_topic, image_msg, t)
                    print(f"Converted image at time {t}")
                except Exception as e:
                    print(f"Error converting image at time {t}: {str(e)}")
            else:
                # Pass through other messages unchanged
                outbag.write(topic, msg, t)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert compressed image to raw image in a ROS bag")
    parser.add_argument("input_bag", help="Input ROS bag file")
    parser.add_argument("output_bag", help="Output ROS bag file")
    parser.add_argument("compressed_topic", help="Compressed image topic in the input bag")
    parser.add_argument("raw_topic", help="Raw image topic in the output bag")
    args = parser.parse_args()
    main(args)