#!/usr/bin/env python

# ---------------------------------------------------
# import rosbag
# import cv2
# from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge
# from tqdm import tqdm
# import argparse

# def convert_compressed_images_to_video(input_bag, output_video, compressed_topic, frame_rate):
#     bridge = CvBridge()
#     fourcc = cv2.VideoWriter_fourcc(*'XVID')
#     video_writer = None
#     initialized = False

#     with rosbag.Bag(input_bag, 'r') as bag:
#         for topic, msg, t in tqdm(bag.read_messages(topics=[compressed_topic]), total=bag.get_message_count(topic_filters=[compressed_topic]), desc="Converting"):
#             try:
#                 img_np = bridge.compressed_imgmsg_to_cv2(msg)
#                 if not initialized:
#                     height, width, _ = img_np.shape
#                     video_writer = cv2.VideoWriter(output_video, fourcc, frame_rate, (width, height))
#                     initialized = True

#                 video_writer.write(img_np)

#             except Exception as e:
#                 print(f"Error converting image at time {t}: {str(e)}")

#     if video_writer is not None:
#         video_writer.release()
#         print(f"Video saved as {output_video}")

# if __name__ == "__main__":
#     parser = argparse.ArgumentParser(description="Convert compressed image messages in a ROS bag to a video")
#     parser.add_argument("input_bag", help="Input ROS bag file")
#     parser.add_argument("output_video", help="Output video file")
#     parser.add_argument("compressed_topic", help="Compressed image topic in the bag")
#     parser.add_argument("--frame_rate", type=int, default=30, help="Frame rate for the output video")

#     args = parser.parse_args()

#     convert_compressed_images_to_video(args.input_bag, args.output_video, args.compressed_topic, args.frame_rate)

# ---------------------------------------------------
import rosbag
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tqdm import tqdm
import argparse

def convert_raw_images_to_video(input_bag, output_video, raw_topic, frame_rate):
    bridge = CvBridge()
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video_writer = None
    initialized = False

    with rosbag.Bag(input_bag, 'r') as bag:
        for topic, msg, t in tqdm(bag.read_messages(topics=[raw_topic]), total=bag.get_message_count(topic_filters=[raw_topic]), desc="Converting"):
            try:
                img_np = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                if not initialized:
                    height, width, _ = img_np.shape
                    video_writer = cv2.VideoWriter(output_video, fourcc, frame_rate, (width, height))
                    initialized = True

                video_writer.write(img_np)

            except Exception as e:
                print(f"Error converting image at time {t}: {str(e)}")

    if video_writer is not None:
        video_writer.release()
        print(f"Video saved as {output_video}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert raw image messages in a ROS bag to a video")
    parser.add_argument("input_bag", help="Input ROS bag file")
    parser.add_argument("output_video", help="Output video file")
    parser.add_argument("raw_topic", help="Raw image topic in the bag")
    parser.add_argument("--frame_rate", type=int, default=30, help="Frame rate for the output video")

    args = parser.parse_args()

    convert_raw_images_to_video(args.input_bag, args.output_video, args.raw_topic, args.frame_rate)

# -----------------------