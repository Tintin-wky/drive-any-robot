import rospy
import os
import io
import argparse
from PIL import Image as PILImage
from sensor_msgs.msg import CompressedImage

IMAGE_TOPIC = "/camera/left/image_raw/compressed"
DESTINATION_DIR="../destination"

def callback_obs(msg):
    obs_img = PILImage.open(io.BytesIO(msg.data))
    obs_img.save(os.path.join(DESTINATION_DIR, f"{args.destination}.png"))
    rospy.loginfo("Image saved")
    rospy.signal_shutdown("Image saved, shutting down")

def main(args: argparse.Namespace):
    rospy.init_node("save_destination", anonymous=False)
    image_curr_msg = rospy.Subscriber(IMAGE_TOPIC, CompressedImage, callback_obs, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Code to run GNMs on the classbot")
    parser.add_argument(
        "--destination",
        "-d",
        default="test",
        type=str,
        help="name of destination",
    )
    args = parser.parse_args()
    main(args)