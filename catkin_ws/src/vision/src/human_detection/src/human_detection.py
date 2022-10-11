import pathlib
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray

ARGS = {
  "GPU_AVAILABLE": True,
  "MODELS_PATH": str(pathlib.Path(__file__).parent) + "/../models",
  "CONFIDENCE": 0.5,
  "SKIP_FRAMES": 5,
}


class Person:
  CLASSES = None
  with open(ARGS["MODELS_PATH"] + '/people/coco.names', 'r') as f:
    CLASSES = [line.strip() for line in f.readlines()]
  
  def __init__(self, xCentroid, yCentroid, x, y, w, h, tracker):
    self.point2D = (xCentroid, yCentroid)
    self.point3D = None
    self.tracker = tracker
    self.depth = None

class CameraProcessing:
    def __init__(self) -> None:
      self.bridge = CvBridge()
      self.depth_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.callback_depth)
      self.rgb_sub = rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color", Image, self.callback_rgb)
      self.rgb_sub_info = rospy.Subscriber("/zed2/zed_node/rgb/camera_info", CameraInfo, self.callback_rgb_info)
      self.publisherImage = rospy.Publisher("/zed2_/image/compressed", CompressedImage, queue_size = 1)
      self.publisherPeople = rospy.Publisher("/zed2_/people_count", Int16, queue_size = 10)
