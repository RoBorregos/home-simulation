from time import sleep
from typing import Tuple
import cv2
import mediapipe as mp
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision.msg import pose_point

indexToName = ["nose",
               "leftEyeInner",
               "leftEye",
               "leftEyeOuter",
               "rightEyeInner",
               "rightEye",
               "rightEyeOuter",
               "leftEar",
               "rightEar",
               "mouthLeft",
               "mouthRight",
               "leftShoulder",
               "rightShoulder",
               "leftElbow",
               "rightElbow",
               "leftWrist",
               "rightWrist",
               "leftPinky",
               "rightPinky",
               "leftIndex",
               "rightIndex",
               "leftThumb",
               "rightThumb",
               "leftHip",
               "rightHip",
               "leftKnee",
               "rightKnee",
               "leftAnkle",
               "rightAnkle",
               "leftHeel",
               "rightHeel",
               "leftFootIndex",
               "rightFootIndex"]


class PoseDetector:
    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.imageReceved = None

        self.bridge = CvBridge()

        rospy.init_node('PoseDetector')

        self.imageSub = rospy.Subscriber(
            "/hsrb/head_center_camera/image_raw", Image, self.image_callback)

        self.posePub = rospy.Publisher(
            "/hsrb/pose", pose_point, queue_size=10)

    def image_callback(self, data):
        self.imageReceved = data

    def run(self):
        with self.mp_pose.Pose(
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as pose:
            while True:
                if imageReceved is not None:
                    image = self.bridge.imgmsg_to_cv2(imageReceved, "rgb8")

                    image.flags.writeable = False
                    results = pose.process(image)

                    if results.pose_landmarks:
                        x = (
                            results.pose_landmarks.landmark[12].x + results.pose_landmarks.landmark[11].x) / 2
                        y = (
                            results.pose_landmarks.landmark[12].y + results.pose_landmarks.landmark[11].y) / 2
                        z = (
                            results.pose_landmarks.landmark[12].z + results.pose_landmarks.landmark[11].z) / 2
                        for(i, landmark) in enumerate(results.pose_landmarks.landmark):
                            self.posePub.publish(
                                indexToName[i], landmark.x, landmark.y, landmark.z)
                            print(indexToName[i])
                            print("x: ", x, "y: ", y, "z: ", z)
                        print("Chest: ")
                        print("x: ", x, "y: ", y, "z: ", z)
                        self.posePub.publish("chest", x, y, z)
                else:
                    print("Image not recived")
                sleep(1)


PoseDetector().run()
