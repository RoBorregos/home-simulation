from time import sleep
import cv2
import mediapipe as mp
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose
imageReceved = None

bridge = CvBridge()


def image_callback(data):
    global imageReceved
    print("Recived cam")
    imageReceved = data


rospy.init_node('ImageRecever', anonymous=True)

imageSub = rospy.Subscriber(
    "/hsrb/head_center_camera/image_raw", Image, image_callback)

# rosbag play 2022-10-06-19-23-37-09.bag

with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as pose:
    while True:
        if imageReceved is not None:
            image = bridge.imgmsg_to_cv2(imageReceved, "rgb8")

            image.flags.writeable = False
            results = pose.process(image)

            # print(results.pose_landmarks.landmark[12])
            # print(results.pose_landmarks.landmark[11])
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            if results.pose_landmarks:

                x = (
                    results.pose_landmarks.landmark[12].x - results.pose_landmarks.landmark[11].x) / 2
                y = (
                    results.pose_landmarks.landmark[12].y - results.pose_landmarks.landmark[11].y) / 2
                z = (
                    results.pose_landmarks.landmark[12].z - results.pose_landmarks.landmark[11].z) / 2

                cv2.circle(image, (int(x * 640), int(y * 480)),
                           5, (0, 0, 255), -1)

                mp_drawing.draw_landmarks(
                    image,
                    results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

                cv2.imshow('MediaPipe Pose', image)
                if cv2.waitKey(5) & 0xFF == 27:
                    break
        else:
            print("Image not recived")
        sleep(1)
