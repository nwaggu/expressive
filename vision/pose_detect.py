import tensorflow as tf
import tensorflow_hub as hub
# from tensorflow_docs.vis import embed
import numpy as np
import cv2

# Import matplotlib libraries
from matplotlib import pyplot as plt
from matplotlib.collections import LineCollection
import matplotlib.patches as patches

# Some modules to display an animation using imageio.
import imageio
from IPython.display import HTML, display

import rospy
import time
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm


def testDevice(source):
   cap = cv2.VideoCapture(source) 
   if cap is None or not cap.isOpened():
       return False
   return True

class PoseDetect():
    def __init__(self):

        self.move_direction = 0
        hm.HelloNode.__init__(self)

        # Dictionary that maps from joint names to keypoint indices.
        self.KEYPOINT_DICT = {
            'nose': 0,
            'left_eye': 1,
            'right_eye': 2,
            'left_ear': 3,
            'right_ear': 4,
            'left_shoulder': 5,
            'right_shoulder': 6,
            'left_elbow': 7,
            'right_elbow': 8,
            'left_wrist': 9,
            'right_wrist': 10,
            'left_hip': 11,
            'right_hip': 12,
            'left_knee': 13,
            'right_knee': 14,
            'left_ankle': 15,
            'right_ankle': 16
        }

        # Maps bones to a matplotlib color name.
        self.KEYPOINT_EDGE_INDS_TO_COLOR = {
            (0, 1): 'm',
            (0, 2): 'c',
            (1, 3): 'm',
            (2, 4): 'c',
            (0, 5): 'm',
            (0, 6): 'c',
            (5, 7): 'm',
            (7, 9): 'm',
            (6, 8): 'c',
            (8, 10): 'c',
            (5, 6): 'y',
            (5, 11): 'm',
            (6, 12): 'c',
            (11, 12): 'y',
            (11, 13): 'm',
            (13, 15): 'm',
            (12, 14): 'c',
            (14, 16): 'c'
        }

        model_name = "movenet_lightning" #@param ["movenet_lightning", "movenet_thunder", "movenet_lightning_f16.tflite", "movenet_thunder_f16.tflite", "movenet_lightning_int8.tflite", "movenet_thunder_int8.tflite"]

        if "movenet_lightning" in model_name:
            self.module = hub.load("https://tfhub.dev/google/movenet/singlepose/lightning/4")
            self.input_size = 192
        elif "movenet_thunder" in model_name:
            self.module = hub.load("https://tfhub.dev/google/movenet/singlepose/thunder/4")
            self.input_size = 256
        else:
            raise ValueError("Unsupported model name: %s" % model_name)

    def movenet(self, input_image):
        """Runs detection on an input image.

        Args:
            input_image: A [1, height, width, 3] tensor represents the input image
            pixels. Note that the height/width should already be resized and match the
            expected input resolution of the model before passing into this function.

        Returns:
            A [1, 1, 17, 3] float numpy array representing the predicted keypoint
            coordinates and scores.
        """
        model = self.module.signatures['serving_default']
        # SavedModel format expects tensor type of int32.
        input_image = tf.cast(input_image, dtype=tf.int32)
        # Run model inference.
        outputs = model(input_image)
        # Output is a [1, 1, 17, 3] tensor.
        keypoints_with_scores = outputs['output_0'].numpy()
        return keypoints_with_scores

    def write_to_buffer(self, value):
        self.move_direction = value

    def read_buffer(self):
        return self.move_direction

    def issue_command(self):
        """
        Function that makes an action call and sends joint trajectory goals
        to a single joint
        :param self: The self reference.
        """
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = ['joint_head_pan']

        point0 = JointTrajectoryPoint()
        point0.positions = [0.65]

        # point1 = JointTrajectoryPoint()
        # point1.positions = [0.5]

        trajectory_goal.trajectory.points = [point0]#, point1]
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'
        self.trajectory_client.send_goal(trajectory_goal)
        rospy.loginfo('Sent goal = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()
    
    def stream_video(self):
        for i in range(100):
            if testDevice(i)==True:
                print("Success on : {i}" )

        cap = cv2.VideoCapture(5)

        while(cap.isOpened()):
            ret, frame = cap.read()
            if ret == True:
                # rotate frame from horizontal
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                # cv2.imshow('Frame', frame)

                # convert to tf image
                image = tf.expand_dims(frame, axis=0)
                image = tf.cast(tf.image.resize_with_pad(image, 1280, 1280), dtype=tf.int32)
                input_image = tf.image.resize_with_pad(image, self.input_size, self.input_size)

                # run model
                keypoints_with_scores = self.movenet(input_image)
                nose = keypoints_with_scores[0][0][0][0:2]
                # print(keypoints_with_scores)
                # height, width = frame.size
                width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

                frame_center = (int(height/2), int(width/2))
                nose_center = (int(height*nose[1]), int(width*nose[0]))

                direction_to_move = (frame_center[0]-nose_center[0], nose_center[1]-frame_center[1])
                direction_to_move /= np.linalg.norm(np.array(direction_to_move))

                # hm.HelloNode.main(self, 'issue_command', 'issue_command', wait_for_first_pointcloud=False)
                # rospy.loginfo('issuing command...')
                # self.issue_command()
                # time.sleep(2)

                # print(direction_to_move)

                self.write_to_buffer(direction_to_move)

                cv2.circle(frame, (nose_center), 3, (255, 0, 0), 3)
                cv2.circle(frame, (frame_center), 3, (0, 255, 0), 2)
                cv2.imshow("frame", frame)

                # break if "q" is pressed
                if cv2.waitKey(25) &0xFF == ord('q'):
                    cap.release()
                    cv2.destroyAllWindows()
                    break

            else:
                cap.release()
                cv2.destroyAllWindows()

if __name__ == "__main__":

    detect_pose = PoseDetect()
    detect_pose.stream_video()