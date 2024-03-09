from pose_detect import PoseDetect

pose = PoseDetect()
direction = pose.read_buffer()
print(direction)