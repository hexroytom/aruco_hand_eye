#!/usr/bin/env python
import rospy
from aruco_hand_eye import MoHandEyeCalibrator

if __name__ == '__main__':
    rospy.init_node('BIRL_hand_eye_calibrate')
    mhec = MoHandEyeCalibrator("/home/yake/new.txt","/home/yake/xyzrpy.txt")
    mhec.read_files()
    mhec.compute_calibration()
