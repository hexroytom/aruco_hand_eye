#!/usr/bin/env python
from aruco_hand_eye import MoHandEyeCalibrator

if __name__ == '__main__':
    mhec = MoHandEyeCalibrator("/home/yake/new.txt")
    mhec.read_files()