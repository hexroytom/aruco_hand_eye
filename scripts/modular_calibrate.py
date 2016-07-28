#!/usr/bin/env python
import rospy
from aruco_hand_eye import MoHandEyeConnector
import tf

def main():
    rospy.init_node('aruco_hand_eye')
    hec = MoHandEyeConnector()
    rospy.spin()
    hec.save_file()

if __name__ == '__main__':
    main()
