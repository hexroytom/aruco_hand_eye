# -*- coding: utf-8 -*-
"""
Created on Thu Jul 28 15:33:01 2016

@author: tom
"""

import rospy
import copy
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick

class MoHandEyeCalibrator(object):
    def __init__(self,cam_marker_file):        
        
        #save file names
        self.cam_marker_file=cam_marker_file        
        
        #count how many camera-marker poses in the file
        self.camera_marker_file=open(self.cam_marker_file,'r')
        self.num_of_lines_of_cam_marker_file=sum(1 for line in self.camera_marker_file)
        self.num_of_cam_marker_poses=self.num_of_lines_of_cam_marker_file/3
        
        #initiate marker poses w.r.t camera
        self.camera_marker_samples=TransformArray()
        self.camera_marker_samples.header.frame_id="/camera_rgb_optical_frame"
    
    def read_files(self):
        
        #initialize the reader        
        j=0
        tmp_transform=Transform()
        self.camera_marker_file=open(self.cam_marker_file,'r')
        
        for i in range(self.num_of_lines_of_cam_marker_file):
            line=self.camera_marker_file.readline()
            if i==j*3+1:
                b=list()
                k=5                
                while k<len(line):
                    if line[k]==" " or line[k]=="\n":
                        b.append(k)
                    k+=1
                tmp_transform.translation.x=float(line[5:b[0]])
                tmp_transform.translation.y=float(line[(b[0]+1):b[1]])
                tmp_transform.translation.z=float(line[(b[1]+1):b[2]])

            if i==j*3+2:
                b=list()
                k=5
                while k<len(line):
                    if line[k]==" " or line[k]=="\n":
                        b.append(k)
                    k+=1
                tmp_transform.rotation.x=float(line[5:b[0]])
                tmp_transform.rotation.y=float(line[(b[0]+1):b[1]])
                tmp_transform.rotation.z=float(line[(b[1]+1):b[2]])
                tmp_transform.rotation.w=float(line[(b[2]+1):b[3]])
                self.camera_marker_samples.transforms.append(copy.deepcopy(tmp_transform))
                j+=1
        for tr in self.camera_marker_samples.transforms:
            print tr
            print "\n"

                


        

                    