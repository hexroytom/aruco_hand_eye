# -*- coding: utf-8 -*-
"""
Created on Thu Jul 28 15:33:01 2016

@author: tom
"""

import rospy
import copy
import math
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick
from tf import transformations

class MoHandEyeCalibrator(object):
    def __init__(self,cam_marker_file,base_ee_file):        
        
        #save file names
        self.name_cam_marker_file=cam_marker_file 
        self.name_base_ee_file=base_ee_file
        
        #count how many camera-marker poses in the file
        self.camera_marker_file=open(self.name_cam_marker_file,'r')
        self.num_of_lines_of_cam_marker_file=sum(1 for line in self.camera_marker_file)
        self.num_of_cam_marker_poses=self.num_of_lines_of_cam_marker_file/3
        
        #initiate marker poses w.r.t camera
        self.camera_marker_samples=TransformArray()
        self.camera_marker_samples.header.frame_id="/camera_rgb_optical_frame"
        
        #count how many base-end-effector poses in the file        
        self.base_ee_file=open(self.name_base_ee_file,'r')
        self.num_of_lines_of_base_ee_file=sum(1 for line in self.base_ee_file)
        self.num_of_base_ee_poses=self.num_of_lines_of_base_ee_file
        
        if self.num_of_cam_marker_poses != self.num_of_base_ee_poses:
            print 'number of camera-marker poses not equal to number of base-ee poses!\n'
            print 'please ctrl+c to exit\n'
        
        #initiate ee poses w.r.t base
        self.base_ee_samples=TransformArray()
        self.base_ee_samples.header.frame_id="/base"        
        
    def read_files(self):
        
        #initialize the reader        
        j=0
        tmp_transform=Transform()
        self.camera_marker_file=open(self.name_cam_marker_file,'r')
        
        #read camera marker pose
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
#        for tr in self.camera_marker_samples.transforms:
#            print tr
#            print "\n"
                
        self.base_ee_file=open(self.name_base_ee_file,'r')
        for i in range(self.num_of_lines_of_base_ee_file):
            line=self.base_ee_file.readline()
            k=0
            b=list()
            transform=Transform()
            while k<len(line):
                if line[k]==" " or line[k]=="\n":
                    b.append(k)
                k+=1
        
            #get translation
            transform.translation.x=float(line[0:b[0]])
            transform.translation.y=float(line[(b[1]+1):b[2]])
            transform.translation.z=float(line[(b[3]+1):b[4]])
    
            #get rotation
            r=(float(line[(b[5]+1):b[6]])/180)*math.pi
            p=(float(line[(b[7]+1):b[8]])/180)*math.pi
            y=(float(line[(b[9]+1):b[10]])/180)*math.pi
            quat=transformations.quaternion_from_euler(r, p, y, 'sxyz')
            transform.rotation=quat
            self.base_ee_samples.transforms.append(transform)
        
        #print self.base_ee_samples

                


        

                    