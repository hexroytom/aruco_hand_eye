# -*- coding: utf-8 -*-
"""
Created on Wed Jul 27 16:20:50 2016

@author: tom
"""

import ipdb
import rospy
import tf
import tf_conversions.posemath as tfconv

from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick

class MoHandEyeConnector(object):
    def __init__(self):

        # Frame which is rigidly attached to the camera
        # The transform from this frame to the camera optical frame is what
        # we're trying to compute.
        # For the eye-in-hand case, this is the end-effector frame.
        # For the eye-on-base case, this is the world or base frame.
        self.camera_parent_frame_id = rospy.get_param('~camera_parent_frame')

        # Frame which is rigidly attached to the marker
        # The transform from the camera parent frame to the marker parent frame
        # is given by forward kinematics.
        # For the eye-in-hand case, this is the world or base frame.
        # For the eye-on-base case, this is the end-effector frame.
        self.marker_parent_frame_id = rospy.get_param('~marker_parent_frame')

        self.publish_tf = rospy.get_param('~publish_tf')
        self.tf_suffix = rospy.get_param('~tf_suffix')
        self.sample_rate = rospy.get_param('~sample_rate')
        self.interactive = rospy.get_param('~interactive')

        # Compute the camera base to optical transform
        #self.xyz_optical_base = rospy.get_param('~xyz_optical_base', [0,0,0])
        #self.rpy_optical_base = rospy.get_param('~rpy_optical_base', [0,0,0])
        #self.F_optical_base = PyKDL.Frame(
        #        PyKDL.Rotation.RPY(*self.rpy_optical_base),
        #        PyKDL.Vector(*self.xyz_optical_base))
        #self.F_base_optical = self.F_optical_base.Inverse()

        # tf structures
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        # rate limiter
        self.rate = rospy.Rate(self.sample_rate)

        # input data
        self.hand_world_samples = TransformArray()
        self.camera_marker_samples = TransformArray()

        # marker subscriber
        self.aruco_subscriber = rospy.Subscriber(
                'aruco_tracker/transform',
                TransformStamped,
                self.aruco_cb,
                queue_size=1)

        # calibration service
        rospy.wait_for_service('compute_effector_camera_quick')
        self.calibrate = rospy.ServiceProxy(
                'compute_effector_camera_quick',
                compute_effector_camera_quick)
        
        #check if receive the marker pose       
        self.cmPub=rospy.Publisher('marker_pose',Transform,queue_size=10)
        
        #file for saving marker poses, pose_num stands for the number of poses
        self.file=open('/home/yake/cam_marker_pose.txt','w')
        self.pose_num=0
        
    def compute_calibration(self, msg):
        rospy.loginfo("Computing from %g poses..." % len(self.hand_world_samples.transforms) )
        result = None

        # Get the camera optical frame for convenience
        optical_frame_id = msg.header.frame_id

        try:
            result = self.calibrate(self.camera_marker_samples, self.hand_world_samples)
        except rospy.ServiceException as ex:
            rospy.logerr("Calibration failed: "+str(ex))
            return None

        if self.publish_tf:
            self.broadcaster.sendTransform(
                    (result.effector_camera.translation.x, result.effector_camera.translation.y, result.effector_camera.translation.z),
                    (result.effector_camera.rotation.x, result.effector_camera.rotation.y, result.effector_camera.rotation.z, result.effector_camera.rotation.w),
                    rospy.Time.now(),
                    optical_frame_id + self.tf_suffix,
                    self.camera_parent_frame_id)

        rospy.loginfo("Result:\n"+str(result))

        return result

    def aruco_cb(self, msg):

        rospy.loginfo("Received marker sample.")
        # Get the camera optical frame for convenience
        optical_frame_id = msg.header.frame_id

        self.camera_marker_samples.header.frame_id = optical_frame_id
        self.camera_marker_samples.transforms.append(msg.transform)

        # interactive
        if self.interactive:
            i = raw_input('Hit [s] to accept this latest sample, or [d] to discard: ')
            if i == 'd':
                del self.camera_marker_samples.transforms[-1]
            if i=='s':
                self.pose_num+=1
                #ipdb.set_trace()
                out_string=""
                out_string+=str(self.pose_num)+":\n"
                out_string+="  t: "+str(msg.transform.translation.x)+" "+str(msg.transform.translation.y)+" "+str(msg.transform.translation.z)+"\n"
                out_string+="  r: "+str(msg.transform.rotation.x)+" "+str(msg.transform.rotation.y)+" "+str(msg.transform.rotation.z)+" "+str(msg.transform.rotation.w)+"\n"
                self.file.write(out_string)
            raw_input('Hit any keys to continue ')
        else:
            self.rate.sleep()
        
        n_min = 2
        if len(self.camera_marker_samples.transforms) < n_min:
            rospy.logwarn("%d more samples needed..." % (n_min-len(self.hand_world_samples.transforms)))
        else:
            for tr in self.camera_marker_samples.transforms:
                self.cmPub.publish(tr)
        
        #save the file when the node is shutting down
    def save_file(self):
        self.file.close()