#!/usr/bin/env python

################################################################################
# DO NOT MODIFY - AUTO-GENERATED
# 
# Copyright (c) 2016, McGill University / Independent Robotics Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

import rospy
import tf
import sys
from math import pi
from std_msgs.msg import Float32 
from aquacore.msg import Command
from aquacore.msg import AutopilotModes
from geometry_msgs.msg import PoseStamped
from aquacore.msg import StateMsg
from aquacore.srv import SetAutopilotMode  
import dynamic_reconfigure.client

class AutopilotClient:
    
    def __init__(self, params):
        self.mode = params['mode']
        self.listener = tf.TransformListener()
        self.target_pose_pub = rospy.Publisher('/aqua/target_pose', PoseStamped, queue_size=3)
        self.current_depth = 0

        rospy.wait_for_service('/aqua/set_3Dauto_mode')
        rospy.wait_for_message('/AP_filtered_depth', Float32)
        
        self.depth_sub = rospy.Subscriber("/AP_filtered_depth", Float32, self.depth_callback)
        self.set_3d_auto_mode = rospy.ServiceProxy('/aqua/set_3Dauto_mode', SetAutopilotMode)
        
        self.listener.waitForTransform('/latest_fix', '/aqua_base', rospy.Time(0), rospy.Duration(4))

        self.dyn_reconf = dynamic_reconfigure.client.Client("/AP_depth_filter", timeout=5)  
        self.original_params = self.dyn_reconf.get_configuration(timeout=5) 
        print 'Previous value of window_size_sec  was: ' + str(self.original_params['window_size_sec'])

        rospy.loginfo('Setting window_size_sec to ' + str(0.5)) 
        self.dyn_reconf.update_configuration({'window_size_sec':float(0.5)}) 
        rospy.loginfo('Returned from update_configuration')

        try:
            self.resp1 = self.set_3d_auto_mode(mode=self.mode)
        except rospy.ServiceException as exc:
            print("Could not set 3d autopilot mode")

    def set_mode(self,params):
        self.mode = params['mode']
        try:
            self.resp1 = self.set_3d_auto_mode(mode=self.mode)
        except rospy.ServiceException as exc:
            print("Could not set 3d autopilot mode")

    def angle_diff(self, from_theta, to_theta):
        diff = to_theta - from_theta
        if (diff > pi):
            diff = diff - 2*pi
    
        if (diff < -pi):
            diff = diff + 2*pi
        
        return diff

   
    def get_rpy_of_imu_in_global(self):
        self.listener.waitForTransform('/latest_fix', '/aqua_base', rospy.Time(0), rospy.Duration(4))
        (position_of_imu_in_global, rotation_from_imu_to_global) = \
            self.listener.lookupTransform('/latest_fix', '/aqua_base', rospy.Time(0))
        rpy_from_imu_to_global = tf.transformations.euler_from_quaternion(rotation_from_imu_to_global)
        return rpy_from_imu_to_global


    def depth_callback(self, msg):
        self.current_depth = msg.data

    def do_straight_line(self, dt_in_sec, target_angles_in_deg, target_depth, vx, vz):
    
        rate = rospy.Rate(10);
        started_at = rospy.Time.now()

        #while (not rospy.is_shutdown()) and ((rospy.Time.now() - started_at).to_sec() < dt_in_sec):
        while (not rospy.is_shutdown()) and (rospy.Time.now() < (started_at + dt_in_sec)):
            rotation_from_target_to_global = tf.transformations.quaternion_from_euler(target_angles_in_deg[0]*pi/180, 
                                                                                      target_angles_in_deg[1]*pi/180, 
                                                                                      target_angles_in_deg[2]*pi/180)
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx # forward speed
            pose_and_vel.pose.position.y = vz # heave
            pose_and_vel.pose.position.z = target_depth 
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel)
            rate.sleep()

    def goto_target_orientation(self, target_angles_in_deg, target_depth, vx, vz):
        
        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(target_angles_in_deg[0]*pi/180, 
                                                                                  target_angles_in_deg[1]*pi/180, 
                                                                                  target_angles_in_deg[2]*pi/180)

        target_angles = tf.transformations.euler_from_quaternion(rotation_from_target_to_global)

        rate = rospy.Rate(10.0)
        while (not rospy.is_shutdown()) and \
                ((abs(self.angle_diff(rpy_from_imu_to_global[0], target_angles[0]))*180/pi > 5) or \
                 (abs(self.angle_diff(rpy_from_imu_to_global[1], target_angles[1]))*180/pi > 5) or \
                 (abs(self.angle_diff(rpy_from_imu_to_global[2], target_angles[2]))*180/pi > 5)):
            
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx
            pose_and_vel.pose.position.y = vz # heave
            pose_and_vel.pose.position.z = target_depth 
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'
            self.target_pose_pub.publish(pose_and_vel) 
            rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
            rate.sleep()

            print 'angle diff: ', (abs(self.angle_diff(rpy_from_imu_to_global[0], target_angles[0]))*180/pi, 
                                   abs(self.angle_diff(rpy_from_imu_to_global[1], target_angles[1]))*180/pi, 
                                   abs(self.angle_diff(rpy_from_imu_to_global[2], target_angles[2]))*180/pi)
        


    def do_relative_angle_change(self, delta_angles_deg, target_depth, vx, vz):

        print 'da: ', delta_angles_deg
        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
        
        print 'rpy: ', (rpy_from_imu_to_global[0]*180/pi, rpy_from_imu_to_global[1]*180/pi, rpy_from_imu_to_global[2]*180/pi)
        target_roll  = rpy_from_imu_to_global[0] + delta_angles_deg[0]*pi/180
        target_pitch = rpy_from_imu_to_global[1] + delta_angles_deg[1]*pi/180
        target_yaw   = rpy_from_imu_to_global[2] + delta_angles_deg[2]*pi/180
        print 'Ta: ', (target_roll*180/pi, target_pitch*180/pi, target_yaw*180/pi)
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(target_roll, 
                                                                                  target_pitch, 
                                                                                  target_yaw)

        target_angles = tf.transformations.euler_from_quaternion(rotation_from_target_to_global)
        print 'Trimmed ta: ', (target_angles[0]*180/pi, target_angles[1]*180/pi, target_angles[2]*180/pi)
        
        print 'angle diff: ', (abs(self.angle_diff(rpy_from_imu_to_global[0], target_angles[0]))*180/pi, 
                               abs(self.angle_diff(rpy_from_imu_to_global[1], target_angles[1]))*180/pi, 
                               abs(self.angle_diff(rpy_from_imu_to_global[2], target_angles[2]))*180/pi)
        

        rate = rospy.Rate(10.0)
        while (not rospy.is_shutdown()) and \
                ((abs(self.angle_diff(rpy_from_imu_to_global[0], target_angles[0]))*180/pi > 5) or \
                (abs(self.angle_diff(rpy_from_imu_to_global[1], target_angles[1]))*180/pi > 5) or \
                (abs(self.angle_diff(rpy_from_imu_to_global[2], target_angles[2]))*180/pi > 5)):
        
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx
            pose_and_vel.pose.position.y = vz # heave
            pose_and_vel.pose.position.z = target_depth 
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel) 
            rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
            rate.sleep()
            
            print 'angle diff: ', (abs(self.angle_diff(rpy_from_imu_to_global[0], target_angles[0]))*180/pi, 
                                   abs(self.angle_diff(rpy_from_imu_to_global[1], target_angles[1]))*180/pi, 
                                   abs(self.angle_diff(rpy_from_imu_to_global[2], target_angles[2]))*180/pi)
        


    def do_relative_depth_change(self, dz, vx, vz):

        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(rpy_from_imu_to_global[0], 
                                                                                  rpy_from_imu_to_global[1], 
                                                                                  rpy_from_imu_to_global[2])
        target_angles = rpy_from_imu_to_global
        target_depth = self.current_depth + dz
        rate = rospy.Rate(10.0)
        
        while (not rospy.is_shutdown()) and \
                ((abs(self.angle_diff(rpy_from_imu_to_global[0], target_angles[0]))*180/pi > 5) or \
                 (abs(self.angle_diff(rpy_from_imu_to_global[1], target_angles[1]))*180/pi > 5) or \
                 (abs(self.angle_diff(rpy_from_imu_to_global[2], target_angles[2]))*180/pi > 5) or \
                 (abs(target_depth - self.current_depth) > 0.2)):
            
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx
            pose_and_vel.pose.position.y = vz # heave
            pose_and_vel.pose.position.z = target_depth 
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel) 
            rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
            rate.sleep()

    def corkscrew_at_angle(self, dt_in_sec, target_depth, target_angle_in_deg, vx, vz, wx_in_deg_per_sec):

        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(target_angle_in_deg[0]*pi/180.0, 
                                                                                  target_angle_in_deg[1]*pi/180.0, 
                                                                                  target_angle_in_deg[2]*pi/180.0)
        T = 0.1
        wx = wx_in_deg_per_sec * pi/180
        rate = rospy.Rate(1.0/T)
        starting_time = rospy.Time.now()
        rotation_from_old_target_to_new_target = tf.transformations.quaternion_from_euler(wx*T, 0, 0)

        while (not rospy.is_shutdown()) and ((starting_time + dt_in_sec) > rospy.Time.now()):
        
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx
            pose_and_vel.pose.position.y = vz # heave
            pose_and_vel.pose.position.z = target_depth 
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel) 
            rate.sleep()

            rotation_from_target_to_global = tf.transformations.quaternion_multiply(rotation_from_target_to_global,
                                                                                    rotation_from_old_target_to_new_target)

    def do_corkscrew(self, dt_in_sec, target_depth, vx, vz, wx_in_deg_per_sec):

        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(rpy_from_imu_to_global[0], 
                                                                                  rpy_from_imu_to_global[1], 
                                                                                  rpy_from_imu_to_global[2])
        T = 0.1
        wx = wx_in_deg_per_sec * pi/180
        rate = rospy.Rate(1.0/T)
        starting_time = rospy.Time.now()
        rotation_from_old_target_to_new_target = tf.transformations.quaternion_from_euler(wx*T, 0, 0)

        while (not rospy.is_shutdown()) and ((starting_time + dt_in_sec) > rospy.Time.now()):
        
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx
            pose_and_vel.pose.position.y = vz # heave
            pose_and_vel.pose.position.z = target_depth 
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel) 
            rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
            rate.sleep()

            rotation_from_target_to_global = tf.transformations.quaternion_multiply(rotation_from_target_to_global,
                                                                                    rotation_from_old_target_to_new_target)

    def do_circular_corkscrew(self, dt_in_sec, vx, vz, wx_in_deg_per_sec, wz_in_deg_per_sec):
        pass
    
    def do_rolling_sideways_somersault(self, dt_in_sec, vx, vz, target_depth, wy_in_deg_per_sec):

        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(pi/2.0, 
                                                                                  0.0, 
                                                                                  rpy_from_imu_to_global[2])
        T = 0.1
        wy = wy_in_deg_per_sec * pi/180
        rate = rospy.Rate(1.0/T)
        starting_time = rospy.Time.now()
        rotation_from_old_target_to_new_target = tf.transformations.quaternion_from_euler(0, wy*T, 0)
        
        depth = self.current_depth
        t = 0.0

        while (not rospy.is_shutdown()) and ((starting_time + dt_in_sec) > rospy.Time.now()):
            
            rotation_from_target_to_global = tf.transformations.quaternion_from_euler(2*wy*t, 
                                                                                  0.0, 
                                                                                  rpy_from_imu_to_global[2] + wy*t)
        
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx
            pose_and_vel.pose.position.y = vz 
            pose_and_vel.pose.position.z = target_depth
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel) 
            
            rate.sleep()
            t = t + T
            
#    def do_rolling_sideways_somersault(self, dt_in_sec, vx, vz, wy_in_deg_per_sec):
#
#        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
#        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(pi/2.0, 
#                                                                                  0.0, 
#                                                                                  rpy_from_imu_to_global[2])
#        T = 0.1
#        wy = wy_in_deg_per_sec * pi/180
#        rate = rospy.Rate(1.0/T)
#        starting_time = rospy.Time.now()
#        rotation_from_old_target_to_new_target = tf.transformations.quaternion_from_euler(0, wy*T, 0)
#        
#        depth = self.current_depth
#        t = 0.0
#
#        while (not rospy.is_shutdown()) and ((starting_time + dt_in_sec) > rospy.Time.now()):
#            
#            rotation_from_target_to_global = tf.transformations.quaternion_from_euler(2*wy*t, 
#                                                                                  0.0, 
#                                                                                  rpy_from_imu_to_global[2] + wy*t)
#        
#            pose_and_vel = PoseStamped()
#            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
#            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
#            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
#            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
#            pose_and_vel.pose.position.x = vx
#            pose_and_vel.pose.position.y = vz 
#            pose_and_vel.pose.position.z = depth
#            pose_and_vel.header.stamp = rospy.Time.now()
#            pose_and_vel.header.frame_id = '/latest_fix'
#
#            self.target_pose_pub.publish(pose_and_vel) 
#            
#            rate.sleep()
#            t = t + T            
            
    
    def do_sideways_somersault(self, dt_in_sec, vx, vz, target_depth, wy_in_deg_per_sec):

        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(pi/2.0, 
                                                                                  0.0, 
                                                                                  rpy_from_imu_to_global[2])
        T = 0.1
        wy = wy_in_deg_per_sec * pi/180
        rate = rospy.Rate(1.0/T)
        starting_time = rospy.Time.now()
        rotation_from_old_target_to_new_target = tf.transformations.quaternion_from_euler(0, wy*T, 0)
        
        depth = self.current_depth

        while (not rospy.is_shutdown()) and ((starting_time + dt_in_sec) > rospy.Time.now()):
        
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx
            pose_and_vel.pose.position.y = vz 
            pose_and_vel.pose.position.z = target_depth
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel) 
            rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
            rate.sleep()

            rotation_from_target_to_global = tf.transformations.quaternion_multiply(rotation_from_target_to_global,
                                                                                    rotation_from_old_target_to_new_target)
    
    def do_vertical_somersault(self, dt_in_sec, vx, vz, wy_in_deg_per_sec):

        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(0.0, 
                                                                                  0.0, 
                                                                                  rpy_from_imu_to_global[2])
        T = 0.1
        wy = wy_in_deg_per_sec * pi/180
        rate = rospy.Rate(1.0/T)
        starting_time = rospy.Time.now()
        rotation_from_old_target_to_new_target = tf.transformations.quaternion_from_euler(0, wy*T, 0)

        while (not rospy.is_shutdown()) and ((starting_time + dt_in_sec) > rospy.Time.now()):
        
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx
            pose_and_vel.pose.position.y = vz 
            pose_and_vel.pose.position.z = self.current_depth
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel) 
            rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
            rate.sleep()

            rotation_from_target_to_global = tf.transformations.quaternion_multiply(rotation_from_target_to_global,
                                                                                    rotation_from_old_target_to_new_target)

    def do_45_somersault(self, dt_in_sec, vx, vz, roll_target, wy_in_deg_per_sec):

        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(roll_target, 
                                                                                  0.0, 
                                                                                  rpy_from_imu_to_global[2])
        T = 0.1
        wy = wy_in_deg_per_sec * pi/180
        rate = rospy.Rate(1.0/T)
        starting_time = rospy.Time.now()
        rotation_from_old_target_to_new_target = tf.transformations.quaternion_from_euler(0, wy*T, 0)

        while (not rospy.is_shutdown()) and ((starting_time + dt_in_sec) > rospy.Time.now()):
        
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx
            pose_and_vel.pose.position.y = vz 
            pose_and_vel.pose.position.z = self.current_depth
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel) 
            rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
            rate.sleep()

            rotation_from_target_to_global = tf.transformations.quaternion_multiply(rotation_from_target_to_global,
                                                                                    rotation_from_old_target_to_new_target)



    def do_lateral_motion(self, dt_in_sec, target_angles_in_deg, target_depth, vx, vz):
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(target_angles_in_deg[0]*pi/180, 
                                                                                  target_angles_in_deg[1]*pi/180, 
                                                                                  target_angles_in_deg[2]*pi/180)
        starting_time = rospy.Time.now()
        rate = rospy.Rate(10.0)

        while (not rospy.is_shutdown()) and ((starting_time + dt_in_sec) > rospy.Time.now()):

            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx
            pose_and_vel.pose.position.y = vz # heave
            pose_and_vel.pose.position.z = target_depth 
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel) 
            rate.sleep()
            
    def clamp180(self,angle):
        while angle > 180.0:
            angle = angle - 360.0
        while angle < -180.0:
            angle = angle + 360.0
            
        return angle
        
    def do_trajectory_sequence(self,speed,trajectory):

        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
        
        max_roll_deg = 45.0
        max_pitch_deg = 45.0
        max_yaw_deg = 45.0
       
        tag_z_min = 4.0
        tag_z_max = 16.0
        
        wr_max_in_deg_per_sec = 10.0
        wy_max_in_deg_per_sec = 10.0
        
        roll_scale_factor = wr_max_in_deg_per_sec / max_roll_deg 
        yaw_scale_factor = wy_max_in_deg_per_sec / max_yaw_deg
        
        current_roll_deg = rpy_from_imu_to_global[0] * 180.0 / pi
        current_pitch_deg = 0.0
        current_yaw_deg = rpy_from_imu_to_global[2] * 180.0 / pi
        
        d_max_in_meters = 1.0
        d_min_in_meters = 0.5
        median_depth = ( d_max_in_meters + d_min_in_meters ) / 2.0 
        depth_scale_factor = ( d_max_in_meters - d_min_in_meters ) / ( tag_z_max - tag_z_min )
        
        have_first_tag_yaw = False
    
        # TODO: What rate to spin at?
        T =1 / 5.0
        rate = rospy.Rate(1.0/T)
 
        while (not rospy.is_shutdown()) and (len(trajectory)>0):
            
            current_traj_point = trajectory.pop(0)
            
            print 'Current waypoint is:' + str(current_traj_point)
            
            for k,v in current_traj_point.items():
                #print 'Tag information is: ' + str(v)
                
                if v.has_key('Orientation'):
                    
                    print '\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n'            
                    command_orientation = tf.transformations.euler_from_quaternion( v['Orientation'] )
                    command_position = v['Position']
                    
                    print 'Tag orientation: r: ' +    str(command_orientation[0]*180.0/pi)  + ', p: ' + str(command_orientation[1]*180.0/pi) + ', y: ' + str(command_orientation[2]*180.0/pi)
                    print 'Tag position: x: ' +    str(command_position[0])  + ', y: ' + str(command_position[1]) + ', z: ' + str(command_position[2])
                    
                    if not have_first_tag_yaw:
                        base_tag_yaw = command_orientation[2]*180.0/pi
                        tag_yaw_change_deg = 0.0
                        have_first_tag_yaw = True
                    else:
                        tag_yaw_change_deg = command_orientation[2]*180.0/pi - base_tag_yaw
                    
                    print 'Tag yaw change: ' + str(tag_yaw_change_deg)
                    
                    current_yaw_deg = rpy_from_imu_to_global[2] * 180.0 / pi +  tag_yaw_change_deg
                    current_roll_deg = command_orientation[1] * 180.0 / pi
                    #current_yaw_deg = current_yaw_deg + yaw_scale_factor * command_orientation[2]*180.0/pi
                    #current_yaw_deg = command_orientation[2]*180.0/pi
                    #current_depth_meters = median_depth + depth_scale_factor * command_orientation[1]*180.0/pi
                    current_depth_meters = depth_scale_factor * ( command_position[2] - tag_z_min ) + d_min_in_meters
                    
                    current_roll_deg = self.clamp180( current_roll_deg )
                    current_yaw_deg =  self.clamp180( current_yaw_deg )
                    
                    print 'Resulting command: r: ' + str(current_roll_deg) + '  y: ' + str(current_yaw_deg) +   '  d: ' + str(current_depth_meters)
                    
                    rotation_from_target_to_global = tf.transformations.quaternion_from_euler(current_roll_deg * pi/180.0, 
                                                                                              0.0, 
                                                                                              current_yaw_deg * pi/180.0)
                    
                    pose_and_vel = PoseStamped()
                    pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
                    pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
                    pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
                    pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
                    pose_and_vel.pose.position.x = speed
                    pose_and_vel.pose.position.y = 0.0
                    pose_and_vel.pose.position.z = min( max(current_depth_meters,d_min_in_meters), d_max_in_meters)
                    pose_and_vel.header.stamp = rospy.Time.now()
                    pose_and_vel.header.frame_id = '/latest_fix'
        
                    self.target_pose_pub.publish(pose_and_vel) 
            
            rate.sleep()

    def send_single_command(self,speed,target_angles_in_deg,target_depth):
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(target_angles_in_deg[0]*pi/180, 
                                                                                  target_angles_in_deg[1]*pi/180, 
                                                                                  target_angles_in_deg[2]*pi/180)
        
        pose_and_vel = PoseStamped()
        pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
        pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
        pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
        pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
        pose_and_vel.pose.position.x = speed
        pose_and_vel.pose.position.y = 0.0 # heave
        pose_and_vel.pose.position.z = target_depth 
        pose_and_vel.header.stamp = rospy.Time.now()
        pose_and_vel.header.frame_id = '/latest_fix'

        self.target_pose_pub.publish(pose_and_vel) 

    def park(self):
        starting_time = rospy.Time.now()
        dt_in_sec = rospy.Duration(4.0)
        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(0.0, 
                                                                                  0.0, 
                                                                                  rpy_from_imu_to_global[2])
        
        while (not rospy.is_shutdown()) and ((starting_time + dt_in_sec) > rospy.Time.now()):
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = 0.0
            pose_and_vel.pose.position.y = -0.8 # heave
            pose_and_vel.pose.position.z = self.current_depth 
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'
            
            self.target_pose_pub.publish(pose_and_vel) 

        rospy.loginfo( 'Exiting the autopilot run because a park action is finishing. See you next year!')
        rospy.signal_shutdown('Tag asked robot to park.')
        
    def flip_turn(self,dt_in_sec,speed,current_yaw_deg):
        #rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()

        T = 0.1
        wy = pi / dt_in_sec
        rate = rospy.Rate(1.0/T)
        depth = self.current_depth
        
        t = 0.0
        starting_time = rospy.Time.now()

        # Execute a 180 pitch maneuver
        while (not rospy.is_shutdown()) and ((starting_time + rospy.Duration(dt_in_sec)) > rospy.Time.now()):
            
            rotation_from_target_to_global = tf.transformations.quaternion_from_euler(0.0, 
                                                                                      -wy*t,
                                                                                      current_yaw_deg*pi/180.0) 
                                                                                      #rpy_from_imu_to_global[2])
        
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = speed
            pose_and_vel.pose.position.y = 0.0
            pose_and_vel.pose.position.z = depth
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel) 
            
            rate.sleep()
            t = t + T  
            
        t = 0.0
        starting_time = rospy.Time.now()

        # Execute a 180 roll maneuver
        while (not rospy.is_shutdown()) and ((starting_time + rospy.Duration(dt_in_sec)) > rospy.Time.now()):
            
            rotation_from_target_to_global = tf.transformations.quaternion_from_euler(wy*t, 
                                                                                      -pi, 
                                                                                      current_yaw_deg*pi/180.0) 
                                                                                      #rpy_from_imu_to_global[2])
        
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = speed
            pose_and_vel.pose.position.y = 0 
            pose_and_vel.pose.position.z = depth
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel) 
            
            rate.sleep()
            t = t + T  
               
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(0, 
                                                                                  0, 
                                                                                  current_yaw_deg*pi/180.0+pi) 
                                                                                  #rpy_from_imu_to_global[2]+pi)               
        pose_and_vel = PoseStamped()
        pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
        pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
        pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
        pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
        pose_and_vel.pose.position.x = speed
        pose_and_vel.pose.position.y = 0 
        pose_and_vel.pose.position.z = depth
        pose_and_vel.header.stamp = rospy.Time.now()
        pose_and_vel.header.frame_id = '/latest_fix'               
                 
            
            
        
