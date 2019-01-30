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

import roslib
roslib.load_manifest('aquaautopilot')
import rospy
from std_msgs.msg import String
import tf
from math import pi

from aquacore.msg import AutopilotModes
from autopilot_client import AutopilotClient  
import dynamic_reconfigure.client

if __name__ == '__main__':
    
    rospy.init_node('choreography_node')
    depth_params = {}
    depth_params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_FIXED_DEPTH

    angle_params = {}
    angle_params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_LOCAL_THRUST

    ac = AutopilotClient(angle_params)
     
    client = dynamic_reconfigure.client.Client("/localAP", timeout=5 )
    
    roll_in_deg = 90
    pitch_in_deg = 0
    
    knife_edge_params = { 'ROLL_P_GAIN' : 1.0, 
                          'PITCH_P_GAIN' : 2.0,
                          'YAW_P_GAIN' : -3.5,
                          'ROLL_D_GAIN' : 0.0,
                          'PITCH_D_GAIN' : 0.0,
                          'YAW_D_GAIN' : -1.5,
                          'KDEPTH' : -1.3,
                          'DEPTH_D_GAIN' : -1.5,
                          'DEPTH_D_FILTER_PERIOD' : 0.6 }

    snowplow_params = { 'ROLL_P_GAIN' : 0.55,
                          'PITCH_P_GAIN' : 1.5,
                          'YAW_P_GAIN' : -2.5,
                          'ROLL_D_GAIN' : 0.0,
                          'PITCH_D_GAIN' : 0.0,
                          'YAW_D_GAIN' : 0.0,
                          'KDEPTH' : -1.2,
                          'DEPTH_D_GAIN' : -0.8,
                          'DEPTH_D_FILTER_PERIOD' : 0.6 }

    client.update_configuration(knife_edge_params)
    target_depth = ac.current_depth
    rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
    
#    try:            
#         print 'SommerSide'
#         ac.set_mode(depth_params)
#         target_angles = [0, 0, rpy_from_imu_to_global[2]*180/pi]
#         vx = 0.9
#         vz = 0.0
#         dt_in_sec = rospy.Duration(15.0)
# 
#         ac.do_sideways_somersault(dt_in_sec, vx, vz, target_depth, -24.0)
#      
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#         print e
    
    try:            
         print 'SommerCork'
         ac.set_mode(depth_params)
         rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
         target_angles = [0, 0, rpy_from_imu_to_global[2]*180/pi]
         vx = 0.8
         vz = 0.0
         dt_in_sec = rospy.Duration(15.0)
 
         ac.do_rolling_sideways_somersault(dt_in_sec, vx, vz, target_depth, 24.0)
      
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
         print e

#    try:            
#         print 'SommerSide'
#         ac.set_mode(depth_params)
#         rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
#         target_angles = [0, 0, rpy_from_imu_to_global[2]*180/pi]
#         vx = 0.9
#         vz = 0.0
#         dt_in_sec = rospy.Duration(20.0)
# 
#         ac.do_sideways_somersault(dt_in_sec, vx, vz, target_depth, 18.0)
#      
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#         print e

#    try:            
#         print 'SommerVert'
#         ac.set_mode(angle_params)
#         rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
#         target_angles = [0, 0, rpy_from_imu_to_global[2]*180/pi]
#         vx = 0.8
#         vz = 0.0
#         dt_in_sec = rospy.Duration(20.0)
# 
#         ac.do_vertical_somersault(dt_in_sec, vx, vz, -18.0)
#      
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#         print e  
#         
#    try:            
#         print 'SommerCork'
#         ac.set_mode(depth_params)
#         rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
#         target_angles = [0, 0, rpy_from_imu_to_global[2]*180/pi]
#         vx = 0.8
#         vz = 0.0
#         dt_in_sec = rospy.Duration(20.0)
# 
#         ac.do_rolling_sideways_somersault(dt_in_sec, vx, vz, target_depth, 18.0)
#      
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#         print e         

#    try:            
#         print 'SommerSide'
#         ac.set_mode(depth_params)
#         rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
#         target_angles = [0, 0, rpy_from_imu_to_global[2]*180/pi]
#         vx = 0.9
#         vz = 0.0
#         dt_in_sec = rospy.Duration(30.0)
# 
#         ac.do_sideways_somersault(dt_in_sec, vx, vz, target_depth, -12.0)
#      
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#         print e
    
    try:            
         print 'Sommer45'
         ac.set_mode(angle_params)
         rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
         target_angles = [0, 0, rpy_from_imu_to_global[2]*180/pi]
         vx = 0.8
         vz = 0.0
         dt_in_sec = rospy.Duration(20.0)
 
         ac.do_45_somersault(dt_in_sec, vx, vz, pi/4.0, -18.0)
      
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
         print e  

#    try:            
#         print 'SommerSide'
#         ac.set_mode(depth_params)
#         rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
#         target_angles = [0, 0, rpy_from_imu_to_global[2]*180/pi]
#         vx = 0.9
#         vz = 0.0
#         dt_in_sec = rospy.Duration(20.0)
# 
#         ac.do_sideways_somersault(dt_in_sec, vx, vz, target_depth, -18.0)
#      
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#         print e
#    
    try:            
         print 'Sommer45'
         ac.set_mode(angle_params)
         rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
         target_angles = [0, 0, rpy_from_imu_to_global[2]*180/pi]
         vx = 0.8
         vz = 0.0
         dt_in_sec = rospy.Duration(20.0)
 
         ac.do_45_somersault(dt_in_sec, vx, vz, -pi/4.0, -18.0)
      
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
         print e  

#    try:            
#         print 'SommerCork'
#         ac.set_mode(depth_params)
#         rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
#         target_angles = [0, 0, rpy_from_imu_to_global[2]*180/pi]
#         vx = 0.9
#         vz = 0.0
#         dt_in_sec = rospy.Duration(20.0)
# 
#         ac.do_rolling_sideways_somersault(dt_in_sec, vx, vz, target_depth, -18.0)
#      
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#         print e
#                        
#    try:            
#         print 'SommerCork'
#         ac.set_mode(depth_params)
#         rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
#         target_angles = [0, 0, rpy_from_imu_to_global[2]*180/pi]
#         vx = 0.8
#         vz = 0.0
#         dt_in_sec = rospy.Duration(20.0)
# 
#         ac.do_rolling_sideways_somersault(dt_in_sec, vx, vz, target_depth, 18.0)
#      
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#         print e
#         
#    try:            
#         print 'SommerVert'
#         ac.set_mode(angle_params)
#         rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
#         target_angles = [0, 0, rpy_from_imu_to_global[2]*180/pi]
#         vx = 0.8
#         vz = 0.0
#         dt_in_sec = rospy.Duration(20.0)
# 
#         ac.do_vertical_somersault(dt_in_sec, vx, vz, -18.0)
#      
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#         print e           
#                        
#         

    try:            
         print 'SommerCork'
         ac.set_mode(depth_params)
         rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
         target_angles = [0, 0, rpy_from_imu_to_global[2]*180/pi]
         vx = 0.8
         vz = 0.0
         dt_in_sec = rospy.Duration(15.0)
 
         ac.do_rolling_sideways_somersault(dt_in_sec, vx, vz, target_depth, -24.0)
      
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
         print e


