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
import yaml
import pickle

from aquacore.msg import AutopilotModes
from autopilot_client import AutopilotClient  
import dynamic_reconfigure.client

if __name__ == '__main__':
    
    rospy.init_node('tag_trajectory_node')
    
    trajectory_filename = rospy.get_param('~trajectory_filename')
    
    depth_params = {}
    depth_params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_FIXED_DEPTH

    angle_params = {}
    angle_params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_LOCAL_THRUST

    ac = AutopilotClient(angle_params)
     
    client = dynamic_reconfigure.client.Client("/localAP", timeout=5 )
        
    knife_edge_params = { 'ROLL_P_GAIN' : 1.0, 
                          'PITCH_P_GAIN' : 2.0,
                          'YAW_P_GAIN' : -3.5,
                          'ROLL_D_GAIN' : 0.0,
                          'PITCH_D_GAIN' : 0.0,
                          'YAW_D_GAIN' : -1.5,
                          'KDEPTH' : -1.3,
                          'DEPTH_D_GAIN' : -1.5,
                          'DEPTH_D_FILTER_PERIOD' : 0.6 }
    

    client.update_configuration(knife_edge_params)
    ac.set_mode(depth_params)
    
    # Choose a trajectory file based on parameter and send it to the ap_client function
    speed = 0.8
    
#     stream = open(trajectory_filename, 'r')
#     trajectory = yaml.load(stream)
    trajectory = pickle.load( open( trajectory_filename, "rb" ) )
         
    try:            
         print 'Starting traj'
         ac.do_trajectory_sequence(speed,trajectory)
      
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
         print e
         
