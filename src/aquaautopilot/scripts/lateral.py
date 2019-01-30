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
import tf
from math import pi

from aquacore.msg import AutopilotModes
from autopilot_client import AutopilotClient  


if __name__ == '__main__':
    
    rospy.init_node('lateral')
    
    dt_in_sec = rospy.Duration(float(rospy.get_param('~duration', 15)))
    
    params = {}
    params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_FIXED_DEPTH
    #params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_LOCAL_THRUST
    ac = AutopilotClient(params)

    roll_in_deg = 90
    pitch_in_deg = 0 

    try:            
             
        print 'Going laterally'
        rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
        target_depth = ac.current_depth
        target_angles = [roll_in_deg, pitch_in_deg, rpy_from_imu_to_global[2]*180/pi]
        vx = 0
        vz = -0.8

        ac.do_lateral_motion(dt_in_sec, target_angles, target_depth, vx, vz)
        
         
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print e

