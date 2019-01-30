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
roslib.load_manifest('scripted_trajectory_samples')
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
    
    forward_speed = 0.8
    loop_duration = 20.0

    print 'Vertical Loop'
    yaw_rate = 0.0
    pitch_rate = 360.0 / loop_duration
    roll_rate = 0.0 
    ac.smoothly_changing_forward_trajectory(loop_duration, forward_speed, roll_rate, pitch_rate, yaw_rate)

    print 'Sideways Looping Corkscrew'
    yaw_rate = 360.0 / loop_duration
    roll_rate = 20.0
    ac.smoothly_changing_forward_fixed_depth_trajectory(loop_duration, forward_speed, roll_rate, yaw_rate)

    print 'Sideways Heave Loop'
    heave_speed = 0.8
    roll_angle = 90.0
    pitch_angle = 0.0
    yaw_rate = 360.0 / loop_duration
    ac.smoothly_changing_heave_fixed_depth_trajectory(loop_duration, heave_speed, roll_angle, pitch_angle, yaw_rate)


