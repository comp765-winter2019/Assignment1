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

from std_srvs.srv import Empty as EmptySrv
from aquacore.srv import IsCalibrated
from aquacore.msg import AutopilotModes
from autopilot_client import AutopilotClient


if __name__ == '__main__':
    rospy.init_node('coverage_ap_controller')
    params = {}
    params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_FIXED_DEPTH
    #params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_LOCAL_THRUST
    ac = AutopilotClient(params)

    # check if is calibrated
    '''doesn't always work correctly in the water, and will always be manually
       calibrated, so leave this out for now -- arnold                        '''

    '''rospy.loginfo('Waiting for /aqua/calibrate and /aqua/is_calibrated...')
    rospy.wait_for_service('/aqua/calibrate')
    rospy.wait_for_service('/aqua/is_calibrated')
    call_calibrate = rospy.ServiceProxy('/aqua/calibrate', EmptySrv)
    call_is_calibrated = rospy.ServiceProxy('/aqua/is_calibrated', IsCalibrated)
    try:
      resp = call_is_calibrated()
      if not resp.value:
        rospy.loginfo('autocalibrating robot...')
        call_calibrate()
      else:
        rospy.loginfo('robot already calibrated :)')
    except rospy.ServiceException, e:
      rospy.logerr('Service call failed: %s' % e)'''

    dt_in_sec = float(rospy.get_param('~duration', 40.0))
    speed = float(rospy.get_param('~speed', 1.0))
    roll_in_deg = float(rospy.get_param('~roll_deg', 0.))
    pitch_in_deg = float(rospy.get_param('~pitch_deg', 0.))
    delta_yaw_in_deg = float(rospy.get_param('~delta_yaw_deg', 0.))

    try:
        rospy.loginfo('Aqua Coverage Controller - swim @ %.1f w RPdeltaY=%.1f, %.1f, %.1f for %.1f sec' % (speed, roll_in_deg, pitch_in_deg, delta_yaw_in_deg, dt_in_sec))
        rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
        curr_yaw_in_deg = rpy_from_imu_to_global[2]*180/pi
        target_depth = ac.current_depth
        target_angles = [roll_in_deg, pitch_in_deg, curr_yaw_in_deg+delta_yaw_in_deg]
        vx = speed
        vz = 0

        ac.do_straight_line(rospy.Duration(dt_in_sec), target_angles, target_depth, vx, vz)
       
        nsteps = 5
        delta_yaw = 180./nsteps
        vx = speed/2; vz = 0
        for i in range(nsteps):
            rospy.loginfo('Aqua Coverage Controller - Turn %.1f @ speed (step %d/%d)' % (delta_yaw, vx, i, nsteps))
            ac.do_relative_angle_change([0,0,delta_yaw], target_depth, vx, vz)
            
            rospy.loginfo('Aqua Coverage Controller - Swim Straight @ speed %.1f for %.1f sec (step %d/%d)' % (vx, 1, i, nsteps))
            rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
            curr_yaw_in_deg = rpy_from_imu_to_global[2]*180/pi
            target_depth = ac.current_depth
            target_angles = [roll_in_deg, pitch_in_deg, curr_yaw_in_deg+delta_yaw_in_deg]
            ac.do_straight_line(rospy.Duration(1), target_angles, target_depth, vx, vz)

        rospy.loginfo('Aqua Coverage Controller - swim @ %.1f w RPdeltaY=%.1f, %.1f, %.1f for %.1f sec' % (speed, roll_in_deg, pitch_in_deg, delta_yaw_in_deg, dt_in_sec))
        rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
        curr_yaw_in_deg = rpy_from_imu_to_global[2]*180/pi
        target_depth = ac.current_depth
        target_angles = [roll_in_deg, pitch_in_deg, curr_yaw_in_deg+delta_yaw_in_deg]
        vx = speed
        vz = 0

        ac.do_straight_line(rospy.Duration(dt_in_sec), target_angles, target_depth, vx, vz)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print e

