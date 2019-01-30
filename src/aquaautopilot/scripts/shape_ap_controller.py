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
    rospy.init_node('shape_ap_controller')
    params = {}
    params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_FIXED_DEPTH
    #params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_LOCAL_THRUST
    ac = AutopilotClient(params)

    # check if is calibrated, and call calibrate otherwise
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
    edges = int(float(rospy.get_param('~edges', 1)))
    if edges < 1:
     edges = 1
    rospy.loginfo('AP Shape Controller - swim %d-side shape @ %.1f w RP=%.1f, %.1f for %.1f sec' % (edges, speed, roll_in_deg, pitch_in_deg, dt_in_sec))

    try:
        rpy_from_imu_to_global = ac.get_rpy_of_imu_in_global()
        init_yaw_in_deg = rpy_from_imu_to_global[2]*180/pi
        target_depth = ac.current_depth
        vx = speed
        vz = 0

        dt = rospy.Duration(dt_in_sec/edges)
        delta_yaw_in_deg = 360.0/edges
        for i in xrange(edges):
          rospy.loginfo('step %d/%d: dyaw=%.1f' % (i+1, edges, delta_yaw_in_deg*i))
          target_angles = [roll_in_deg, pitch_in_deg, init_yaw_in_deg+delta_yaw_in_deg*i]
          ac.do_straight_line(dt, target_angles, target_depth, vx, vz)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print e

