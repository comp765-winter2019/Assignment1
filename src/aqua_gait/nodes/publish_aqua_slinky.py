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

import math
import rospy
from std_msgs.msg import String
from aquacore.msg import PeriodicLegCommand
from aquacore.srv import SetGait


def wrapAngle(angle, angleRange = 360.0):
  return (angle - math.floor(angle/angleRange)*angleRange)


def slink():
    rospy.init_node('slinky')

    set_gait_cln = rospy.ServiceProxy('/aqua/set_gait', SetGait)
  
    rospy.loginfo('Setting gait to flexible-sine')
    set_gait_cln('flexible-sine')
    
    leg_pub = rospy.Publisher('/aqua/periodic_leg_command',PeriodicLegCommand,queue_size=10)
    rate = rospy.Rate(50)
    
    scale = 6.0
    
    offset_rate_radps = math.pi*scale
    spin_duration_sec = 8.0/scale

    plc = PeriodicLegCommand()
    plc.header.stamp = rospy.Time.now()
    plc.header.frame_id = '/aqua_base'
    for i in xrange(6):
      plc.amplitudes[i] = 0
      plc.frequencies[i] = 0
      plc.phase_offsets[i] = 0
      plc.leg_offsets[i] = 0
      plc.leg_velocities[i] = 0
    
    start_time = rospy.Time.now()
    prev_time = start_time
    while not rospy.is_shutdown():
      now = rospy.Time.now()
      t = (now - start_time).to_sec()
      dt = (now - prev_time).to_sec()
      
      if int(t/spin_duration_sec) % 2 == 0: # hind legs
        #plc.leg_offsets[2] = wrapAngle(plc.leg_offsets[2] + dt*offset_rate_radps, math.pi*2)
        #plc.leg_offsets[5] = wrapAngle(plc.leg_offsets[5] + dt*offset_rate_radps, math.pi*2)

        plc.leg_offsets[0] = 0
        plc.leg_velocities[0] = 0
        plc.leg_offsets[2] = 0
        plc.leg_velocities[2] = offset_rate_radps;
        plc.leg_offsets[3] = 0
        plc.leg_velocities[3] = 0
        plc.leg_offsets[5] = 0
        plc.leg_velocities[5] = offset_rate_radps;
      else: # fore legs
        #plc.leg_offsets[0] = wrapAngle(plc.leg_offsets[0] - dt*offset_rate_radps, math.pi*2)
        #plc.leg_offsets[3] = wrapAngle(plc.leg_offsets[3] - dt*offset_rate_radps, math.pi*2)
        
        plc.leg_offsets[0] = 0
        plc.leg_velocities[0] = -offset_rate_radps;
        plc.leg_offsets[2] = 0
        plc.leg_velocities[2] = 0
        plc.leg_offsets[3] = 0
        plc.leg_velocities[3] = -offset_rate_radps;
        plc.leg_offsets[5] = 0
        plc.leg_velocities[5] = 0
      leg_pub.publish(plc)
      
      prev_time = now
      
      rate.sleep()


if __name__ == '__main__':
  try:
    slink()
  except rospy.ROSInterruptException: pass
