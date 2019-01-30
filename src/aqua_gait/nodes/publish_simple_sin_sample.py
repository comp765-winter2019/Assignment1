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

## by JCGH, AX and DM
import rospy
from std_msgs.msg import String
from aquacore.msg import PeriodicLegCommand

def talker():
    leg_pub = rospy.Publisher('/aqua/periodic_leg_command',PeriodicLegCommand,queue_size=1)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(50)
    tick = 0
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        # cycle data being send every 1 second
        if (rospy.Time.now() - start_time).to_sec() > 1:
          tick = (tick + 1) % 30
          start_time = rospy.Time.now()

        plc = PeriodicLegCommand()
        plc.header.stamp = rospy.Time.now()
        plc.header.frame_id = '/aqua_base'
        
        non_zero_idx = int(tick)/5
        field_idx = tick - non_zero_idx*5
        if field_idx == 0:
          plc.leg_offsets[non_zero_idx] = tick
        elif field_idx == 1:
          plc.leg_velocities[non_zero_idx] = tick
        elif field_idx == 2:
          plc.amplitudes[non_zero_idx] = tick
        elif field_idx == 3:
          plc.frequencies[non_zero_idx] = tick
        elif field_idx == 4:
          plc.phase_offsets[non_zero_idx] = tick
        
        leg_pub.publish(plc)

        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
