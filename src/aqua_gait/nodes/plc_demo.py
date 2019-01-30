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

import rospy, math
from std_msgs.msg import String
from aquacore.msg import PeriodicLegCommand

T = 2.0
offsets = [-0.7854,0,+0.7854]
current_offset = 0
actuated_legs = [5]

def talker():
    global T
    global offsets
    global current_offset
    global actuated_legs
    leg_pub = rospy.Publisher('/aqua/periodic_leg_command',PeriodicLegCommand,queue_size=1)
    rospy.init_node('talker', anonymous=True)

    rospy.loginfo('plc_demo:: waiting for /aqua/calibrate...')
    rospy.wait_for_service('/aqua/calibrate')

    r = rospy.Rate(200)
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        plc = PeriodicLegCommand()
        plc.header.frame_id = '/aqua_base'
        plc.header.stamp = rospy.Time.now()
        for l in xrange(6):
           if l not in actuated_legs:
                plc.leg_offsets[l] = math.pi

        for l in actuated_legs:
            plc.amplitudes[l] = 0.3491
            plc.frequencies[l] = 2.5
            plc.leg_velocities[l] = 0.0
            plc.leg_offsets[l] = offsets[current_offset]
            plc.phase_offsets[l] = 0.0

        # cycle data being send every T second
        if (rospy.Time.now() - start_time).to_sec() > T:
            rospy.loginfo("plc_demo:: updating leg offset to %f"%(offsets[current_offset]))
            start_time = rospy.Time.now()
            current_offset = (current_offset+1)%len(offsets)

        leg_pub.publish(plc)
        r.sleep()

if __name__ == '__main__':

    try:
        talker()
    except rospy.ROSInterruptException: pass
