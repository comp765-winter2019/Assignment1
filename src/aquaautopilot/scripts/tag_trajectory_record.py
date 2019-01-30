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
import cPickle
from std_msgs.msg import String
from ar_recog.msg import Tag
from ar_recog.msg import Tags
from visualization_msgs.msg import Marker
import tf
from math import pi
import yaml

import sys

from aquacore.msg import AutopilotModes
from autopilot_client import AutopilotClient  
import dynamic_reconfigure.client


class TrajectoryRecPlay():
  tag_sub = []
  content = []
  trajectory_filename = "trajectory.p"


  def __init__(self):
    self.trajectory_filename = rospy.get_param('~trajectory_filename')
    self.tag_sub = rospy.Subscriber('/ar_compute_pose/visualization_marker', Marker, self.processTags)


  def processTags(self, msg):
    if int(msg.text) == 5:
        print msg.pose.position.x;
        info = {}
        value = []
        value.append(msg.pose.position.x)
        value.append(msg.pose.position.y)
        value.append(msg.pose.position.z)
        info['Position'] = value
        value = []
        value.append(msg.pose.orientation.x)
        value.append(msg.pose.orientation.y)
        value.append(msg.pose.orientation.z)
        value.append(msg.pose.orientation.w)
        info['Orientation'] = value
        info['Frame No.'] = msg.id
        self.content.append(info)
        
    elif int(msg.text) == 0:
      print self.content
      self.tag_sub.unregister();
      cPickle.dump(self.content, open(self.trajectory_filename,"wb"),-1)
      print("Wrote trajectory to file: %s" %(self.trajectory_filename)) 
      rospy.signal_shutdown("I saw tag 3, shutting down.")
    
if __name__ == '__main__':
    rospy.init_node('trajectory_record_play')
    recnplay = TrajectoryRecPlay()
    rospy.spin()
