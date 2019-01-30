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

import rospy
import math
import threading
from std_msgs.msg import Bool, Float32
from std_srvs.srv import *
from aquacore.msg import StateMsg
from dynamic_reconfigure.server import Server
from aquadepth.cfg import DepthFilterConfig
from aquacore.srv import *


class DepthFilter:
  def __init__(self):
    rospy.init_node('depth_filter')
    self.min_window_entries = rospy.get_param('~min_window_entries', 4)
    self.window_size_sec = rospy.get_param('~window_size_sec', 2.0)
    self.cutoff_sigma_multiplier = rospy.get_param('~cutoff_sigma_multiplier', 3)
    self.N_sigma_cutoff_m = rospy.get_param('~N_sigma_cutoff_m', 0.15)
    self.window = []
    self.constant_depth_off_mutex = threading.Lock()
    self.prev_constant_depth = None
    self.constant_depth = False
    self.has_reached_constant_depth = False
    
    self.filtered_depth_pub = rospy.Publisher('/aqua/filtered_depth', Float32)
    self.constant_depth_pub = rospy.Publisher('/aqua/constant_depth_flag', Bool)
    self.constant_depth_blocking_svc = rospy.Service('/aqua/wait_till_constant_depth', Empty, self.waitTillConstantDepth)
    self.constant_depth_query_svc = rospy.Service('/aqua/has_reached_constant_depth', EmptyBool, self.hasReachedConstantDepth)
    self.state_sub = rospy.Subscriber('/aqua/state', StateMsg, self.stateCB)
    self.dyncfg_server = Server(DepthFilterConfig, self.dyncfgCB)
    
  def waitTillConstantDepth(self, req):
    self.constant_depth_off_mutex.acquire()
    self.constant_depth_off_mutex.release()
    return EmptyResponse()

  def hasReachedConstantDepth(self, req):
    result = EmptyBoolResponse(self.has_reached_constant_depth)
    self.has_reached_constant_depth = False
    return result

  def dyncfgCB(self, config, level):
    self.min_window_entries = config.min_window_entries
    self.window_size_sec = config.window_size_sec
    self.cutoff_sigma_multiplier = config.cutoff_sigma_multiplier
    self.N_sigma_cutoff_m = config.N_sigma_cutoff_m
    return config

  def stateCB(self, data):
    curr_entry = (data.header.stamp.to_sec(), data.Depth)
    
    # Remove outdated entries from sliding window
    if len(self.window) > 0:
      outdated_i = len(self.window)
      for i in xrange(len(self.window)):
        if curr_entry[0] - self.window[i][0] <= self.window_size_sec: # stop loop when found entry within sliding window
          outdated_i = i
          break
      del self.window[:outdated_i]
    
    # Compute mean and standard deviation of depth values
    self.window.append(curr_entry)
    num_entries = len(self.window)
    if num_entries >= self.min_window_entries:
      sum = 0.0
      sum_sqrd = 0.0
      for v in self.window:
        sum += v[1]
        sum_sqrd += v[1]*v[1]
      mean_depth = sum/num_entries
      stdev_depth = 0
      try:
        stdev_depth = math.sqrt(sum_sqrd/num_entries - mean_depth*mean_depth)
      except ValueError:
        mean_depth = 0
        pass # deal with math domain errors resulting from 1e-17 - 1e-10; arises when robot is not calibrated
      self.constant_depth = stdev_depth * self.cutoff_sigma_multiplier < self.N_sigma_cutoff_m
      #rospy.loginfo('n: %d, u: %.4f, s: %.4f, c: %d' % (num_entries, mean_depth, stdev_depth, self.constant_depth))
      
      # Publish mean depth (a.k.a. filtered depth)
      self.filtered_depth_pub.publish(Float32(mean_depth))
      
      # Publish flag indicating whether depth is constant or not
      self.constant_depth_pub.publish(Bool(self.constant_depth))
      
      # Update constant-depth-OFF mutex
      if self.prev_constant_depth is None:
        if not self.constant_depth:
          self.constant_depth_off_mutex.acquire()
      elif self.constant_depth and not self.prev_constant_depth:
        self.constant_depth_off_mutex.release()
      elif not self.constant_depth and self.prev_constant_depth:
        self.constant_depth_off_mutex.acquire()
      self.prev_constant_depth = self.constant_depth
      if self.constant_depth:
        self.has_reached_constant_depth = True

if __name__ == '__main__':
  try:
    filter = DepthFilter()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
