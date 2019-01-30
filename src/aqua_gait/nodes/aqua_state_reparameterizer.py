#! /usr/bin/env python
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
from std_msgs.msg import String, Time, Empty
from aquacore.msg import StateMsg
from sensor_msgs.msg import Joy
from aqua_gait.srv import UpdateTargetRPYD, UpdateTargetRPYDResponse


def capDeltaDeg(val):
  while val <= -180.0:
    val += 360.0
  while val > 180.0:
    val -= 360.0
  return val

class Reparameterizer:
  def __init__(self):
    self.update_mode = True
    self.depth_bias_m = 0.0
    self.yaw_bias_deg = 0.0
    self.joy_yaw = 0.0
    self.joy_depth = 0.0
    self.joy_lx = 0.0
    self.joy_ry = 0.0
    self.last_joy_time = None
    
    self.latest_aqua_state = None
    
    self.new_state_pub = rospy.Publisher('/aqua/state_reparameterized', StateMsg, queue_size=1)
    self.debug_pub = rospy.Publisher('~state', String, queue_size=1)
    
    self.update_target_rpyd_srv = rospy.Service('~update_target_rypd', UpdateTargetRPYD, self.handleUpdateTargetRPYD)
      
    self.joy_yaw_gain = rospy.get_param('~joy_yaw_gain', 5.0)
    self.joy_depth_gain = rospy.get_param('~joy_depth_gain', 0.25)
    self.target_roll_deg = rospy.get_param('~target_roll_deg', 0.0)
    self.target_pitch_deg = rospy.get_param('~target_pitch_deg', 0.0)
    self.target_yaw_deg = rospy.get_param('~target_yaw_deg', 0.0)
    self.target_depth_m = rospy.get_param('~target_depth_m', 5.0)
    
    #self.joy_sub = rospy.Subscriber('/joy', Joy, self.handleJoy) # AX: disabled for 2016/08 version
    self.state_sub = rospy.Subscriber('/aqua/state', StateMsg, self.stateCallback, queue_size=1)
    #self.new_episode_sub = rospy.Subscriber('/aqua_rl/new_episode', Time, self.newEpisodeCallback, queue_size=1) # AX: disabled for 2016/08 version
    #self.terminate_sub = rospy.Subscriber('/aqua_rl/terminate_episode', String, self.terminateEpisodeCallback, queue_size=1) # AX: disabled for 2016/08 version
    self.new_episode_sub = rospy.Subscriber('/aqua_rl/trigger_start', Empty, self.newEpisodeCallback, queue_size=1)
    self.terminate_sub = rospy.Subscriber('/aqua_rl/trigger_stop', Empty, self.terminateEpisodeCallback, queue_size=1)
      
    rospy.loginfo('Reparameterizer started, target RPYD:\t%.2f deg\t%.2f deg\t%.2f deg\t%.2f m' % (self.target_roll_deg, self.target_pitch_deg, self.target_yaw_deg, self.target_depth_m))

  

  def handleJoy(self, msg):
    if len(msg.axes) == 6 and len(msg.buttons) == 12: # assume logitech mapping
      
      curr_r2_btn_state = msg.buttons[7]
        
      if curr_r2_btn_state:
        self.joy_lx = msg.axes[0]
        self.joy_ry = msg.axes[3]
      else:
        self.joy_lx = 0.0
        self.joy_ry = 0.0
      
    else:
      rospy.logwarn('Ignoring joy msg with %d axes and %d buttons' % (len(msg.axes), len(msg.buttons)))


  def newEpisodeCallback(self, msg):
    # This sleep is needed because the simulator resets its state based on the same message.
    # If we start updating here right away, our biases will be computed based on the pre-reset simulator state and bad things will occur.
    rospy.sleep(0.02)
    self.joy_yaw = 0.0
    self.joy_depth = 0.0
    self.update_mode = False
    self.publishDebugState()

  def terminateEpisodeCallback(self, msg):
    self.update_mode = True
    self.publishDebugState()

    
  def stateCallback(self, msg):
    self.latest_aqua_state = msg
    
    # Update biases
    if self.update_mode:
      self.depth_bias_m = self.target_depth_m - msg.Depth
      self.yaw_bias_deg = self.target_yaw_deg - msg.YawAngle # TODO: confirm that this is in degrees
    
    # Publish reparameterized message
    new_msg = msg
    new_msg.Depth += self.depth_bias_m
    new_msg.YawAngle += self.yaw_bias_deg
    
    if not self.update_mode:
    
      now = rospy.Time.now()
            
      if not self.last_joy_time == None:
          
        dt = (now - self.last_joy_time).to_sec() 
        self.joy_yaw  += dt * self.joy_yaw_gain * self.joy_lx
        self.joy_depth  -= dt * self.joy_depth_gain * self.joy_ry
        
      self.last_joy_time = now
      
      new_msg.YawAngle += self.joy_yaw
      new_msg.Depth += self.joy_depth
    
    new_msg.YawAngle = capDeltaDeg(new_msg.YawAngle)
    self.new_state_pub.publish(new_msg)
    
    self.publishDebugState()


  def publishDebugState(self):
    delta_roll_deg = 999 # Naive way of warning that /aqua/state had not been received
    delta_pitch_deg = 999
    if self.latest_aqua_state is not None:
      delta_roll_deg = capDeltaDeg(self.target_roll_deg - self.latest_aqua_state.RollAngle)
      delta_pitch_deg = capDeltaDeg(self.target_pitch_deg - self.latest_aqua_state.PitchAngle)
    
    if self.update_mode:
      state = '~ '
    else:
      state = '. '
    state += 'droll=%3d''\tdpitch=%3d''\tbyaw=%.2f''\tbdepth=%.2fm' % (delta_roll_deg, delta_pitch_deg, self.yaw_bias_deg, self.depth_bias_m)
    self.debug_pub.publish(String(state))
    
  
  def handleUpdateTargetRPYD(self, req):
    self.target_roll_deg = req.target_roll_deg
    self.target_pitch_deg = req.target_pitch_deg
    self.target_yaw_deg = req.target_yaw_deg
    self.target_depth_m = req.target_depth_m
    rospy.loginfo('New target RPYD:\t%.2f deg\t%.2f deg\t%.2f deg\t%.2f m' % (self.target_roll_deg, self.target_pitch_deg, self.target_yaw_deg, self.target_depth_m))
    return UpdateTargetRPYDResponse()
  
  
  def spin(self):
    rospy.spin()


if __name__ == '__main__':
  rospy.init_node('aqua_state_reparameterizer')

  try:
    hal = Reparameterizer()
    hal.spin()
  except rospy.ROSInterruptException:
    pass
