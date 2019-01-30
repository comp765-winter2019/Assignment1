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
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from aquacore.msg import PeriodicLegCommand
from aquacore.srv import SetAutopilotMode, SetBool, SetBoolResponse


class AquaMarshall:
  def __init__(self):
    rospy.init_node('aqua_marshall')
    
    self.FSM_RL = False # None == no pass-through, False == AP, True == RL
    self.joy_start_btn_state = False
    self.joy_select_btn_state = False
    self.joy_A_btn_state = False
    self.gimped_AP = False
    
    rospy.loginfo('aqua_marshall waiting for /aqua/set_3Dauto_mode...')
    rospy.wait_for_service('/aqua/set_3Dauto_mode')
    self.set_ap_mode_cln = rospy.ServiceProxy('/aqua/set_3Dauto_mode', SetAutopilotMode)

    rospy.loginfo('aqua_marshall waiting for /aqua/reset_3D_autopilot_state...')
    rospy.wait_for_service('/aqua/reset_3D_autopilot_state')
    self.reset_ap_cln = rospy.ServiceProxy('/aqua/reset_3D_autopilot_state', Empty)
    
    rospy.loginfo('aqua_marshall waiting for /aqua_rl/trigger_start...')
    rospy.wait_for_service('/aqua_rl/trigger_start')
    self.trigger_start_cln = rospy.ServiceProxy('/aqua_rl/trigger_start', Empty)
    
    self.select_rl_svr = rospy.Service('/aqua_marshall/select_rl', SetBool, self.handleSelectRL)
    self.plc_in_ap_sub = rospy.Subscriber('/sandbox/AP/periodic_leg_command', PeriodicLegCommand, self.handleAPPLC)
    self.plc_in_rl_sub = rospy.Subscriber('/sandbox/RL/periodic_leg_command', PeriodicLegCommand, self.handleRLPLC)
    self.joy_sub = rospy.Subscriber('/joy', Joy, self.handleJoy)
    self.plc_out_pub = rospy.Publisher('/aqua/periodic_leg_command',PeriodicLegCommand,queue_size=10)
    self.fsm_pub = rospy.Publisher('/aqua_marshall/rl_mode',Bool,queue_size=10, latch=True)
    rate = rospy.Rate(50)
    
    self.activateAP()
    rospy.loginfo('MARSHALL > AP')
    self.fsm_pub.publish(Bool(self.FSM_RL))

    
  def activateAP(self):
    self.reset_ap_cln()
    self.set_ap_mode_cln(4)
    

  def deactivateAP(self):
    self.reset_ap_cln()
    self.set_ap_mode_cln(0)
    self.reset_ap_cln()
    
    # Zero-center all flippers
    msg = PeriodicLegCommand()
    self.plc_out_pub.publish(msg)
    rospy.sleep(2.0)
    
    self.trigger_start_cln()
  
  
  def handleAPPLC(self, msg_):
    if self.FSM_RL is not None and not self.FSM_RL:
      msg = msg_
      if self.gimped_AP:
        msg.amplitudes = list(msg.amplitudes)
        msg.frequencies = list(msg.frequencies)
        msg.phase_offsets = list(msg.phase_offsets)
        msg.leg_offsets = list(msg.leg_offsets)
        msg.leg_velocities = list(msg.leg_velocities)
      
        # front-left
        msg.amplitudes[0] = 0
        msg.frequencies[0] = 0
        msg.phase_offsets[0] = 0
        msg.leg_offsets[0] = 0
        msg.leg_velocities[0] = 0
        
        # mid-left
        msg.amplitudes[1] = 0
        msg.frequencies[1] = 0
        msg.phase_offsets[1] = 0
        msg.leg_offsets[1] = 0
        msg.leg_velocities[1] = 0
        
        # front-right
        msg.amplitudes[3] = 0
        msg.frequencies[3] = 0
        msg.phase_offsets[3] = 0
        msg.leg_offsets[3] = 0
        msg.leg_velocities[3] = 0
        
        # mid-right
        msg.amplitudes[4] = 0
        msg.frequencies[4] = 0
        msg.phase_offsets[4] = 0
        msg.leg_offsets[4] = 0
        msg.leg_velocities[4] = 0
        
      self.plc_out_pub.publish(msg)


  def handleRLPLC(self, msg):
    if self.FSM_RL is not None and self.FSM_RL:
      self.plc_out_pub.publish(msg)
  
  
  def handleJoy(self, msg):
    if len(msg.axes) == 6 and len(msg.buttons) == 12: # assume logitech mapping
      curr_A_btn_state = msg.buttons[1]
      if self.joy_A_btn_state and not curr_A_btn_state: # upon release
        self.gimped_AP = not self.gimped_AP      
      self.joy_A_btn_state = curr_A_btn_state
      
      curr_start_btn_state = msg.buttons[9]
      
      if self.joy_start_btn_state and not curr_start_btn_state: # upon release
        self.selectRL(not self.FSM_RL)
      
      self.joy_start_btn_state = curr_start_btn_state
      
      curr_select_btn_state = msg.buttons[8]
      
      if self.joy_select_btn_state and not curr_select_btn_state: # upon release
        self.trigger_start_cln()
      
      self.joy_select_btn_state = curr_select_btn_state
    else:
      rospy.logwarn('Ignoring joy msg with %d axes and %d buttons' % (len(msg.axes), len(msg.buttons)))

      
  def handleSelectRL(self, req):
    self.selectRL(req.value)
    return SetBoolResponse()


  def selectRL(self, enableRL):
    if enableRL != self.FSM_RL:
      self.FSM_RL = None
      if enableRL:
        self.deactivateAP()
        rospy.loginfo('MARSHALL > RL')
      else:
        self.activateAP()
        rospy.loginfo('MARSHALL > AP')
      self.FSM_RL = enableRL
      

  def spin(self):
    rospy.spin()



if __name__ == '__main__':
  try:
    node = AquaMarshall()
    node.spin()
  except rospy.ROSInterruptException: pass
