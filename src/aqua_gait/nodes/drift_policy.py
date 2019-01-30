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


import sys
import time
import argparse
import rospy
import rospkg
import tf
from std_msgs.msg import String, Time
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from geometry_msgs.msg import PoseStamped
from aquacore.msg import StateMsg, Command, AutopilotModes
from aquacore.srv import SetAutopilotMode, SetAutopilotModeRequest, SetGait, SetGaitRequest
from aqua_rl.srv import ResetPolicy, ResetPolicyRequest, ResetPolicyResponse
from aqua_rl.srv import StepPolicy, StepPolicyRequest, StepPolicyResponse
from drift_common import *


class Policy:
  def __init__(self, dbfile, use_ap, sample_init_yaw_depth):
    self.alive = True
    self.sample_init_yaw_depth = sample_init_yaw_depth
    self.use_ap = use_ap
    self.live_mode = False
    self.start_time = None
    self.default_yaw_deg = 0.0
    self.policies = pickle.load(open(dbfile, "r"))
    
    self.max_sec_for_silent_dismiss_step_without_state_after_reset = 1.0
    
    for service in ['/aqua/set_gait', '/aqua/set_3Dauto_mode', '/aqua_rl/hack_policy_param_to_critic', '/AP_depth_filter/reset', '/hover_midoff_node/reset']:
      rospy.loginfo('Waiting for' + service)
      rospy.wait_for_service(service)
    rospy.loginfo('All necessary services found')
         
    self.set_gait_cln = rospy.ServiceProxy('/aqua/set_gait', SetGait)
    self.ap_mode_cln = rospy.ServiceProxy('/aqua/set_3Dauto_mode', SetAutopilotMode)
    self.reset_hover_midoff_cln = rospy.ServiceProxy('/hover_midoff_node/reset', Empty)
    self.reset_ap_cln = rospy.ServiceProxy('/aqua/reset_3D_autopilot_state', Empty)
    self.reset_depth_filter_cln = rospy.ServiceProxy('/AP_depth_filter/reset', Empty)
    
    self.reset_srv = rospy.Service('/aqua_rl/reset_policy', ResetPolicy, self.resetCallback)
    self.step_srv = rospy.Service('/aqua_rl/step_policy', StepPolicy, self.stepCallback)
    self.new_episode_sub = rospy.Subscriber('/aqua_rl/new_episode', Time, self.newEpisodeCallback, queue_size=10)
    self.run_live_sub = rospy.Subscriber('/aqua_rl/run_live', EmptyMsg, self.runLiveCallback, queue_size=10)
    self.state_sub = rospy.Subscriber('/aqua/state', StateMsg, self.stateCallback, queue_size=10)
    
    self.policy_param_cln = rospy.ServiceProxy('/aqua_rl/hack_policy_param_to_critic', ResetPolicy)
    
    self.cmd_pub = rospy.Publisher('/aqua/command', Command, queue_size=10)
    self.ap_pub = rospy.Publisher('/aqua/target_pose', PoseStamped, queue_size=10)
    
    rospy.logdebug('Setting gait to flexible-sine (since hover_midoff_node will translate)')
    self.set_gait_cln('flexible-sine')
    
    self.reset()
    
    rospy.loginfo('Policy started')
    
    
  def reset(self, param_str = '0'):
    self.start_time = None # not needed when not in live mode; otherwise set to current ROS time when run live is called
    #self.robot_state = None # commented so that robot can sample initial pose even if plant (e.g. sim) stops publishing after being reset, and before runlive/step
    self.live_mode = False
    try:
      self.params = pickle.loads(param_str)
    except pickle.UnpicklingError:
      self.params = self.policies[param_str]

    self.reset_hover_midoff_cln(EmptyRequest())
    
    self.default_yaw_deg = 0.0
    
    self.ap_cmd = PoseStamped()
    self.ap_cmd.header.frame_id = 'aqua'
    self.ap_cmd.header.stamp = time.time()
    self.ap_cmd.pose.position.x = 0
    self.ap_cmd.pose.position.y = 0
    self.ap_cmd.pose.position.z = 0
    self.ap_cmd_roll_deg = 0
    self.ap_cmd_yaw_deg = self.default_yaw_deg
    self.ap_cmd_pitch_deg = 0
    self.ap_cmd_depth = 0.0
    self.ap_cmd_max_yaw_rate_degps = -20.0 # TODO: might need to be tuned to work reasonably on real robot, in water
    quat = tf.transformations.quaternion_from_euler(self.ap_cmd_roll_deg*radian, self.ap_cmd_pitch_deg*radian, self.ap_cmd_yaw_deg*radian, 'sxyz')
    self.ap_cmd.pose.orientation.x = quat[0]
    self.ap_cmd.pose.orientation.y = quat[1]
    self.ap_cmd.pose.orientation.z = quat[2]
    self.ap_cmd.pose.orientation.w = quat[3]
    
    self.step_ap_cmd_latest_time = None
    
    if self.use_ap:
      rospy.logdebug('Resetting autopilot state')
      self.reset_ap_cln(EmptyRequest())
      
      rospy.logdebug('Setting autopilot mode to AutopilotModes.DEPTH')
      self.ap_mode_cln(SetAutopilotModeRequest(AutopilotModes.DEPTH))

      rospy.logdebug('Resetting autopilot depth filter')
      self.reset_depth_filter_cln(EmptyRequest())
      pass
    
    
  def step(self, state, dt):
    if self.use_ap:
      self.stepAPCmd(state, dt)
    else:
      self.stepBodyCmd(state, dt)
    
  def stepBodyCmd(self, state, dt):
    if state is None:
      if dt is None or dt > self.max_sec_for_silent_dismiss_step_without_state_after_reset:
        rospy.logerr('stepBodyCmd called with empty state, live_mode: %d' % self.live_mode)
      return
    elif dt is None:
      rospy.logerr('stepBodyCmd called with empty dt, live_mode: %d' % self.live_mode)
      return

    # Compute body command based on policy phase
    policy_phase = None
    cmd = Command()
    cmd.speed = 0
    cmd.heave = 0
    cmd.roll = 0
    cmd.pitch = 0
    cmd.yaw = 0
    if dt < self.params.A_duration:
      policy_phase = DriftPhase.A_SWIM_FORWARD
      cmd.speed = self.params.A_forward_rate
      
    elif dt < self.params.A_duration + self.params.B_duration:
      policy_phase = DriftPhase.B_ANTICIPATE_RIGHT
      cmd.speed = self.params.B_forward_rate
      cmd.yaw = self.params.B_yaw_rate
      
    elif dt < self.params.A_duration + self.params.B_duration + self.params.C_duration:
      policy_phase = DriftPhase.C_TURN_LEFT
      cmd.speed = self.params.C_forward_rate
      cmd.yaw = self.params.C_yaw_rate
      
    elif dt < self.params.A_duration + self.params.B_duration + self.params.C_duration + self.params.D_duration:
      policy_phase = DriftPhase.D_DRIFT
      
    else:
      policy_phase = DriftPhase.E_TERMINAL
      if dt < self.params.A_duration + self.params.B_duration + self.params.C_duration + self.params.D_duration + self.params.E_duration:
        cmd.speed = self.params.E_forward_rate

    # Output command
    self.cmd_pub.publish(cmd)
    rospy.logdebug('stepBodyCmd(dt=%.4f): %s - spd=%.2f yaw=%.2f' % (dt, DriftPhase.reverse_mapping[policy_phase], cmd.speed, cmd.yaw))
    
    
  def stepAPCmd(self, state, dt):
    if state is None:
      if dt is None or dt > self.max_sec_for_silent_dismiss_step_without_state_after_reset:
        rospy.logerr('stepAPCmd called with empty state, live_mode: %d' % self.live_mode)
      return
    elif dt is None:
      rospy.logerr('stepAPCmd called with empty dt, live_mode: %d' % self.live_mode)
      return

    # Compute body command based on policy phase
    policy_phase = None
    speed = 0
    yaw_rate = 0
    if dt < self.params.A_duration:
      policy_phase = DriftPhase.A_SWIM_FORWARD
      speed = self.params.A_forward_rate
      
    elif dt < self.params.A_duration + self.params.B_duration:
      policy_phase = DriftPhase.B_ANTICIPATE_RIGHT
      speed = self.params.B_forward_rate
      yaw_rate = self.params.B_yaw_rate
      
    elif dt < self.params.A_duration + self.params.B_duration + self.params.C_duration:
      policy_phase = DriftPhase.C_TURN_LEFT
      speed = self.params.C_forward_rate
      yaw_rate = self.params.C_yaw_rate
      
    elif dt < self.params.A_duration + self.params.B_duration + self.params.C_duration + self.params.D_duration:
      policy_phase = DriftPhase.D_DRIFT
      
      # Update the yaw angle to the current robot's angle, so that as soon as the robot exits from drift, it will move forward based on its current heading
      #
      # NOTE: this is still needed even though we have below 'curr_yaw_deg = self.default_yaw_deg - 180.0', since we want the robot to swim forward given its latest angle at the start of phase E
      self.ap_cmd_yaw_deg = self.robot_state.YawAngle
          
    else:
      policy_phase = DriftPhase.E_TERMINAL
      if dt < self.params.A_duration + self.params.B_duration + self.params.C_duration + self.params.D_duration + self.params.E_duration:
        speed = self.params.E_forward_rate

    # Step forward
    now = dt
    if self.step_ap_cmd_latest_time is not None:
      dt_fn_call = now - self.step_ap_cmd_latest_time
      self.ap_cmd.pose.position.z = self.ap_cmd_depth
      self.ap_cmd_yaw_deg = wrapAngle(self.ap_cmd_yaw_deg + dt_fn_call*self.ap_cmd_max_yaw_rate_degps*yaw_rate, 360.0)
    self.step_ap_cmd_latest_time = now
    
    curr_yaw_deg = self.ap_cmd_yaw_deg
    if policy_phase == DriftPhase.D_DRIFT: # force robot's yaw to be at -180 during drift, even though we are using self.ap_cmd_yaw_deg to store current robot yaw, in anticipation for E_TERMINAL
      curr_yaw_deg = self.default_yaw_deg - 180.0
    quat = tf.transformations.quaternion_from_euler(self.ap_cmd_roll_deg*radian, self.ap_cmd_pitch_deg*radian, curr_yaw_deg*radian, 'sxyz')
    self.ap_cmd.pose.orientation.x = quat[0]
    self.ap_cmd.pose.orientation.y = quat[1]
    self.ap_cmd.pose.orientation.z = quat[2]
    self.ap_cmd.pose.orientation.w = quat[3]

    # Output command
    self.ap_cmd.header.stamp = rospy.Time.now()
    self.ap_cmd.pose.position.x = speed
    self.ap_pub.publish(self.ap_cmd)
    rospy.loginfo('stepAPCmd(dt=%.4f): %s - spd=%.2f yaw=%.2f, yaw_rate=%.2f' % (dt, DriftPhase.reverse_mapping[policy_phase], self.ap_cmd.pose.position.x, curr_yaw_deg, yaw_rate))
    
    
  def newEpisodeCallback(self, msg):
    self.start_time = msg.data
    rospy.logdebug('Received new episode time')
    
    # Tell drift_critic about our current policy
    self.policy_param_cln(pickle.dumps(self.params))
    rospy.logdebug('Told critic about our current policy params')
    
    # Update default depth and yaw angle based on sensed robot state
    if self.sample_init_yaw_depth:
      if self.robot_state is None:
        rospy.logerr('Cannot sample default yaw and depth since did not receive any messages on /aqua/state')
      else:
        self.default_yaw_deg = self.robot_state.YawAngle
        self.ap_cmd_depth = self.robot_state.Depth

        self.ap_cmd_yaw_deg = self.default_yaw_deg
        quat = tf.transformations.quaternion_from_euler(self.ap_cmd_roll_deg*radian, self.ap_cmd_pitch_deg*radian, self.ap_cmd_yaw_deg*radian, 'sxyz')
        self.ap_cmd.pose.orientation.x = quat[0]
        self.ap_cmd.pose.orientation.y = quat[1]
        self.ap_cmd.pose.orientation.z = quat[2]
        self.ap_cmd.pose.orientation.w = quat[3]
        
        rospy.loginfo('Setting default yaw=%.2f deg and depth=%.2f m at start of episode' % (self.default_yaw_deg, self.ap_cmd_depth))

    
  def runLiveCallback(self, msg):
    self.live_mode = True
    rospy.logdebug('Entering live mode')
    
    
  def stateCallback(self, msg):
    self.robot_state = msg
    if self.live_mode and self.start_time is not None:
      self.step(msg, (rospy.Time.now() - self.start_time).to_sec())


  def resetCallback(self, req):
    self.reset(req.policy_param)
    return ResetPolicyResponse()


  def stepCallback(self, req):
    if not self.live_mode and self.start_time is not None:
      self.step(self.robot_state, (req.desired_time - self.start_time).to_sec())
    else:
      if self.live_mode:
        rospy.logerr('stepCallback called in live mode!')
      if self.start_time is None:
        rospy.logerr('stepCallback called without having received start_time!')
    return StepPolicyResponse()
    
  
  def spin(self):
    rospy.spin()


if __name__ == '__main__':
  rospy.init_node('drift_policy')

  rospack = rospkg.RosPack()
  parser = argparse.ArgumentParser()
  parser.add_argument('-d', '--dbfile', type=str, help='path to database file', required=False, default='%s/nodes/drift_policies.db' % (rospack.get_path('aqua_gait')))
  parser.add_argument('-c', '--createdb', help='re-generate drift policy parameter database from using data from drift_common.py', required=False, action='store_true')
  parser.add_argument('-a', '--autopilot', help='use autopilot (if false, then send raw body commands == NO ANGLE AUTOPILOT!)', required=False, action='store_true')
  parser.add_argument('-s', '--sample_init_yaw_depth', help='sample initial yaw and depth of robot at start of episode (at reception of /aqua_rl/start_episode)', required=False, action='store_true')
  args = parser.parse_args(args=rospy.myargv(argv=sys.argv[1:]))
  
  if args.createdb:
    genPolicyParamDB(args.dbfile)
  else:
    try:
      dp = Policy(args.dbfile, args.autopilot, args.sample_init_yaw_depth)
      dp.spin()
    except rospy.ROSInterruptException:
      pass
