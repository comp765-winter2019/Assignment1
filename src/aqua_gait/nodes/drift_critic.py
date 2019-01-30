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
import math
import argparse
import rospy
import rospkg
from std_msgs.msg import String, Time
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from aquacore.msg import StateMsg
from aqua_rl.msg import Reward
from aqua_rl.srv import ResetPolicy, ResetPolicyRequest, ResetPolicyResponse
from drift_common import *
import tf.transformations


class Critic:
  def __init__(self, dbfile, max_episode_time):
    self.max_episode_time = max_episode_time
    self.episode_alive = False
    self.start_time = None
    self.params = None
    self.policies = pickle.load(open(dbfile, "r"))

    self.alignment_reward_amplifier = 5.0/math.pow(30.0/180*math.pi, 2)

    self.policy_param_srv = rospy.Service('/aqua_rl/hack_policy_param_to_critic', ResetPolicy, self.hackCallback)
    
    self.reset_srv = rospy.Service('/aqua_rl/reset_critic', Empty, self.resetCallback)
    self.new_episode_sub = rospy.Subscriber('/aqua_rl/new_episode', Time, self.newEpisodeCallback, queue_size=10)
    self.state_sub = rospy.Subscriber('/aqua/state', StateMsg, self.stateCallback, queue_size=10)
    self.terminate_pub = rospy.Publisher('/aqua_rl/terminate_episode', String, queue_size=10)
    self.reward_pub = rospy.Publisher('/aqua_rl/reward', Reward, queue_size=10)
    
    self.critic_state_pub = rospy.Publisher('/aqua_rl/critic_state', String, queue_size=10)
    
    self.reset()
    
    rospy.loginfo('Critic started')


  def hackCallback(self, req):
    try:
      self.params = pickle.loads(req.policy_param)
    except pickle.UnpicklingError:
      self.params = self.policies[req.policy_param]
    print 'Received policy params from policy node:\n', self.params, '\n'
    return ResetPolicyResponse()

    
  def reset(self):
    self.episode_alive = False
    self.start_time = None
    self.params = None

    
  def newEpisodeCallback(self, msg):
    self.start_time = msg.data
    self.cumulative_reward = 0
    
    self.phase_A_sum_cos_yaw = 0
    self.phase_A_sum_sin_yaw = 0
    self.phase_A_count = 0
    self.phase_E_sum_cos_yaw = 0
    self.phase_E_sum_sin_yaw = 0
    self.phase_E_count = 0
    
    self.episode_alive = True
    
    
  def stateCallback(self, msg):
    # Return immediately if episode has terminated or has not been started
    if not self.episode_alive:
      return
      
    # Return if policy has not sent over current params
    if self.params is None:
      rospy.loginfo('Cannot evaluate state because did not receive parameters from policy yet')
      return

    # Determine episode time (which should be the same as simulation time)
    episode_time = (rospy.Time.now() - self.start_time).to_sec()
    
    # Determine current phase
    if episode_time < self.params.A_duration:
      policy_phase = DriftPhase.A_SWIM_FORWARD
    elif episode_time < self.params.A_duration + self.params.B_duration:
      policy_phase = DriftPhase.B_ANTICIPATE_RIGHT
    elif episode_time < self.params.A_duration + self.params.B_duration + self.params.C_duration:
      policy_phase = DriftPhase.C_TURN_LEFT
    elif episode_time < self.params.A_duration + self.params.B_duration + self.params.C_duration + self.params.D_duration:
      policy_phase = DriftPhase.D_DRIFT
    else:
      policy_phase = DriftPhase.E_TERMINAL

    # Accumulate estimates for average yaw headings in phases A and E
    if policy_phase == DriftPhase.A_SWIM_FORWARD:
      self.phase_A_sum_cos_yaw = math.cos(msg.YawAngle*radian)
      self.phase_A_sum_sin_yaw = math.sin(msg.YawAngle*radian)
      self.phase_A_count += 1
    elif policy_phase == DriftPhase.E_TERMINAL:
      self.phase_E_sum_cos_yaw = math.cos(msg.YawAngle*radian)
      self.phase_E_sum_sin_yaw = math.sin(msg.YawAngle*radian)
      self.phase_E_count += 1

    # Determine end-of-episode condition
    end_of_episode = (episode_time > min(self.max_episode_time, self.params.A_duration + self.params.B_duration + self.params.C_duration + self.params.D_duration + self.params.E_duration))

    # Compute instantaneous reward
    roll_dist = angularDist(msg.RollAngle*radian, 0, two_pi)
    pitch_dist = angularDist(msg.PitchAngle*radian, 0, two_pi)
    instantaneous_reward = 0
    if policy_phase == DriftPhase.D_DRIFT: # Only care about flatness in drift mode
      instantaneous_reward = -roll_dist*roll_dist - pitch_dist*pitch_dist
    if end_of_episode:
      phase_A_yaw = 0
      if self.phase_A_count > 0:
        phase_A_yaw = math.atan2(self.phase_A_sum_sin_yaw, self.phase_A_sum_cos_yaw)
      else:
        rospy.logerr('Did not collect any yaw angles for phase %s' % (DriftPhase.reverse_mapping[DriftPhase.A_SWIM_FORWARD]))
      phase_E_yaw = 0
      if self.phase_E_count > 0:
        phase_E_yaw = math.atan2(self.phase_E_sum_sin_yaw, self.phase_E_sum_cos_yaw)
      else:
        rospy.logerr('Did not collect any yaw angles for phase %s' % (DriftPhase.reverse_mapping[DriftPhase.E_TERMINAL]))
      alignment_reward = -math.pow(angularDist(phase_A_yaw, phase_E_yaw + math.pi, two_pi), 2)*self.alignment_reward_amplifier
      drift_length_reward = self.params.D_duration
      instantaneous_reward += alignment_reward + drift_length_reward
      rospy.loginfo('End-of-episode reward: phase A yaw: %.4f, phase E yaw: %.4f, alignment reward: %.4f, drift length reward: %.4f' % (phase_A_yaw/radian, phase_E_yaw/radian, alignment_reward, drift_length_reward))

      # TODO: remove debug publisher
      critic_state_msg = String('phase_A_yaw: %.4f, phase_E_yaw: %.4f, alignment_reward: %.4f, drift_length_reward: %.4f, cumulative_reward: %.4f' % (phase_A_yaw/radian, phase_E_yaw/radian, alignment_reward, drift_length_reward, self.cumulative_reward+instantaneous_reward))
      self.critic_state_pub.publish(critic_state_msg)
    
    # Accumulate into cumulative reward
    self.cumulative_reward += instantaneous_reward
    
    # Publish reward
    reward_msg = Reward()
    reward_msg.header.stamp = rospy.Time.now()
    reward_msg.episode_time = episode_time
    reward_msg.instantaneous_reward = instantaneous_reward
    reward_msg.cumulative_reward = self.cumulative_reward
    self.reward_pub.publish(reward_msg)

    rospy.loginfo('stateCallback(t=%.4f; rpy=%.1f, %.1f, %.1f; ri=%.4f, rc=%.4f' % (episode_time, msg.RollAngle, msg.PitchAngle, msg.YawAngle, instantaneous_reward, self.cumulative_reward))
    
    # Determine end-of-episode condition and publish
    if end_of_episode:
      self.episode_alive = False
      self.terminate_pub.publish(String('maximum episode time reached (curr episode time: %.4f, max episode time: %.4f)' % (episode_time, self.max_episode_time)))
      rospy.loginfo('stateCallback(episode_time=%.4f, end-of-episode detected)' % episode_time)


  def resetCallback(self, req):
    self.reset()
    return EmptyResponse()

  
  def spin(self):
    rospy.spin()


if __name__ == '__main__':
  rospy.init_node('drift_critic')

  rospack = rospkg.RosPack()
  parser = argparse.ArgumentParser()
  parser.add_argument('-d', '--dbfile', type=str, help='path to database file', required=False, default='%s/nodes/drift_policies.db' % (rospack.get_path('aqua_gait')))
  parser.add_argument('-m', '--maxtime', type=float, help='maximum episode time', required=False, default=90.0)
  args = parser.parse_args(args=rospy.myargv(argv=sys.argv[1:]))
  
  try:
    dc = Critic(args.dbfile, args.maxtime)
    dc.spin()
  except rospy.ROSInterruptException:
    pass
