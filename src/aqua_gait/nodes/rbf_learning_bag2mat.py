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



import math
import sys
import rosbag
import roslib
import rospkg
import argparse
import rospy
import scipy.io
from std_msgs.msg import String
from aqua_rl.msg import Reward, ControllerState
from rbf_common import *


class PLC:
  def __init__(self, time, episode_idx, step_idx, amplitudes, frequencies, phase_offsets, leg_offsets):
    self.time = time
    self.episode_idx = episode_idx
    self.step_idx = step_idx
    self.amplitudes = amplitudes
    self.frequencies = frequencies
    self.phase_offsets = phase_offsets
    self.leg_offsets = leg_offsets
  
  def __str__(self):
    return 'time: %.4f\nepisode_idx: %.4f\nstep_idx: %.4f\namplitudes: %s\nfrequencies: %s\nphase_offsets: %s\nleg_offsets: %s\n' % (self.time, self.episode_idx, self.step_idx, self.amplitudes, self.frequencies, self.phase_offsets, self.leg_offsets)


class AquaState:
  def __init__(self, time, episode_idx, step_idx, RollAngle, PitchAngle, YawAngle, Depth, Speed):
    self.time = time
    self.episode_idx = episode_idx
    self.step_idx = step_idx
    self.roll_angle = RollAngle
    self.pitch_angle = PitchAngle
    self.yaw_angle = YawAngle
    self.depth = Depth
    self.speed = Speed
  
  def __str__(self):
    return 'time: %.4f\nepisode_idx: %.4f\nstep_idx: %.4f\nroll_angle: %.4f\npitch_angle: %.4f\nyaw_angle: %.4f\ndepth: %.4f\nspeed: %.4f\n' % (self.time, self.episode_idx, self.step_idx, self.roll_angle, self.pitch_angle, self.yaw_angle, self.depth, self.speed)


class EndOfEpisode:
  def __init__(self, wall_time, episode_idx, step_idx, cumulative_reward, curr_params, delta_params, param_idx, reward_orig, reward_pos, reward_neg, twiddle_state):
    if reward_orig is None:
      reward_orig = float('nan')
    if reward_pos is None:
      reward_pos = float('nan')
    if reward_neg is None:
      reward_neg = float('nan')
    self.wall_time = wall_time
    self.episode_idx = episode_idx
    self.step_idx = step_idx
    self.cumulative_reward = cumulative_reward
    self.curr_params_vec = [curr_params[i] for i in xrange(curr_params.getParamLen())]
    self.delta_params = delta_params
    self.param_idx = param_idx
    self.twiddle_reward_best = reward_orig
    self.twiddle_reward_orig = reward_orig
    self.twiddle_reward_pos = reward_pos
    self.twiddle_reward_neg = reward_neg
    self.twiddle_state = twiddle_state
  
  def __str__(self):
    return 'wall_time: %.4f\nepisode_idx: %d\nstep_idx: %d\ncumulative_reward: %.4f\ncurr_params_vec: %s\ndelta_params: %s\nparam_idx: %d\ntwiddle_reward_best: %.4f\ntwiddle_reward_orig: %.4f\ntwiddle_reward_pos: %.4f\ntwiddle_reward_neg: %.4f\ntwiddle_state: %s\n' % (self.wall_time, self.episode_idx, self.step_idx, self.cumulative_reward, str(self.curr_params_vec), str(self.delta_params), self.param_idx, self.twiddle_reward_best, self.twiddle_reward_orig, self.twiddle_reward_pos, self.twiddle_reward_neg, self.twiddle_state)


def bag2mat(init_policy_file, bagfile):
  print 'Loading', bagfile
  bag = rosbag.Bag(bagfile)
  episode_idx = -1
  step_idx = -1
  curr_params = RBFPolicyParams.load(init_policy_file)
  aqua_state_vec = []
  plc_vec = []
  eoe_vec = []
  cumulative_reward = None
  
  print 'Parsing', bagfile
  topics = [
    '/aqua/state',
    '/aqua/periodic_leg_command',
    '/aqua_rl/reward',
    '/aqua_rl/state',
    '/aqua_rl/learner_state']
  for topic, msg, t in bag.read_messages(topics=topics):
    if topic == '/aqua_rl/state':
      episode_idx = msg.episode
      step_idx = msg.step
      
    elif topic == '/aqua/state':
      aqua_state_data = AquaState(t.to_sec(), episode_idx, step_idx, msg.RollAngle, msg.PitchAngle, msg.YawAngle, msg.Depth, msg.Speed)
      #print aqua_state_data
      aqua_state_vec.append(aqua_state_data)
      
    elif topic == '/aqua/periodic_leg_command':
      plc_data = PLC(t.to_sec(), episode_idx, step_idx, msg.amplitudes, msg.frequencies, msg.phase_offsets, msg.leg_offsets)
      #print plc_data
      plc_vec.append(plc_data)
      
    elif topic == '/aqua_rl/reward':
      cumulative_reward = msg.cumulative_reward
      plc_vec.append(plc_data)
      
    elif topic == '/aqua_rl/learner_state':
      twiddle_state = pickle.loads(msg.data)
      
      if episode_idx is None:
        raise Exception('episode_idx is none @ time t=%.4f' % t.to_sec())
      if step_idx is None:
        raise Exception('step_idx is none @ time t=%.4f' % t.to_sec())
      if curr_params is None:
        raise Exception('curr_params is none @ time t=%.4f' % t.to_sec())
      
      # Save entry
      if twiddle_state is None:
        raise Exception('No twiddle_state at end of episode')
      elif cumulative_reward is None:
        raise Exception('No cumulative_reward at end of episode')
      eoe_data = EndOfEpisode(t.to_sec(), episode_idx, step_idx, cumulative_reward, curr_params, twiddle_state['delta_params'], twiddle_state['param_idx'], twiddle_state['reward_orig'], twiddle_state['reward_pos'], twiddle_state['reward_neg'], twiddle_state['twiddle_state'])
      #print eoe_data
      eoe_vec.append(eoe_data)

      # Start scanning data from the next episode
      curr_params = twiddle_state['curr_params'] # the current params are in fact the next twiddle state params
      episode_idx = -1
      step_idx = -1
  bag.close()
  
  matfile = '%s.mat' % bagfile
  print 'Saving', matfile
  scipy.io.savemat(file_name=matfile, mdict={'aqua_state_data': aqua_state_vec, 'ap_cmd_data': [], 'plc_data': plc_vec, 'eoe_data': eoe_vec}, oned_as='row')
  
  print 'All done!'


if __name__ == '__main__':
  rospack = rospkg.RosPack()
  parser = argparse.ArgumentParser()
  parser.add_argument('-p', '--init_policy_file', type=str, help='path to sample policy (will specify shape, has velocity, num rbfs, etc)', required=True, default='%s/misc/initial_policy.h5' % (rospack.get_path('aqua_gait')))
  parser.add_argument('-b', '--bagfile', type=str, help='path to rosbag file', required=True)
  args = parser.parse_args()
  bag2mat(args.init_policy_file, args.bagfile)
