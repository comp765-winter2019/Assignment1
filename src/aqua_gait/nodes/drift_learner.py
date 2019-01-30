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
import random
import argparse
import rospy
import rospkg
from std_msgs.msg import String
from aqua_rl.msg import Reward
from aqua_rl.srv import Learn, LearnRequest, LearnResponse
from drift_common import *


TwiddleState = enum('QUERY_ORIG', 'QUERY_POS', 'QUERY_NEG')


class Learner:
  def __init__(self, dbfile, learner_type='twiddle', on_real_robot='false'):
    self.alive = True
    self.learner_type = learner_type
    self.policies = pickle.load(open(dbfile, "r"))
    self.curr_params = self.policies['0']
    self.latest_reward = None
    
    # Parameter specifications
    self.num_params = 11
    self.variable_params = [False, False,
      False, True, False,
      False, True, False,
      True,
      False, False]
    if on_real_robot:
      self.params_min = [0, 0,
        0, 0, 0,
        0, -1, 0,
        1,
        0, 0]
      self.params_max = [1.0, 10,
        1.0, 1, 10,
        1.0, 0, 10,
        10,
        1.0, 10]
    else:
      self.params_min = [0, 0,
        0, 0, 0,
        0, -1, 0,
        1,
        0, 0]
      self.params_max = [1, 20,
        1, 1, 20,
        1, 0, 20,
        20,
        1, 20]
    self.delta_params = [abs(self.params_max[i] - self.params_min[i])*0.1 for i in xrange(self.num_params)]
    
    # Twiddle state
    self.param_idx = None
    for i in xrange(self.num_params):
      if self.variable_params[i]:
        self.param_idx = i
        break
    if self.param_idx is None:
      raise Exception('No params are variable; cannot learn')
    self.reward_orig = None
    self.reward_pos = None
    self.reward_neg = None
    self.twiddle_state = TwiddleState.QUERY_ORIG
    
    self.learn_srv = rospy.Service('/aqua_rl/update_policy', Learn, self.learnCallback)
    self.reward_sub = rospy.Subscriber('/aqua_rl/reward', Reward, self.rewardCallback)
    self.learner_state_pub = rospy.Publisher('/aqua_rl/learner_state', String, queue_size=10)
    
    rospy.loginfo('Learner started')
    
    
  def rewardCallback(self, msg):
    self.latest_reward = msg
    
    
  def learnCallback(self, req):
    new_params = None
    if self.learner_type == 'twiddle':
      if self.latest_reward is None:
        rospy.logerr('Cannot learn since no rewards were received')
        return LearnResponse(pickle.dumps(self.curr_params))
      else:
        while new_params is None:
          new_params = self.stepTwiddle(self.latest_reward.cumulative_reward)
    elif self.learner_type == 'none':
      return LearnResponse(pickle.dumps(self.curr_params))
    else:
      rospy.logerr('Invalid learn type: ' + self.learner_type)

    # Publish twiddle state as pickled string
    twiddle_state_dict = {'variable_params': self.variable_params, 'params_min': self.params_min, 'params_max': self.params_max, 'delta_params': self.delta_params, 'curr_params': self.curr_params, 'param_idx': self.param_idx, 'reward_orig': self.reward_orig, 'reward_pos': self.reward_pos, 'reward_neg': self.reward_neg, 'twiddle_state': self.twiddle_state, 'twiddle_state_str': TwiddleState.reverse_mapping[self.twiddle_state]}
    state_msg = String(pickle.dumps(twiddle_state_dict))
    self.learner_state_pub.publish(state_msg)

    rospy.loginfo('TWIDDLE STATE AFTER EPISODE WITH REWARD: %.4f' % (self.latest_reward.cumulative_reward))
    rospy.loginfo('variable_params: ' + str(self.variable_params))
    rospy.loginfo('params_min: ' + str(self.params_min))
    rospy.loginfo('params_max: ' + str(self.params_max))
    rospy.loginfo('delta_params: ' + str(self.delta_params))
    rospy.loginfo('curr_params: ' + str([self.curr_params[i] for i in xrange(self.num_params)]))
    rospy.loginfo('param_idx: ' + str(self.param_idx))
    rospy.loginfo('reward_orig: ' + str(self.reward_orig))
    rospy.loginfo('reward_pos: ' + str(self.reward_pos))
    rospy.loginfo('reward_neg: ' + str(self.reward_neg))
    rospy.loginfo('twiddle_state: ' + str(TwiddleState.reverse_mapping[self.twiddle_state]))
    rospy.loginfo('\n')
    
    self.latest_reward = None
        
    return LearnResponse(new_params)
    
    
  def testTwiddleCallback(self, cumulative_reward):
    new_params = None
    while new_params is None:
      new_params = self.stepTwiddle(cumulative_reward)

    print 'TWIDDLE STATE'
    print 'variable_params:', self.variable_params
    print 'params_min:', self.params_min
    print 'params_max:', self.params_max
    print 'delta_params:', self.delta_params
    print 'curr_params:', [self.curr_params[i] for i in xrange(self.num_params)]
    print 'param_idx:', self.param_idx
    print 'reward_orig:', self.reward_orig
    print 'reward_pos:', self.reward_pos
    print 'reward_neg:', self.reward_neg
    print 'twiddle_state:', TwiddleState.reverse_mapping[self.twiddle_state]
    print '\n'
    
    return new_params
        
  
  def stepTwiddle(self, new_cumulative_reward):
    recall_step_twiddle = False
    next_param = False
    if self.twiddle_state == TwiddleState.QUERY_ORIG:
      if self.reward_orig is None:
        self.reward_orig = new_cumulative_reward
      self.orig_param_val = self.curr_params[self.param_idx]
      
      # Try positive step
      self.twiddle_state = TwiddleState.QUERY_POS
      if self.curr_params[self.param_idx] >= self.params_max[self.param_idx]:
        #print 'DEBUG: @ max'
        recall_step_twiddle = True # Currently at max, so skip positive direction
      else:
        new_param_val = self.curr_params[self.param_idx] + self.delta_params[self.param_idx]
        if new_param_val > self.params_max[self.param_idx]:
          new_param_val = self.params_max[self.param_idx]
        self.curr_params[self.param_idx] = new_param_val
        
    elif self.twiddle_state == TwiddleState.QUERY_POS:
      self.reward_pos = new_cumulative_reward
      if self.reward_pos > self.reward_orig:
        # Accept positive step
        self.delta_params[self.param_idx] = self.delta_params[self.param_idx] * 1.1
        next_param = True
        self.twiddle_state = TwiddleState.QUERY_ORIG
        recall_step_twiddle = True
        self.reward_orig = self.reward_pos
      else:
        # Try negative step
        self.twiddle_state = TwiddleState.QUERY_NEG
        if self.curr_params[self.param_idx] <= self.params_min[self.param_idx]:
          #print 'DEBUG: @ min'
          recall_step_twiddle = True # Currently at min, so skip negative direction
        else:
          new_param_val = self.curr_params[self.param_idx] - 2*self.delta_params[self.param_idx]
          if new_param_val < self.params_min[self.param_idx]:
            new_param_val = self.params_min[self.param_idx]
          self.curr_params[self.param_idx] = new_param_val
        
    elif self.twiddle_state == TwiddleState.QUERY_NEG:
      self.reward_neg = new_cumulative_reward
      if self.reward_neg > self.reward_orig:
        # Accept negative step
        self.delta_params[self.param_idx] = self.delta_params[self.param_idx] * 1.1 # keep the sign positive to simplify logic; next time for the current index it will step against the previously-negative gradient at first
        next_param = True
        self.twiddle_state = TwiddleState.QUERY_ORIG
        recall_step_twiddle = True
        self.reward_orig = self.reward_neg
      else:
        #print 'DEBUG: @revert to orig,', self.orig_param_val
        # Revert to original param values and penalize current dimension
        self.curr_params[self.param_idx] = self.orig_param_val
        self.delta_params[self.param_idx] = self.delta_params[self.param_idx] * 0.9
        # Do not reset self.reward_orig, since we reverted to the original set of parameters
        next_param = True
        self.twiddle_state = TwiddleState.QUERY_ORIG
        recall_step_twiddle = True
        
    if next_param:
      found_next = False
      for i in xrange(1, self.num_params):
        idx = (self.param_idx + i) % self.num_params
        if self.variable_params[idx]:
          found_next = True
          self.param_idx = idx
          #print 'DEBUG: next idx:', self.param_idx
          break
      if found_next is False:
        print self.param_idx
        print self.variable_params
        raise Exception('Failed to find next idx')
        
    if recall_step_twiddle:
      return None

    return pickle.dumps(self.curr_params)


  def spin(self):
    rospy.spin()


if __name__ == '__main__':
  rospy.init_node('drift_learner')

  rospack = rospkg.RosPack()
  parser = argparse.ArgumentParser()
  parser.add_argument('-d', '--dbfile', type=str, help='path to database file', required=False, default='%s/nodes/drift_policies.db' % (rospack.get_path('aqua_gait')))
  parser.add_argument('-t', '--learner_type', type=str, help='Type of learner', required=False, default='twiddle')
  parser.add_argument('-z', '--realz', help='for real robot (i.e. use more conservative parameter ranges)', required=False, action='store_true')
  parser.add_argument('-u', '--unit_test', required=False, action='store_true')
  parser.add_argument('-r', '--unit_test_random', required=False, action='store_true')
  args = parser.parse_args(args=rospy.myargv(argv=sys.argv[1:]))
  
  if args.unit_test:
    dl = Learner(args.dbfile, args.learner_type)
    
    print 'ORIG PARAMS'
    print 'curr_params:', [dl.curr_params[i] for i in xrange(dl.num_params)], '\n\n'
    
    print 'A1. INIT POLICY REWARD: -10'
    dl.testTwiddleCallback(-10)
    print 'A2. POS REWARD: -11 (REJECT)'
    dl.testTwiddleCallback(-11)
    print 'A3. NEG REWARD: -12 (REJECT, KEEP ORIG, NEXT DIM)'
    dl.testTwiddleCallback(-12)
    print 'B1. POS REWARD: -9 (ACCEPT, NEXT DIM)'
    dl.testTwiddleCallback(-9)
    print 'C1. POS REWARD: -15 (REJECT)'
    dl.testTwiddleCallback(-15)
    print 'C2. NEG REWARD: -8 (ACCEPT, NEXT DIM)'
    dl.testTwiddleCallback(-8)
    print 'D1. POS REWARD: -16 (REJECT)'
    dl.testTwiddleCallback(-16)
    print 'D2. NEG REWARD: -12 (REJECT, KEEP ORIG, NEXT DIM)'
    dl.testTwiddleCallback(-12)
    print 'E1. POS REWARD: -7 (ACCEPT, NEXT DIM)'
    dl.testTwiddleCallback(-7)
    print 'F1. POS REWARD: -15 (REJECT)'
    dl.testTwiddleCallback(-15)
    print 'F2. NEG REWARD: 0 (ACCEPT, NEXT DIM)'
    dl.testTwiddleCallback(0)
    
  elif args.unit_test_random:
    dl = Learner(args.dbfile, args.learner_type, args.realz)
    
    print 'ORIG PARAMS'
    print 'curr_params:', [dl.curr_params[i] for i in xrange(dl.num_params)], '\n\n'
    
    for i in range(1000):
      reward = round(-random.random()*100)
      print '%d. POLICY REWARD: %.4f' % (i, reward)
      dl.testTwiddleCallback(reward)

  else:
    try:
      dl = Learner(args.dbfile, args.learner_type)
      dl.spin()
    except rospy.ROSInterruptException:
      pass
