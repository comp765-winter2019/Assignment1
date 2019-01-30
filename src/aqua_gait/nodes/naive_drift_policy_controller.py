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
import math
import argparse
import cPickle as pickle
import rospy
import rospkg
import std_msgs.msg
from std_msgs.msg import String, Float64, Time
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from aquacore.msg import StateMsg, Command
from aqua_rl.srv import ResetPolicy, ResetPolicyRequest, ResetPolicyResponse
from aqua_rl.srv import StepPolicy, StepPolicyRequest, StepPolicyResponse
from aquacore.srv import StepSimulation, StepSimulationRequest, StepSimulationResponse
from aquacore.srv import RunSimulationUntilTime, RunSimulationUntilTimeRequest, RunSimulationUntilTimeResponse


radian = math.pi/180.0


class Controller:
  def __init__(self, theta):
    for service in ['/gazebo/reset_world', '/gazebo/pause_physics', '/gazebo/unpause_physics', '/aqua_rl/reset_policy', '/aqua_rl/step_policy', '/aqua_gazebo/run_until_time']:
      rospy.loginfo('Waiting for ' + service)
      rospy.wait_for_service(service)
    
    self.theta = theta
    #self.reset_sim_cln = rospy.ServiceProxy('/gazebo/reset_simulation', Empty) # WARNING: DO NOT RESET SIMULATION, EVER, OTHERWISE TF WOULD BREAK (TF_OLD_TIME ERRORS)
    self.reset_sim_world_cln = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    self.pause_sim_cln = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    self.unpause_sim_cln = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    self.reset_policy_cln = rospy.ServiceProxy('/aqua_rl/reset_policy', ResetPolicy)
    self.step_policy_cln = rospy.ServiceProxy('/aqua_rl/step_policy', StepPolicy)
    self.step_simulator_cln = rospy.ServiceProxy('/aqua_gazebo/run_until_time', RunSimulationUntilTime)
    self.new_episode_pub = rospy.Publisher('/aqua_rl/new_episode', Time, queue_size=10)
    self.run_live_pub = rospy.Publisher('/aqua_rl/run_live', std_msgs.msg.Empty, queue_size=10)
    rospy.loginfo('Policy Controller started')
    
  
  def runLive(self, maxTime):
    time.sleep(0.5) # not sure why need this delay, but otherwise /aqua_rl/new_episode doesn't publish properly

    rospy.loginfo('=== RUN LIVE (%.4f) ===' % maxTime)
    
    rospy.loginfo('Resetting robot pose')
    #self.reset_sim_cln(EmptyRequest())
    self.reset_sim_world_cln(EmptyRequest())

    rospy.loginfo('Resetting policy at start of runLive')
    self.reset_policy_cln(ResetPolicyRequest(self.theta))
    
    time_msg = Time()
    time_msg.data = rospy.Time.now()
    rospy.loginfo('Announcing new episode start time')
    self.new_episode_pub.publish(time_msg)

    rospy.loginfo('Unpausing simulation')
    self.unpause_sim_cln(EmptyRequest())
    
    rospy.loginfo('Publishing run_live')
    self.run_live_pub.publish(EmptyMsg())

    rospy.loginfo('Sleeping for %.2f seconds before stopping live episode' % maxTime)
    try:
      time.sleep(maxTime)
    except KeyboardInterrupt:
      pass
    
    rospy.loginfo('Resetting policy at end of runLive')
    self.reset_policy_cln(ResetPolicyRequest(self.theta))
    
    rospy.loginfo('runLive done!')
    
    
  def runSim(self, maxTime, incrTime, superRealTimeFactor):
    rospy.loginfo('=== RUN SIM (%.4f, %.4f, %.4f) ===' % (maxTime, incrTime, superRealTimeFactor))

    rospy.loginfo('Unpausing simulation to unlock /clock')
    self.unpause_sim_cln(EmptyRequest())

    rospy.loginfo('Resetting robot pose')
    #self.reset_sim_cln(EmptyRequest())
    self.reset_sim_world_cln(EmptyRequest())

    rospy.loginfo('Resetting policy at start of runSim')
    self.reset_policy_cln(ResetPolicyRequest(self.theta))

    rospy.loginfo('Pausing simulation')
    self.pause_sim_cln(EmptyRequest())

    sim_start_time = rospy.Time.now()
    time_msg = Time()
    time_msg.data = sim_start_time
    rospy.loginfo('Announcing new episode start time')
    self.new_episode_pub.publish(time_msg)

    latest_real_time = time.time()
    for i in xrange(int(math.ceil(float(maxTime)/incrTime))):
      t = float(i*incrTime)
      desired_time = sim_start_time + rospy.Duration(t)
      
      step_policy_req = StepPolicyRequest(desired_time)
      rospy.loginfo('StepPolicy(%.4f)' % (step_policy_req.desired_time).to_sec())
      self.step_policy_cln(step_policy_req)
      
      step_sim_req = RunSimulationUntilTimeRequest(desired_time)
      rospy.loginfo('StepSimulator(%.4f)' % (step_sim_req.desired_time).to_sec())
      self.step_simulator_cln(step_sim_req)
      
      # For now we will sleep for some time before stepping next; in reality we would go as fast as plant lets us
      sleep_dur = incrTime/superRealTimeFactor
      now = time.time()
      real_time_diff = now - latest_real_time
      if real_time_diff < sleep_dur:
        try:
          time.sleep(sleep_dur - real_time_diff)
        except KeyboardInterrupt:
          break
      latest_real_time = time.time()
    
    rospy.loginfo('Resetting policy at end of runSim')
    self.reset_policy_cln(ResetPolicyRequest(self.theta))

    rospy.loginfo('Unpausing simulation to unlock /clock')
    self.unpause_sim_cln(EmptyRequest())
    
    rospy.loginfo('runSim done!')


if __name__ == '__main__':
  rospy.init_node('drift_policy_controller')

  rospack = rospkg.RosPack()
  parser = argparse.ArgumentParser()
  parser.add_argument('-l', '--live', required=False, action='store_true')
  parser.add_argument('-m', '--maxtime', type=float, help='Maximum time to run (both in real time and step time) before resetting policy', required=False, default=45.0)
  parser.add_argument('-s', '--simsteptime', type=float, help='Step time in simulation', required=False, default=0.1)
  parser.add_argument('-p', '--superrealtimefactor', type=float, help='Super real time factor when stepping in simulation', required=False, default=2.0)
  parser.add_argument('-t', '--theta', type=str, help='Filename of policy parameters', required=False, default='0' )
  args = parser.parse_args(args=rospy.myargv(argv=sys.argv[1:]))
  
  try:
    ctrlr = Controller(args.theta)
    if args.live:
      ctrlr.runLive(args.maxtime)
    else:
      ctrlr.runSim(args.maxtime, args.simsteptime, args.superrealtimefactor)
  except rospy.ROSInterruptException:
    pass
  except KeyboardInterrupt:
    pass
