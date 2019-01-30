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
import rosbag
import tf
from std_msgs.msg import String, Time
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from geometry_msgs.msg import PoseStamped
from aquacore.msg import StateMsg, PeriodicLegCommand, Command, AutopilotModes
from aquacore.srv import SetAutopilotMode, SetAutopilotModeRequest, SetGait, SetGaitRequest
from aqua_rl.srv import ResetPolicy, ResetPolicyRequest, ResetPolicyResponse
from aqua_rl.srv import StepPolicy, StepPolicyRequest, StepPolicyResponse


def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    reverse = dict((value, key) for key, value in enums.iteritems())
    enums['reverse_mapping'] = reverse
    return type('Enum', (), enums)


TopicType = enum('AP_CMD', 'BODY_CMD', 'PLC_CMD')


class Playback:
  def __init__(self, bagfile, episode, target_topic):
    # Parse arguments and set variables
    self.bagfile = bagfile
    self.target_episode = episode
    self.target_topic = target_topic
    if self.target_topic == '/aqua/target_pose':
      self.topic_type = TopicType.AP_CMD
    elif self.target_topic == '/aqua/command':
      self.topic_type = TopicType.BODY_CMD
    elif self.target_topic == '/aqua/periodic_leg_command':
      self.topic_type = TopicType.PLC_CMD
    else:
      raise Exception('Unrecognized topic type: %s' % target_topic)
      

    # Extract content from bag
    rospy.loginfo('Loading %s' % bagfile)
    bag = rosbag.Bag(bagfile)    
    
    self.cmds = [] # list of tuples of (dt, cmd)
    curr_episode = -1
    episode_start_wall_time = None
    rospy.loginfo('Parsing %s' % bagfile)
    for topic, msg, t in bag.read_messages(topics=['/aqua_rl/state', '/aqua_rl/new_episode', self.target_topic]):
      if topic == '/aqua_rl/state':
        curr_episode = msg.episode

      elif topic == '/aqua_rl/new_episode':
        episode_start_wall_time = t
      
      elif topic == '/aqua/target_pose' and self.topic_type == TopicType.AP_CMD:
        if episode_start_wall_time is not None:
          self.cmds.append(((t-episode_start_wall_time).to_sec(), msg))
        
      elif topic == '/aqua/command' and self.topic_type == TopicType.BODY_CMD:
        if episode_start_wall_time is not None:
          self.cmds.append(((t-episode_start_wall_time).to_sec(), msg))
        
      elif topic == '/aqua/periodic_leg_command' and self.topic_type == TopicType.PLC_CMD:
        if episode_start_wall_time is not None:
          self.cmds.append(((t-episode_start_wall_time).to_sec(), msg))
          
    bag.close()
    rospy.loginfo('Parsed %d commands on %s in %s' % (len(self.cmds), self.target_topic, bagfile))
      
    if len(self.cmds) <= 0:
      raise Exception('Failed to parse commands on %s in %s' % (self.target_topic, bagfile))
    
    self.start_time = None # not needed when not in live mode; otherwise set to current ROS time when run live is called
    self.robot_state = None
    self.cmd_idx = 0
    self.episode_alive = False
    self.live_mode = False
        
    for service in ['/aqua/set_gait', '/aqua/set_3Dauto_mode', '/AP_depth_filter/reset', '/hover_midoff_node/reset']:
      rospy.loginfo('Waiting for' + service)
      rospy.wait_for_service(service)
    rospy.loginfo('All necessary services found')

    self.set_gait_cln = rospy.ServiceProxy('/aqua/set_gait', SetGait)
    self.ap_mode_cln = rospy.ServiceProxy('/aqua/set_3Dauto_mode', SetAutopilotMode)
    self.reset_hover_midoff_cln = rospy.ServiceProxy('/hover_midoff_node/reset', Empty)
    self.reset_ap_cln = rospy.ServiceProxy('/aqua/reset_3D_autopilot_state', Empty)
    self.reset_depth_filter_cln = rospy.ServiceProxy('/AP_depth_filter/reset', Empty)
    
    self.reset_critic_srv = rospy.Service('/aqua_rl/reset_critic', Empty, self.resetCriticCallback)
    self.reset_policy_srv = rospy.Service('/aqua_rl/reset_policy', ResetPolicy, self.resetPolicyCallback)
    self.step_srv = rospy.Service('/aqua_rl/step_policy', StepPolicy, self.stepCallback)
    self.new_episode_sub = rospy.Subscriber('/aqua_rl/new_episode', Time, self.newEpisodeCallback, queue_size=10)
    self.run_live_sub = rospy.Subscriber('/aqua_rl/run_live', EmptyMsg, self.runLiveCallback, queue_size=10)
    self.state_sub = rospy.Subscriber('/aqua/state', StateMsg, self.stateCallback, queue_size=10)
    self.terminate_pub = rospy.Publisher('/aqua_rl/terminate_episode', String, queue_size=10)
    
    self.ap_pub = rospy.Publisher('/aqua/target_pose', PoseStamped, queue_size=10)
    self.plc_pub = rospy.Publisher('/aqua/periodic_leg_command', PeriodicLegCommand, queue_size=10)
    self.cmd_pub = rospy.Publisher('/aqua/command', Command, queue_size=10)
    
    rospy.logdebug('Setting gait to flexible-sine (since hover_midoff_node will translate)')
    self.set_gait_cln('flexible-sine')
    
    self.reset()
    
    rospy.loginfo('Playback policy+critic started')
    
    
  def reset(self, param_str = '0'):
    self.start_time = None
    self.robot_state = None
    self.cmd_idx = 0
    self.episode_alive = False
    self.live_mode = False

    self.reset_hover_midoff_cln(EmptyRequest())
    
    self.ap_cmd = PoseStamped()
    self.ap_cmd.header.frame_id = 'aqua'
    self.ap_cmd.header.stamp = time.time()
    self.ap_cmd.pose.position.x = 0
    self.ap_cmd.pose.position.y = 0
    self.ap_cmd.pose.position.z = 0
    quat = tf.transformations.quaternion_from_euler(0, 0, 0, 'sxyz')
    self.ap_cmd.pose.orientation.x = quat[0]
    self.ap_cmd.pose.orientation.y = quat[1]
    self.ap_cmd.pose.orientation.z = quat[2]
    self.ap_cmd.pose.orientation.w = quat[3]
    
    if self.topic_type == TopicType.AP_CMD:
      rospy.logdebug('Resetting autopilot state')
      self.reset_ap_cln(EmptyRequest())
      
      rospy.logdebug('Setting autopilot mode to AutopilotModes.DEPTH')
      self.ap_mode_cln(SetAutopilotModeRequest(AutopilotModes.DEPTH))

      rospy.logdebug('Resetting autopilot depth filter')
      self.reset_depth_filter_cln(EmptyRequest())
    
    
  def step(self, state, dt):
    if not self.episode_alive:
      rospy.logdebug('Ignoring step(%.4f) since not self.episode_alive' % (dt))
      return
    
    stepping = True
    while stepping:
      if self.cmd_idx >= len(self.cmds):
        stepping = False
        break
        
      elif self.cmds[self.cmd_idx][0] <= dt:
        # Publish
        if self.topic_type == TopicType.AP_CMD:
          msg = self.cmds[self.cmd_idx][1]
          msg.header.stamp = rospy.Time.now()
          msg.header.frame_id = 'aqua'
          self.ap_pub.publish(msg)
        
        elif self.topic_type == TopicType.BODY_CMD:
          self.cmd_pub.publish(self.cmds[self.cmd_idx][1])
        
        elif self.topic_type == TopicType.PLC_CMD:
          msg = self.cmds[self.cmd_idx][1]
          msg.header.stamp = rospy.Time.now()
          msg.header.frame_id = 'aqua'
          self.plc_pub.publish(msg)
        
        self.cmd_idx += 1
        
      else:
        stepping = False
        break
        
    if self.cmd_idx >= len(self.cmds):
      self.terminate_pub.publish(String('Playback ended at %.4f with last msg at %.4f' % (dt, self.cmds[-1][0])))
      self.episode_alive = False

    
  def newEpisodeCallback(self, msg):
    self.start_time = msg.data
    rospy.logdebug('Received new episode time')
    self.episode_alive = True

    
  def runLiveCallback(self, msg):
    self.live_mode = True
    rospy.logdebug('Entering live mode')
    
    
  def stateCallback(self, msg):
    self.robot_state = msg
    if self.live_mode and self.start_time is not None:
      self.step(msg, (rospy.Time.now() - self.start_time).to_sec())


  def resetCriticCallback(self, req):
    return EmptyResponse()
    

  def resetPolicyCallback(self, req):
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
  rospy.init_node('playback_policy_critic')

  rospack = rospkg.RosPack()
  parser = argparse.ArgumentParser()
  parser.add_argument('-b', '--bagfile', type=str, help='path to bag file', required=True)
  parser.add_argument('-e', '--episode', type=int, help='episode to play back', required=False, default=1)
  parser.add_argument('-t', '--topic', type=str, help='policy topic (/aqua/target_pose | /aqua/command | /aqua/periodic_leg_command)', required=False, default='/aqua/periodic_leg_command')
  args = parser.parse_args(args=rospy.myargv(argv=sys.argv[1:]))
  
  try:
    player = Playback(args.bagfile, args.episode, args.topic)
    player.spin()
  except rospy.ROSInterruptException:
    pass
