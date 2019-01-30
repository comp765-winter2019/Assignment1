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
from std_msgs.msg import String
import tf
from math import pi
import yaml

from aquacore.msg import AutopilotModes
from autopilot_client import AutopilotClient  
import dynamic_reconfigure.client
from ar_recog.msg import Tags
from aquacore.msg import StateMsg

class ShipProgram(object):
    
    def __init__(self,speed,forward_swim_time,sea_floor_depth,deck_depth,hold_depth,tag_dead_zone,do_swimmer_turn):
        rospy.loginfo('In ShipProgram init')
        self.depth_params = {}
        self.depth_params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_FIXED_DEPTH
        
        self.angle_params = {}
        self.angle_params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_LOCAL_THRUST
    
        self.ac = AutopilotClient(self.depth_params)
         
        self.client = dynamic_reconfigure.client.Client("/localAP", timeout=5 )
            
        self.knife_edge_params = { 'ROLL_P_GAIN' : 1.0, 
                              'PITCH_P_GAIN' : 2.0,
                              'YAW_P_GAIN' : -3.5,
                              'ROLL_D_GAIN' : 0.0,
                              'PITCH_D_GAIN' : 0.0,
                              'YAW_D_GAIN' : -1.5,
                              'KDEPTH' : -1.3,
                              'DEPTH_D_GAIN' : -1.5,
                              'DEPTH_D_FILTER_PERIOD' : 0.6 }
        
        self.behaviour_dict = {    0 : [ 'flip_turn', 'forward' ],
                                   1:  [ 'right_turn', 'forward'],
                                   2:  [ 'knife_edge', 'forward'],
                                   3:  [ 'flatten', 'forward' ], 
                                   4:  [ 'left_turn', 'forward'],
                                   5:  [ 'park' ]
				   # [ 'up_to_deck', 'forward' ],  
				   # [ 'forward_timed', 'down_to_sea_floor', 'forward' ]
                                   # [ 'dance', 'forward' ]  
                                   # [ 'forward_timed',  'down_to_hold', 'forward_timed', 'up_to_deck', 'forward]
                                }
    
        self.client.update_configuration(self.knife_edge_params)
        self.ac.set_mode(self.depth_params)
        self.target_depth = self.ac.current_depth
        self.rpy_from_imu_to_global = self.ac.get_rpy_of_imu_in_global()
        self.current_roll_deg = self.rpy_from_imu_to_global[0]*180.0/pi
        self.current_pitch_deg =  self.rpy_from_imu_to_global[1]*180.0/pi
        self.current_yaw_deg = self.rpy_from_imu_to_global[2]*180.0/pi
        
        # Choose a trajectory file based on parameter and send it to the ap_client function
        self.active_command = None
        self.command_list = []
        self.command_endtime = None
        
        # TODO make params
        self.sea_floor_depth = sea_floor_depth
        self.deck_depth = deck_depth
        self.hold_depth = hold_depth
        self.forward_swim_time = forward_swim_time
        self.speed = speed
        self.tag_dead_zone = tag_dead_zone
        self.do_swimmer_turn = do_swimmer_turn
        self.last_tag_time = None
        
        rospy.Subscriber("/tags", Tags, self.tag_callback)
        rospy.Subscriber("/aqua/state", StateMsg, self.state_callback)
        
        rospy.loginfo( 'Finished constructor')        
        
    def clamp180(self,angle):
        while angle > 180.0:
            angle = angle - 360.0
        while angle < -180.0:
            angle = angle + 360.0
            
        return angle
    
    def tag_callback(self,tag):
       
        rospy.loginfo('Tag callback.') 
        if len(tag.tags) > 0:
            rospy.loginfo( 'Saw a tag.')
        else:
            return

        # A small delay after seeing a tag so that double detections dont hurt us
        if (self.last_tag_time == None) or ( rospy.Time.now() - self.last_tag_time ) > rospy.Duration(self.tag_dead_zone):
            self.last_tag_time = rospy.Time.now()
            
            # Handle the tag based on its ID:
            if not self.behaviour_dict.has_key(tag.tags[0].id):
                rospy.loginfo( 'Unrecognized tag: ' + str(tag.tags[0].id) + ' detected.') 
            else:
                self.command_list.extend( self.behaviour_dict[tag.tags[0].id] )
		rospy.loginfo( 'Read tag: ' + str(tag.tags[0].id) + ' to make action list: ' + str(self.command_list))
               
    def state_callback(self,event):
        
        rospy.loginfo('State callback. Current command is: ' + str(self.active_command))
        
        new_command = False
        
        if self.active_command == None:
            new_command = True
        else:            
            if self.active_command == 'up_to_deck':
                if self.ac.current_depth <= self.deck_depth:
                    new_command = True
            elif self.active_command == 'forward_timed':
                if self.command_endtime <= rospy.Time.now():
                    new_command = True
            elif self.active_command == 'down_to_sea_floor':
                if self.ac.current_depth >= self.sea_floor_depth:
                    new_command = True
            elif self.active_command == 'down_to_hold':
                if self.ac.current_depth >= self.hold_depth:
                    new_command = True
            else:
                rospy.loginfo( 'Error, command unknown: ' + str(self.active_command))
        
        if new_command:
            if len(self.command_list)>0:
                
                self.active_command = self.command_list.pop(0)
                
                if self.active_command == 'forward':
                    # Implementation of "flat swim"
                    rospy.loginfo('Switching to command forward')
                    # The behaviour is immediate, so can unset active command
                    self.active_command = None
                    self.current_pitch_deg = 0.0
                    self.ac.set_mode(self.depth_params)
                if self.active_command == 'right_turn':
                    self.current_yaw_deg = self.current_yaw_deg - 90.0
                    # The behaviour is immediate, so can unset active command
                    self.active_command = None
                    self.ac.set_mode(self.depth_params)
                if self.active_command == 'left_turn':
                    self.current_yaw_deg = self.current_yaw_deg + 90.0
                    # The behaviour is immediate, so can unset active command
                    self.active_command = None
                    self.ac.set_mode(self.depth_params)
                if self.active_command == 'knife_edge':
                    self.current_roll_deg = 90.0
                    # The behaviour is immediate, so can unset active command
                    self.active_command = None
                    self.ac.set_mode(self.depth_params)
                if self.active_command == 'flatten':
                    self.current_roll_deg = 0.0
                    # The behaviour is immediate, so can unset active command
                    self.active_command = None
                    self.ac.set_mode(self.depth_params)
                if self.active_command == 'dance':
                    self.ac.set_mode(self.depth_params)
                    self.ac.do_rolling_sideways_somersault(rospy.Duration(20), 0.8, 0.0, self.ac.current_depth, 18.0)
                    self.active_command = None
                if self.active_command == 'flip_turn':
                    # This is the implementation of the awkward turn. Just 180 yaw and let the robot swim
                    rospy.loginfo('Switching to command flipturn')
                    
                    if self.do_swimmer_turn:
                        self.ac.flip_turn(4,self.speed,self.current_yaw_deg)
                        
                    self.current_yaw_deg = self.current_yaw_deg + 180.0
                    self.ac.set_mode(self.depth_params)
                    # The behaviour is immediate, so can unset active command
                    self.active_command = None 
                elif self.active_command == 'forward_timed':
                    rospy.loginfo('Switching to command forward timed')
                    self.current_pitch_deg = 0.0
                    self.command_endtime = rospy.Time.now() + rospy.Duration(self.forward_swim_time)
                    self.ac.set_mode(self.depth_params)
                elif self.active_command == 'up_to_deck':
                    rospy.loginfo('Switching to command up to deck')
                    self.current_pitch_deg = -90.0
                    self.ac.set_mode(self.angle_params)
                    self.target_depth = self.deck_depth
                elif self.active_command == 'down_to_sea_floor':
                    rospy.loginfo('Switching to command down to sea floor')
                    self.current_pitch_deg = 90.0
                    self.ac.set_mode(self.angle_params)
                    self.target_depth = self.sea_floor_depth
                elif self.active_command == 'down_to_hold':
                    rospy.loginfo('Switching to command down to hold')
                    self.current_pitch_deg = 90.0
                    self.ac.set_mode(self.angle_params)
                    self.target_depth = self.hold_depth
                elif self.active_command == 'park':
                    self.ac.set_mode(self.angle_params)
                    rospy.loginfo('Switching to command park')
                    #self.ac.park()       
                    rospy.loginfo( 'Exiting the autopilot run because a park action is finishing. See you next year!')
                    rospy.signal_shutdown('Tag asked robot to park.')
                else:
                    rospy.loginfo( 'Unreckognized action: ' + str(self.active_command))
        
        try:            
            target_angles_deg = [self.current_roll_deg, self.current_pitch_deg, self.current_yaw_deg ]
            self.ac.send_single_command(self.speed,target_angles_deg,self.target_depth)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print e

if __name__ == '__main__':
    
    rospy.init_node('tag_trajectory_node')
    rospy.loginfo( 'Ship programming starting!')
    
    rospy.loginfo( 'Parameters: \nspeed: ' + str(rospy.get_param('~speed',0.5)) + 
                            '\nforward_time: ' + str(rospy.get_param('~forward_swim_time',5.0)) +
                            '\nfloor_depth: ' + str(rospy.get_param('~sea_floor_depth',5.0)) +
                            '\ndeck_depth: ' + str(rospy.get_param('~deck_depth',2.5)) +
                            '\nhold_depth: ' + str(rospy.get_param('~hold_depth',3.5)) +
                            '\ntag_dead_zone: ' + str(rospy.get_param('~tag_dead_zone_secs',5.0)) +
                            '\nswimmer_turn: ' + str(rospy.get_param('~do_swimmer_turn',False))
                             )
    
    SP = ShipProgram(float(rospy.get_param('~speed',0.5)), float(rospy.get_param('~forward_swim_time',5.0)),
                     float(rospy.get_param('~sea_floor_depth',5.0)), float(rospy.get_param('~deck_depth',2.5)),
                     float(rospy.get_param('~hold_depth',3.5)), float(rospy.get_param('~tag_dead_zone_secs',5.0)),
                     rospy.get_param('~do_swimmer_turn',False) )
    
    rospy.spin()
         
