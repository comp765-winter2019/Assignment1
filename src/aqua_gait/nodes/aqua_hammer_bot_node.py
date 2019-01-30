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

from LegChoreographies import *
from scipy.io import savemat
import sys
import getopt
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from aquacore.msg import PeriodicLegCommand
from aquacore.srv import SetGait, IsCalibrated
import pygame
import rospkg


def pubCmds(play):
  bpm = 133 # beats per minute
  bps = bpm/60.0 # beats per second
  spb = 1.0/bps # seconds per beat
  bpbr = 4.0 # beats per bar
  spbr = spb*bpbr # seconds per bar
  incr_time = 0.02
  
  common_downscale_factor = 1.0
  common_amplitude_deg = common_downscale_factor*45.0
  common_max_swing_deg = common_downscale_factor*90.0

  if play:
    hammer_start_offset = spb*8 # start 8 beats in advance
    hammer_start_pos = 174.50 - hammer_start_offset # slightly ahead of actual beat
    rospack = rospkg.RosPack()
    pygame.mixer.init()
    pygame.mixer.music.load(rospack.get_path('aqua_gait') + '/misc/hammer.mp3')
  else:
    hammer_start_offset = 0
  
  cmds = []
  auxiliary = []
  null_cmd = TimedLegState()
  start_time = 0
  choreos = []
  
  # Define choreography for "Can't Touch This" (ctt) section
  if True:
    # Bars 1/2: start really timidly, and also as if swimming
    if True:
      ctt_downscale_factor = 0.15
      ctt_beats_cmd = TimedLegState()
      ctt_beats_cmd.amplitude = ctt_downscale_factor*common_max_swing_deg*radian
      ctt_beats_cmd.frequency = bps/2
      ctt_beats_cmd.phase_offset = -math.pi/2
      ctt_beats_angles = [(-90, 1)]
      ctt_beats_hind_angles = [(0, 1)]
      ctt_beatB_cmd = TimedLegState()
      ctt_beatB_cmd.amplitude = ctt_downscale_factor*common_max_swing_deg*radian
      ctt_beatB_cmd.frequency = bps/2
      ctt_beatB_cmd.phase_offset = math.pi/2
      ctt_beatB_angles = [(-90, 1)]
      ctt_beatB_hind_angles = [(0, 1)]
      ctt_sweep_dir = 1
      ctt_choreos = [ # da- da da da-- da | da-- da da can't touch this
        [ # Bar A
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beatB_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beatB_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beatB_cmd, ctt_beats_hind_angles),
        ],
        [ # Bar B
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beatB_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beatB_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beatB_cmd, ctt_beats_hind_angles),
        ],
      ]
      choreos += ctt_choreos
    # Bars 3/4: increase gutsiness
    if True:
      ctt_downscale_factor = 0.4
      ctt_beats_cmd = TimedLegState()
      ctt_beats_cmd.amplitude = ctt_downscale_factor*common_max_swing_deg*radian
      ctt_beats_cmd.frequency = bps/2
      ctt_beats_cmd.phase_offset = -math.pi/2
      ctt_beatB_cmd = TimedLegState()
      ctt_beatB_cmd.amplitude = ctt_downscale_factor*common_max_swing_deg*radian
      ctt_beatB_cmd.frequency = bps/2
      ctt_beatB_cmd.phase_offset = math.pi/2
      ctt_beats_angles = [(-45, 1)]
      ctt_sweep_cmd = TimedLegState()
      ctt_sweep_cmd.amplitude = ctt_downscale_factor*common_amplitude_deg*radian
      ctt_sweep_cmd.frequency = bps
      ctt_sweep_cmd.phase_offset = 0
      ctt_sweepB_cmd = TimedLegState()
      ctt_sweepB_cmd.amplitude = ctt_downscale_factor*common_amplitude_deg*radian
      ctt_sweepB_cmd.frequency = bps
      ctt_sweepB_cmd.phase_offset = 180
      ctt_sweep_deg = -ctt_downscale_factor*common_max_swing_deg+ctt_sweep_cmd.amplitude/radian
      ctt_sweep_dir = 1
      ctt_choreos = [ # da- da da da-- da | da-- da da can't touch this
        [ # Bar A
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beatB_cmd, ctt_beats_angles),
        SweepLegChoreographer(bpbr, bpm, start_time, ctt_sweep_cmd, -ctt_sweep_deg, ctt_sweep_deg, ctt_sweep_dir*-1),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beatB_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_angles),
        SweepLegChoreographer(bpbr, bpm, start_time, ctt_sweepB_cmd, ctt_sweep_deg, -ctt_sweep_deg, ctt_sweep_dir),
        ],
        [ # Bar B
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beatB_cmd, ctt_beats_angles),
        Sweep5Fixed3LegChoreographer(bpm, start_time, ctt_sweep_cmd, -ctt_sweep_deg*-1, ctt_sweep_deg*-1, ctt_sweep_dir, ctt_sweep_deg),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beatB_cmd, ctt_beats_hind_angles),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_angles),
        Sweep5Fixed3LegChoreographer(bpm, start_time, ctt_sweepB_cmd, -ctt_sweep_deg, ctt_sweep_deg, ctt_sweep_dir*-1, ctt_sweep_deg),
        ],
      ]
      choreos += ctt_choreos
    # Bars 5/6: the full thing
    if True:
      ctt_downscale_factor = 1.0
      ctt_beats_cmd = TimedLegState()
      ctt_beats_cmd.amplitude = ctt_downscale_factor*common_max_swing_deg*radian
      ctt_beats_cmd.frequency = bps/2
      ctt_beats_cmd.phase_offset = -math.pi/2
      ctt_beats_angles = [(-90, 1)]
      ctt_sweep_cmd = TimedLegState()
      ctt_sweep_cmd.amplitude = ctt_downscale_factor*common_amplitude_deg*radian
      ctt_sweep_cmd.frequency = bps
      ctt_sweep_cmd.phase_offset = 0
      ctt_sweep_deg = -ctt_downscale_factor*common_max_swing_deg+ctt_sweep_cmd.amplitude/radian
      ctt_sweep_dir = 1
      ctt_choreos = [ # da- da da da-- da | da-- da da can't touch this
        [ # Bar A
        Walk112LegChoreographer(bpm, start_time, null_cmd, -90, 1),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_angles),
        SweepLegChoreographer(bpbr, bpm, start_time, ctt_sweep_cmd, -ctt_sweep_deg, ctt_sweep_deg, ctt_sweep_dir*-1),
        Walk211LegChoreographer(bpm, start_time, null_cmd, -90, 1),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_angles),
        SweepLegChoreographer(bpbr, bpm, start_time, ctt_sweep_cmd, ctt_sweep_deg, -ctt_sweep_deg, ctt_sweep_dir),
        ],
        [ # Bar B
        Walk112LegChoreographer(bpm, start_time, null_cmd, -90, 1),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_angles),
        Sweep5Fixed3LegChoreographer(bpm, start_time, ctt_sweep_cmd, -ctt_sweep_deg*-1, ctt_sweep_deg*-1, ctt_sweep_dir, ctt_sweep_deg),
        Walk211LegChoreographer(bpm, start_time, null_cmd, -90, 1),
        MultiStepLegChoreographer(bpbr, bpm, start_time, ctt_beats_cmd, ctt_beats_angles),
        Sweep5Fixed3LegChoreographer(bpm, start_time, ctt_sweep_cmd, -ctt_sweep_deg, ctt_sweep_deg, ctt_sweep_dir*-1, ctt_sweep_deg),
        ],
      ]
      choreos += ctt_choreos
    # Bars 7/8: the full thing, double-time!
    if True:
      ctt_downscale_factor = 1.0
      ctt_beats_cmd = TimedLegState()
      ctt_beats_cmd.amplitude = ctt_downscale_factor*common_max_swing_deg*radian
      ctt_beats_cmd.frequency = bps/2
      ctt_beats_cmd.phase_offset = -math.pi/2
      ctt_beats_angles = [(-90, 1)]
      ctt_sweep_cmd = TimedLegState()
      ctt_sweep_cmd.amplitude = ctt_downscale_factor*common_amplitude_deg*radian
      ctt_sweep_cmd.frequency = bps
      ctt_sweep_cmd.phase_offset = 0
      ctt_sweep_deg = -ctt_downscale_factor*common_max_swing_deg+ctt_sweep_cmd.amplitude/radian
      ctt_sweep_dir = 1
      ctt_choreos = [ # da- da da da-- da | da-- da da can't touch this
        [ # Bar A
        Walk112LegChoreographer(bpm, start_time, null_cmd, -90, 1, True),
        Walk211LegChoreographer(bpm, start_time, null_cmd, -90, 1, True),
        SweepLegChoreographer(bpbr, bpm, start_time, ctt_sweep_cmd, -ctt_sweep_deg, ctt_sweep_deg, ctt_sweep_dir*-1),
        Walk211LegChoreographer(bpm, start_time, null_cmd, -90, 1, True),
        Walk112LegChoreographer(bpm, start_time, null_cmd, -90, 1, True),
        SweepLegChoreographer(bpbr, bpm, start_time, ctt_sweep_cmd, ctt_sweep_deg, -ctt_sweep_deg, ctt_sweep_dir),
        ],
        [ # Bar B
        Walk112LegChoreographer(bpm, start_time, null_cmd, -90, 1, True),
        Walk211LegChoreographer(bpm, start_time, null_cmd, -90, 1, True),
        Sweep5Fixed3LegChoreographer(bpm, start_time, ctt_sweep_cmd, -ctt_sweep_deg*-1, ctt_sweep_deg*-1, ctt_sweep_dir, ctt_sweep_deg),
        Walk211LegChoreographer(bpm, start_time, null_cmd, -90, 1, True),
        Walk112LegChoreographer(bpm, start_time, null_cmd, -90, 1, True),
        Sweep5Fixed3LegChoreographer(bpm, start_time, ctt_sweep_cmd, -ctt_sweep_deg, ctt_sweep_deg, ctt_sweep_dir*-1, ctt_sweep_deg),
        ],
      ]
      choreos += ctt_choreos

  # Define choreography for "Stop, Hammer Time" (sht) section
  if True:
    # Bars 1-4: match to the beat
    if True:
      pretime = 0.01
      sht_chorus_cmd = TimedLegState()
      sht_chorus_cmd.amplitude = 0
      sht_chorus_cmd.frequency = 0
      sht_chorus_cmd.phase_offset = -math.pi/2
      sht_chorus_angles_A = [(-45.0+i*90.0, i+1-pretime) for i in xrange(4)]
      sht_chorus_angles_B = [(270.0-i*90.0, i+1-pretime) for i in xrange(4)]
      sht_chorus_angles_C = [(-90, 1-pretime), (45, 4-pretime)]
      sht_chorus_angles_D = [(-90, 1-pretime)]
      sht_323_angles_left = [-common_max_swing_deg+(2*common_max_swing_deg)/3.0*i for i in xrange(3)]
      sht_323_angles_right = [common_max_swing_deg-(2*common_max_swing_deg)/3.0*i for i in xrange(3)]
      sht_323_angles_left_hind = [-common_max_swing_deg-(2*common_max_swing_deg)/3.0*i for i in xrange(3)]
      sht_323_angles_right_hind = [common_max_swing_deg+(2*common_max_swing_deg)/3.0*i for i in xrange(3)]
      sht_35_angles_left = [90.0, 0.0]
      sht_35_angles_right = [-90.0, 0.0]
      sht_35_angles_left_hind = [90.0, 180.0]
      sht_35_angles_right_hind = [-90.0, 180.0]
      ctt_choreos = [
        [ # Bar A # oh- oh- oh- oh-
          Step323LegChoreographer(bpm, start_time, null_cmd, sht_323_angles_left_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, null_cmd, sht_chorus_angles_A),
          Step323LegChoreographer(bpm, start_time, null_cmd, sht_323_angles_left),
          Step323LegChoreographer(bpm, start_time, null_cmd, sht_323_angles_right_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, null_cmd, sht_chorus_angles_A),
          Step323LegChoreographer(bpm, start_time, null_cmd, sht_323_angles_right),
        ],
        [ # Bar B # oh- oh- oh- oh-
          Step35LegChoreographer(bpm, start_time, null_cmd, sht_35_angles_left_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, null_cmd, sht_chorus_angles_B),
          Step35LegChoreographer(bpm, start_time, null_cmd, sht_35_angles_left),
          Step35LegChoreographer(bpm, start_time, null_cmd, sht_35_angles_right_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, null_cmd, sht_chorus_angles_B),
          Step35LegChoreographer(bpm, start_time, null_cmd, sht_35_angles_right),
        ],
        [ # Bar C # oh- -- -- oh-
          Step323LegChoreographer(bpm, start_time, null_cmd, sht_323_angles_left_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, sht_chorus_cmd, sht_chorus_angles_C),
          Step323LegChoreographer(bpm, start_time, null_cmd, sht_323_angles_left),
          Step323LegChoreographer(bpm, start_time, null_cmd, sht_323_angles_right_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, sht_chorus_cmd, sht_chorus_angles_C),
          Step323LegChoreographer(bpm, start_time, null_cmd, sht_323_angles_right),
        ],
        [ # Bar D # oh- -- -- --
          Step35LegChoreographer(bpm, start_time, sht_chorus_cmd, sht_35_angles_left_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, sht_chorus_cmd, sht_chorus_angles_D),
          Step35LegChoreographer(bpm, start_time, sht_chorus_cmd, sht_35_angles_left),
          Step35LegChoreographer(bpm, start_time, sht_chorus_cmd, sht_35_angles_right_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, sht_chorus_cmd, sht_chorus_angles_D),
          Step35LegChoreographer(bpm, start_time, sht_chorus_cmd, sht_35_angles_right),
        ],
      ]
      choreos += ctt_choreos
    # Bars 5-8: double freq + Stop! Hammer Time
    if True:
      pretime = 0.01
      sht_chorus_angles_A = [(-45.0+i*90.0, i+1-pretime) for i in xrange(8)]
      sht_chorus_angles_B = [(270.0-i*90.0, i+1-pretime) for i in xrange(8)]
      sht_spasm_cmd = TimedLegState()
      sht_spasm_cmd.amplitude = 0.2*common_max_swing_deg*radian
      sht_spasm_cmd.frequency = bps*3.0/2.0
      sht_spasm_cmd.phase_offset = -math.pi/2.0
      sht_spasmB_cmd = TimedLegState()
      sht_spasmB_cmd.amplitude = 0.2*common_max_swing_deg*radian
      sht_spasmB_cmd.frequency = bps*3.0/2.0
      sht_spasmB_cmd.phase_offset = math.pi/2.0
      sht_323_angles_left = [-common_max_swing_deg+(2*common_max_swing_deg)/3.0*i for i in xrange(3)]
      sht_323_angles_right = [common_max_swing_deg-(2*common_max_swing_deg)/3.0*i for i in xrange(3)]
      sht_323_angles_left_hind = [-common_max_swing_deg-(2*common_max_swing_deg)/3.0*i for i in xrange(3)]
      sht_323_angles_right_hind = [common_max_swing_deg+(2*common_max_swing_deg)/3.0*i for i in xrange(3)]
      sht_35_angles_left = [90.0, 0.0]
      sht_35_angles_right = [-90.0, 0.0]
      sht_35_angles_left_hind = [90.0, 180.0]
      sht_35_angles_right_hind = [-90.0, 180.0]
      ctt_choreos = [
        [ # Bar A # oh- oh- oh- oh-
          Step323LegChoreographer(bpm, start_time, sht_spasm_cmd, sht_323_angles_left_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, sht_spasmB_cmd, sht_chorus_angles_A),
          Step323LegChoreographer(bpm, start_time, sht_spasm_cmd, sht_323_angles_left),
          Step323LegChoreographer(bpm, start_time, sht_spasmB_cmd, sht_323_angles_right_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, sht_spasm_cmd, sht_chorus_angles_A),
          Step323LegChoreographer(bpm, start_time, sht_spasmB_cmd, sht_323_angles_right),
        ],
        [ # Bar B # oh- oh- oh- oh-
          Step35LegChoreographer(bpm, start_time, sht_spasm_cmd, sht_35_angles_left_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, sht_spasmB_cmd, sht_chorus_angles_B),
          Step35LegChoreographer(bpm, start_time, sht_spasm_cmd, sht_35_angles_left),
          Step35LegChoreographer(bpm, start_time, sht_spasmB_cmd, sht_35_angles_right_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, sht_spasm_cmd, sht_chorus_angles_B),
          Step35LegChoreographer(bpm, start_time, sht_spasmB_cmd, sht_35_angles_right),
        ],
        [ # Bar C # oh- -- -- oh-
          Step323LegChoreographer(bpm, start_time, sht_spasm_cmd, sht_323_angles_left_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, sht_spasmB_cmd, sht_chorus_angles_A),
          Step323LegChoreographer(bpm, start_time, sht_spasm_cmd, sht_323_angles_left),
          Step323LegChoreographer(bpm, start_time, sht_spasmB_cmd, sht_323_angles_right_hind),
          MultiStepLegChoreographer(bpbr, bpm, start_time, sht_spasm_cmd, sht_chorus_angles_A),
          Step323LegChoreographer(bpm, start_time, sht_spasmB_cmd, sht_323_angles_right),
        ],
        [ # Bar D # oh- Stop! Hammer Time!
          Step2Stop6LegChoreographer(bpm, start_time, sht_spasm_cmd, sht_chorus_angles_B[1][0], -45),
          Step2Stop6LegChoreographer(bpm, start_time, sht_spasmB_cmd, sht_chorus_angles_B[0][0], -135),
          Step2Stop6LegChoreographer(bpm, start_time, sht_spasm_cmd, sht_35_angles_left[0], -30),
          Step2Stop6LegChoreographer(bpm, start_time, sht_spasmB_cmd, sht_chorus_angles_B[1][0], -45),
          Step2Stop6LegChoreographer(bpm, start_time, sht_spasm_cmd, sht_chorus_angles_B[0][0], -135),
          Step2Stop6LegChoreographer(bpm, start_time, sht_spasmB_cmd, sht_35_angles_right[0], 30),
        ],
      ]
      choreos += ctt_choreos

  
  num_bars = len(choreos)
  
  end_time = spb*bpbr*num_bars
  
  prev_bar = 0
  prev_cmds = [TimedLegState(), TimedLegState(), TimedLegState(), TimedLegState(), TimedLegState(), TimedLegState(),]

  rospy.init_node('aqua_hammer_bot_node')
  
  rospy.loginfo('Waiting for /aqua/is_calibrated...')
  rospy.wait_for_service('/aqua/is_calibrated')
  cln_is_calibrated = rospy.ServiceProxy('/aqua/is_calibrated', IsCalibrated)
  res = cln_is_calibrated()

  if not res.value:
    rospy.loginfo('Waiting for /aqua/calibrate...')
    rospy.wait_for_service('/aqua/calibrate')
    cln_calibrate = rospy.ServiceProxy('/aqua/calibrate', Empty)
    res = cln_calibrate()

  rospy.loginfo('Waiting for /aqua/set_gait...')
  rospy.wait_for_service('/aqua/set_gait')
  cln_set_gait = rospy.ServiceProxy('/aqua/set_gait', SetGait)
  res = cln_set_gait('flexible-sine')
  
  cmd_pub = rospy.Publisher('/aqua/periodic_leg_command', PeriodicLegCommand, queue_size=1)
  if not play:
    r = rospy.Rate(50)

  rospy.loginfo('LET\'S GO!!!')

  prev_beat_int = int(-1)
  ros_start_time = rospy.Time.now()
  if play:
    pygame.mixer.music.play(0, hammer_start_pos)
  while not rospy.is_shutdown() and ((not play) or (pygame.mixer.music.get_busy() == True)):
    curr_time = (rospy.Time.now() - ros_start_time).to_sec()
    if curr_time < hammer_start_offset:
      continue
    curr_time -= hammer_start_offset
    curr_beat_float = curr_time*bps
    curr_bar = math.floor(curr_beat_float / bpbr) + 1 # 1-indexed
    curr_beat_int = int(curr_beat_float)
    
    bar_idx = int(math.floor(curr_bar-1))
    if bar_idx < 0 or bar_idx >= num_bars:
      #print 'ERROR! bar_idx unexpected', bar_idx, num_bars
      #break
      print bar_idx
      continue
    if curr_beat_int > prev_beat_int:
      rospy.loginfo('Bar %d Beat %d' % (curr_bar, (curr_beat_int % bpbr)+1))
      prev_beat_int = curr_beat_int

    new_cmds = [None, None, None, None, None, None]
    has_new_cmd = False
    for leg_i in xrange(6):
      new_cmd = choreos[bar_idx][leg_i].sample(curr_time)
      if new_cmd is not None:
        new_cmds[leg_i] = new_cmd
        has_new_cmd = True
    
    if has_new_cmd:
      for leg_i in xrange(6):
        if new_cmds[leg_i] is None:
          new_cmds[leg_i] = copy.deepcopy(prev_cmds[leg_i])
      #cmds.append(new_cmds)
      plc = PeriodicLegCommand()
      plc.header.stamp = rospy.Time.now()
      plc.header.frame_id = '/aqua_base'
      for leg_i in xrange(6):
        plc.amplitudes[leg_i] = new_cmds[leg_i].amplitude
        plc.frequencies[leg_i] = new_cmds[leg_i].frequency
        plc.phase_offsets[leg_i] = new_cmds[leg_i].phase_offset
        plc.leg_offsets[leg_i] = new_cmds[leg_i].leg_offset
        plc.leg_velocities[leg_i] = 0
      cmd_pub.publish(plc)
      prev_cmds = new_cmds
    
    if not play:
      r.sleep()


class Usage(Exception):
  def __init__(self, msg):
    self.msg = msg


def main(argv=None):
  if argv is None:
    argv = sys.argv
  try:
    try:
      opts, args = getopt.getopt(argv[1:], "hp")
    except getopt.error, msg:
       raise Usage(msg)
    play = False
    for opt in opts:
      if opt[0] == '-p':
        play = True
    
    pubCmds(play)
    
  except rospy.ROSInterruptException:
    return 0
    
  except Usage, err:
    print >>sys.stderr, err.msg
    print >>sys.stderr, "for help use --help"
    return 2


if __name__ == "__main__":
  sys.exit(main())
