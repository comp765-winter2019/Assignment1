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


def genCmds(matpath):
  beats_per_minute = 132.0
  beats_per_second = beats_per_minute/60.0
  beats_per_bar = 4.0
  start_time = 0.0
  end_time = 16.0/beats_per_second
  incr_time = 0.02
  
  common_downscale_factor = 1.0
  common_amplitude_deg = common_downscale_factor*20
  common_max_swing_deg = common_downscale_factor*60
  
  cmds = []
  auxiliary = []
  curr_time = start_time
  curr_choreo = None
  prev_bar = 0
  while curr_time <= end_time:
    dt = curr_time - start_time
    curr_beat_float = dt*beats_per_second
    curr_bar = math.floor(curr_beat_float / beats_per_bar) + 1 # 1-indexed

    # Update choreographer
    if prev_bar < curr_bar:
      prev_bar = curr_bar
      curr_bar_start_time = (curr_beat_float - (curr_bar - 1)*beats_per_bar)/beats_per_second

      if curr_bar == 1: # downwards sinusoidal sweep
        # Choreo. settings
        default_cmd = TimedLegState()
        default_cmd.amplitude = common_amplitude_deg*radian
        default_cmd.frequency = beats_per_second
        default_cmd.phase_offset = 0
        default_cmd.start_time = curr_bar_start_time
        sweep_start_angle_deg = -common_max_swing_deg+default_cmd.amplitude/radian
        sweep_end_angle_deg = common_max_swing_deg-default_cmd.amplitude/radian
        sweep_dir = 1
        curr_choreo = SweepLegChoreographer(beats_per_bar, beats_per_minute, curr_bar_start_time, default_cmd, sweep_start_angle_deg, sweep_end_angle_deg, sweep_dir)

      elif curr_bar == 2: # upwards sinusoidal sweep + jump to end angle
        # Choreo. settings
        default_cmd = TimedLegState()
        default_cmd.amplitude = common_amplitude_deg*radian
        default_cmd.frequency = beats_per_second
        default_cmd.phase_offset = 0
        default_cmd.start_time = curr_bar_start_time
        sweep_start_angle_deg = common_max_swing_deg-default_cmd.amplitude/radian
        sweep_end_angle_deg = -common_max_swing_deg+default_cmd.amplitude/radian
        fixed_end_angle_deg = -90
        sweep_dir = -1
        curr_choreo = Sweep5Fixed3LegChoreographer(beats_per_minute, curr_bar_start_time, default_cmd, sweep_start_angle_deg, sweep_end_angle_deg, sweep_dir, fixed_end_angle_deg)

      elif curr_bar == 3: # walk
        # Choreo. settings
        default_cmd = TimedLegState()
        default_cmd.start_time = curr_bar_start_time
        walk_start_angle_deg = -90
        walk_direction = 1
        curr_choreo = Walk112LegChoreographer(beats_per_minute, curr_bar_start_time, default_cmd, walk_start_angle_deg, walk_direction)
  
      elif curr_bar == 4: # Jump following 3-3-2 half-beats rhythm, while oscillating
        # Choreo. settings
        default_cmd = TimedLegState()
        default_cmd.amplitude = common_amplitude_deg*radian
        default_cmd.frequency = beats_per_second
        default_cmd.phase_offset = -math.pi/2
        default_cmd.start_time = curr_bar_start_time
        swing_range_deg = 2*common_max_swing_deg
        swing_incr_deg = swing_range_deg/3
        leg_angles_deg = (-common_max_swing_deg, -common_max_swing_deg + swing_incr_deg, -common_max_swing_deg + 2*swing_incr_deg)
        curr_choreo = Step323LegChoreographer(beats_per_minute, curr_bar_start_time, default_cmd, leg_angles_deg)

      else:
        print 'Found unexpected curr_bar (%f), stopping.' % curr_bar
        curr_time = end_time + 1
        break
    
    # Sample command
    new_cmd = curr_choreo.sample(curr_time)
    if new_cmd is not None:
      cmds.append(new_cmd)
      auxiliary.append((curr_time, curr_beat_float, curr_bar))
    
    curr_time += incr_time
  
  # Save to output
  print 'Sampled %d commands' % len(cmds)
  savemat(matpath, mdict={'cmds': cmds, 'auxiliary': auxiliary, 'end_time': end_time, 'beats_per_minute': beats_per_minute}, oned_as='column')
  print 'Saved to %s' % matpath
  

class Usage(Exception):
  def __init__(self, msg):
    self.msg = msg


def main(argv=None):
  if argv is None:
    argv = sys.argv
  try:
    try:
      opts, args = getopt.getopt(argv[1:], "o:h")
    except getopt.error, msg:
       raise Usage(msg)
    outpath = './sampled_single_leg_choreo.mat'
    for opt in opts:
      if opt[0] == '-o':
        outpath = opt[1]
      elif opt[0] == '-h':
        print 'Usage: %s [-o OUTPUT_FILE]' % argv[0]
        return 0
    
    genCmds(outpath)
    
  except Usage, err:
    print >>sys.stderr, err.msg
    print >>sys.stderr, "for help use --help"
    return 2


if __name__ == "__main__":
  sys.exit(main())
