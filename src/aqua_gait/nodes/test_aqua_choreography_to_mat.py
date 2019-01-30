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
  bpm = 132.0 # beats per minute
  bps = bpm/60.0 # beats per second
  spb = 1.0/bps # seconds per beat
  bpbr = 4.0 # beats per bar
  spbr = spb*bpbr # seconds per bar
  start_time = 0.0
  incr_time = 0.02
  
  common_downscale_factor = 0.1
  common_amplitude_deg = common_downscale_factor*20
  common_max_swing_deg = common_downscale_factor*60
  
  cmds = []
  auxiliary = []
  curr_time = start_time
  
  
  # Define per-bar choreographers for each leg
  null_cmd = TimedLegState()
  bar1_start_time = start_time
  bar2_start_time = start_time + spbr
  bar12_beats_cmd = TimedLegState()
  bar12_beats_cmd.amplitude = common_max_swing_deg*radian
  bar12_beats_cmd.frequency = bps/2
  bar12_beats_cmd.phase_offset = -math.pi/2
  bar12_beats_angles = [(-90, 1)]
  bar12_sweep_cmd = TimedLegState()
  bar12_sweep_cmd.amplitude = common_amplitude_deg*radian
  bar12_sweep_cmd.frequency = bps
  bar12_sweep_cmd.phase_offset = 0
  bar12_sweep_deg = -common_max_swing_deg+bar12_sweep_cmd.amplitude/radian
  bar12_sweep_dir = 1
  bar12_choreos = [ # da- da da da-- da | da-- da da can't touch this
    [ # Bar 1
    Walk112LegChoreographer(bpm, bar1_start_time, null_cmd, -90, 1),
    Walk211LegChoreographer(bpm, bar1_start_time, null_cmd, 90, 1),
    MultiStepLegChoreographer(bpbr, bpm, bar1_start_time, bar12_beats_cmd, bar12_beats_angles),
    MultiStepLegChoreographer(bpbr, bpm, bar1_start_time, bar12_beats_cmd, bar12_beats_angles),
    SweepLegChoreographer(bpbr, bpm, bar1_start_time, bar12_sweep_cmd, -bar12_sweep_deg, bar12_sweep_deg, bar12_sweep_dir*-1),
    SweepLegChoreographer(bpbr, bpm, bar1_start_time, bar12_sweep_cmd, bar12_sweep_deg, -bar12_sweep_deg, bar12_sweep_dir),
    ],
    [ # Bar 2
    Walk112LegChoreographer(bpm, bar2_start_time, null_cmd, -90, 1),
    Walk211LegChoreographer(bpm, bar2_start_time, null_cmd, 90, 1),
    MultiStepLegChoreographer(bpbr, bpm, bar2_start_time, bar12_beats_cmd, bar12_beats_angles),
    MultiStepLegChoreographer(bpbr, bpm, bar2_start_time, bar12_beats_cmd, bar12_beats_angles),
    Sweep5Fixed3LegChoreographer(bpm, bar2_start_time, bar12_sweep_cmd, -bar12_sweep_deg*-1, bar12_sweep_deg*-1, bar12_sweep_dir, bar12_sweep_deg),
    Sweep5Fixed3LegChoreographer(bpm, bar2_start_time, bar12_sweep_cmd, -bar12_sweep_deg, bar12_sweep_deg, bar12_sweep_dir*-1, bar12_sweep_deg),
    ],
  ]
    
  choreos = []
  choreos += bar12_choreos
  choreos += copy.deepcopy(bar12_choreos)
  choreos += copy.deepcopy(bar12_choreos)
  choreos += copy.deepcopy(bar12_choreos)
  
  num_bars = len(choreos)
  
  end_time = spb*bpbr*num_bars
  
  prev_bar = 0
  prev_cmds = [TimedLegState(), TimedLegState(), TimedLegState(), TimedLegState(), TimedLegState(), TimedLegState(), ]
  while curr_time <= end_time:
    dt = curr_time - start_time
    curr_beat_float = dt*bps
    curr_bar = math.floor(curr_beat_float / bpbr) + 1 # 1-indexed
    
    bar_idx = int(math.floor(curr_bar-1))
    if bar_idx < 0 or bar_idx >= num_bars:
      print 'ERROR! bar_idx unexpected', bar_idx, num_bars
      break

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
      cmds.append(new_cmds)
      prev_cmds = new_cmds
      auxiliary.append((curr_time, curr_beat_float, curr_bar))
    
    curr_time += incr_time
  
  # Save to output
  print 'Sampled %d commands' % len(cmds)
  savemat(matpath, mdict={'cmds': cmds, 'auxiliary': auxiliary, 'end_time': end_time, 'bpm': bpm}, oned_as='column')
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
    outpath = './sampled_aqua_choreo.mat'
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
