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
import copy


radian = math.pi/180.0


def wrapAngle(angle, angleRange = 360.0):
  return (angle - math.floor(angle/angleRange)*angleRange)


def angularMag(b, a, angleRange = 360.0):
  d = b - a + angleRange/2
  if d > 0:
    d = d - math.floor(d/angleRange)*angleRange - angleRange/2
  else:
    d = d - (math.floor(d/angleRange) + 1)*angleRange + angleRange/2
  return d


def angularDist(b, a, angleRange = 360.0):
  return abs(angularMag(b, a, angleRange))


class TimedLegState:
  def __init__(self, amplitude_rad=20.0*radian, frequency_hz = 4.0, phase_offset_rad=0.0, leg_offset_rad=0.0, start_time=0.0):
    self.amplitude = amplitude_rad
    self.frequency = frequency_hz
    self.phase_offset = phase_offset_rad
    self.leg_offset = leg_offset_rad
    self.start_time = start_time
    
  def __str__(self):
    return 'ampl: %.2f deg, freq: %.2f Hz, phase_off: %.2f deg, leg_off: %.2f deg, start_t: %.3f sec' % (self.amplitude/radian, self.frequency, self.phase_offset/radian, self.leg_offset/radian, self.start_time)
    
  
class BaseLegChoreographer:
  def __init__(self, beats_per_bar=4, beats_per_minute=60.0, start_time=0.0, default_cmd=TimedLegState()):
    self.beats_per_bar = float(beats_per_bar) - 1e-15 # account for numerical inacuracies of sampling by slightly decreasing this measure
    self.bps = float(beats_per_minute)/60
    self.start_time = start_time
    self.default_cmd = copy.deepcopy(default_cmd)
    
  def sample(self, time):
    print 'BaseLegChoreographer.sample(' + str(time) + ') called'
    return None
    
    
# Sweeps leg offset across range
class SweepLegChoreographer(BaseLegChoreographer):
  def __init__(self, beats_per_bar=4, beats_per_minute=60.0, start_time=0.0, default_cmd=TimedLegState(), sweep_start_angle_deg=0.0, sweep_end_angle_deg = 90, sweep_dir = 1):
    BaseLegChoreographer.__init__(self, beats_per_bar, beats_per_minute, start_time, default_cmd)
    self.sweep_start_angle_rad = float(sweep_start_angle_deg)*radian
    self.sweep_end_angle_rad = float(sweep_end_angle_deg)*radian
    if sweep_dir != -1:
      self.sweep_dir = 1
    else:
      self.sweep_dir = -1

  def sample(self, time):
    if time < self.start_time:
      return None
    
    dt = time - self.start_time
    curr_beat_float = dt*self.bps
    curr_beat = (curr_beat_float % self.beats_per_bar) + 1 # 1-indexed, in range [1, 1 + self.beats_per_bar)
    
    curr_sweep_ratio = (curr_beat - 1)/self.beats_per_bar
    
    sweep_range_rad = angularMag(self.sweep_end_angle_rad, self.sweep_start_angle_rad, 2*math.pi)
    if self.sweep_dir == 1:
      if sweep_range_rad < 0:
        sweep_range_rad = 2*math.pi + sweep_range_rad
    else: # self.sweep_dir == -1
      if sweep_range_rad > 0:
        sweep_range_rad = sweep_range_rad - 2*math.pi
    
    new_cmd = copy.deepcopy(self.default_cmd)
    new_cmd.leg_offset = wrapAngle(curr_sweep_ratio*sweep_range_rad + self.sweep_start_angle_rad, 2*math.pi)
    new_cmd.start_time = time

    return new_cmd


# Sweeps leg offset across range on a 4-beats-per-bar rhythm,
# but on 6th half-beat (i.e. beat 3.5) switches over to a fixed leg offset
class Sweep5Fixed3LegChoreographer(BaseLegChoreographer):
  def __init__(self, beats_per_minute=60.0, start_time=0.0, default_cmd=TimedLegState(), sweep_start_angle_deg = 0, sweep_end_angle_deg = 90, sweep_dir = 1, fixed_end_angle_deg = 0):
    BaseLegChoreographer.__init__(self, 4, beats_per_minute, start_time, default_cmd)
    self.sweep_start_angle_rad = float(sweep_start_angle_deg)*radian
    self.sweep_end_angle_rad = float(sweep_end_angle_deg)*radian
    self.fixed_end_angle_rad = float(fixed_end_angle_deg)*radian
    if sweep_dir != -1:
      self.sweep_dir = 1
    else:
      self.sweep_dir = -1


  def sample(self, time):
    if time < self.start_time:
      return None
    
    dt = time - self.start_time
    curr_beat_float = dt*self.bps
    curr_beat = (curr_beat_float % self.beats_per_bar) + 1 # 1-indexed, in range [1, 1 + self.beats_per_bar)
    
    if curr_beat < 3.5: # sweep mode
      curr_sweep_ratio = (curr_beat - 1)/self.beats_per_bar
      
      sweep_range_rad = angularMag(self.sweep_end_angle_rad, self.sweep_start_angle_rad, 2*math.pi)
      if self.sweep_dir == 1:
        if sweep_range_rad < 0:
          sweep_range_rad = 2*math.pi + sweep_range_rad
      else: # self.sweep_dir == -1
        if sweep_range_rad > 0:
          sweep_range_rad = sweep_range_rad - 2*math.pi
      
      new_cmd = copy.deepcopy(self.default_cmd)
      new_cmd.leg_offset = wrapAngle(curr_sweep_ratio*sweep_range_rad + self.sweep_start_angle_rad, 2*math.pi)
      new_cmd.start_time = time
      return new_cmd
      
    else: # jump to final angle
      new_cmd = copy.deepcopy(self.default_cmd)
      new_cmd.amplitude = 0
      new_cmd.frequency = 0
      new_cmd.phase_offset = 0
      new_cmd.leg_offset = wrapAngle(self.fixed_end_angle_rad, 2*math.pi)
      new_cmd.start_time = time
      return new_cmd # TODO: ambiguous leg change: jump might go to target leg offset in wrong direction
    
    
# Implements one or more fixed leg offset targets
# NOTE: beat_i is 1-indexed
class MultiStepLegChoreographer(BaseLegChoreographer):
  def __init__(self, beats_per_bar=4, beats_per_minute=60.0, start_time=0.0, default_cmd=TimedLegState(), angle_deg_beat_i_pairs=[(0, 1)]):
    BaseLegChoreographer.__init__(self, beats_per_bar, beats_per_minute, start_time, default_cmd)
    if angle_deg_beat_i_pairs is None or len(angle_deg_beat_i_pairs) == 0:
      self.angle_deg_beat_i_pairs = [(0, 1)]
    else:
      self.angle_deg_beat_i_pairs = copy.deepcopy(angle_deg_beat_i_pairs)


  def sample(self, time):
    if time < self.start_time:
      return None
    
    dt = float(time) - self.start_time
    curr_beat_float = dt*self.bps
    curr_beat = (curr_beat_float % self.beats_per_bar) + 1 # 1-indexed, in range [1, 1 + self.beats_per_bar)
    
    # Check to see if a new command needs to be issued
    new_angle_deg_beat_i = None
    while len(self.angle_deg_beat_i_pairs) > 0:
      if self.angle_deg_beat_i_pairs[0][1] <= curr_beat:
        new_angle_deg_beat_i = self.angle_deg_beat_i_pairs.pop(0)
      else:
        break
        
    if new_angle_deg_beat_i is not None:
      new_cmd = copy.deepcopy(self.default_cmd)
      new_cmd.leg_offset = wrapAngle(new_angle_deg_beat_i[0]*radian, 2*math.pi)
      new_cmd.start_time = time
      return new_cmd
    else:
      return None
      

# Jumps to angle for 1 beat, then stops at end angle starting from 2nd beat,
# for a 4-beats-per-bar rhythm
class Step2Stop6LegChoreographer(BaseLegChoreographer):
  def __init__(self, beats_per_minute=60.0, start_time=0.0, default_cmd=TimedLegState(), step_angle_deg=0, stop_angle_deg=0):
    BaseLegChoreographer.__init__(self, 4, beats_per_minute, start_time, default_cmd)
    self.step_angle_rad = step_angle_deg*radian
    self.stop_angle_rad = stop_angle_deg*radian
    self.issued_step = False
    self.issued_stop = False


  def sample(self, time):
    if time < self.start_time:
      return None
    
    dt = float(time) - self.start_time
    curr_beat_float = dt*self.bps
    curr_beat = (curr_beat_float % self.beats_per_bar) + 1 # 1-indexed, in range [1, 1 + self.beats_per_bar)
    
    if curr_beat < 2: # Jump mode
      if not self.issued_step:
        new_cmd = copy.deepcopy(self.default_cmd)
        new_cmd.leg_offset = wrapAngle(self.step_angle_rad, 2*math.pi)
        new_cmd.start_time = time
        self.issued_step = True
        return new_cmd
      else:
        return None
    else: # Stop mode
      if not self.issued_stop:
        new_cmd = TimedLegState()
        new_cmd.amplitude = 0
        new_cmd.frequency = 0
        new_cmd.phase_offset = 0
        new_cmd.leg_offset = wrapAngle(self.stop_angle_rad, 2*math.pi)
        new_cmd.start_time = time
        self.issued_stop = True
        return new_cmd
      else:
        return None
      

# Jumps ++140' on beat 1, ++90' on beat 2, and ++130' on beat 3
# on a 4-beats-per-bar rhythm
class Walk112LegChoreographer(MultiStepLegChoreographer):
  def __init__(self, beats_per_minute=60.0, start_time=0.0, default_cmd=TimedLegState(), walk_start_angle_deg=-90, walk_direction=1, double_time=False):
    if walk_direction != -1:
      walk_direction = 1
    default_cmd.amplitude = 0
    default_cmd.frequency = 0
    default_cmd.phase_offset = 0
    default_cmd.leg_offset = walk_start_angle_deg
    if double_time:
      angle_deg_beat_i_pairs = [(walk_start_angle_deg+walk_direction*140, 1.0), (walk_start_angle_deg+walk_direction*(140+90), 1.5), (walk_start_angle_deg, 2.0), (walk_start_angle_deg+walk_direction*140, 3.0), (walk_start_angle_deg+walk_direction*(140+90), 3.5), (walk_start_angle_deg, 4.0)]
    else:
      angle_deg_beat_i_pairs = [(walk_start_angle_deg+walk_direction*140, 1.0), (walk_start_angle_deg+walk_direction*(140+90), 2.0), (walk_start_angle_deg, 3.0)]
    MultiStepLegChoreographer.__init__(self, 4, beats_per_minute, start_time, default_cmd, angle_deg_beat_i_pairs)


# Jumps ++140+90' offset, ++130' on beat 1, ++140' on beat 3, and ++90' on beat 4
# on a 4-beats-per-bar rhythm
#
# This is the off-phase version of Walk112LegChoreographer
class Walk211LegChoreographer(MultiStepLegChoreographer):
  def __init__(self, beats_per_minute=60.0, start_time=0.0, default_cmd=TimedLegState(), walk_start_angle_deg=-90, walk_direction=1, double_time=False):
    if walk_direction != -1:
      walk_direction = 1
    default_cmd.amplitude = 0
    default_cmd.frequency = 0
    default_cmd.phase_offset = 0
    default_cmd.leg_offset = walk_start_angle_deg
    if double_time:
      angle_deg_beat_i_pairs = [(walk_start_angle_deg, 1.0), (walk_start_angle_deg+walk_direction*(140), 2.0), (walk_start_angle_deg+walk_direction*(140+90), 2.5), (walk_start_angle_deg, 3.0), (walk_start_angle_deg+walk_direction*(140), 4.0), (walk_start_angle_deg+walk_direction*(140+90), 4.5)]
    else:
      angle_deg_beat_i_pairs = [(walk_start_angle_deg, 1.0), (walk_start_angle_deg+walk_direction*(140), 3.0), (walk_start_angle_deg+walk_direction*(140+90), 4.0)]
    MultiStepLegChoreographer.__init__(self, 4, beats_per_minute, start_time, default_cmd, angle_deg_beat_i_pairs)


# Sweeps to 3 different leg offsets on a 4-beats-per-bar rhythm,
# at beat 1, beat 2.5, and beat 3.5
class Step323LegChoreographer(MultiStepLegChoreographer):
  def __init__(self, beats_per_minute=60.0, start_time=0.0, default_cmd=TimedLegState(), leg_angles_deg=(0, 45, 90)):
    if len(leg_angles_deg) != 3:
      raise Exception("Step323LegChoreographer expects len(leg_angles_deg) == 3")
    
    angle_deg_beat_i_pairs = [(leg_angles_deg[0], 1), (leg_angles_deg[1], 2.5), (leg_angles_deg[2], 3.5)]
    MultiStepLegChoreographer.__init__(self, 4, beats_per_minute, start_time, default_cmd, angle_deg_beat_i_pairs)


# Sweeps to 2 different leg offsets on a 4-beats-per-bar rhythm,
# at beat 1 and beat 2.5
class Step35LegChoreographer(MultiStepLegChoreographer):
  def __init__(self, beats_per_minute=60.0, start_time=0.0, default_cmd=TimedLegState(), leg_angles_deg=(0, 90)):
    if len(leg_angles_deg) != 2:
      raise Exception("Step35LegChoreographer expects len(leg_angles_deg) == 2")
    angle_deg_beat_i_pairs = [(leg_angles_deg[0], 1), (leg_angles_deg[1], 2.5)]
    MultiStepLegChoreographer.__init__(self, 4, beats_per_minute, start_time, default_cmd, angle_deg_beat_i_pairs)
