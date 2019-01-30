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
import cPickle as pickle


radian = math.pi/180.0
two_pi = math.pi*2

def angularDist(a, b, maxAngle=360.0):
  d = b - a + maxAngle/2
  if d > 0:
    d = d - math.floor(d/maxAngle)*maxAngle - maxAngle/2
  else:
    d = d - (math.floor(d/maxAngle) + 1)*maxAngle + maxAngle/2
  return abs(d)
  

def wrapAngle(angle, maxAngle=360.0):
  return angle - math.floor(angle/maxAngle)*maxAngle
  

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    reverse = dict((value, key) for key, value in enums.iteritems())
    enums['reverse_mapping'] = reverse
    return type('Enum', (), enums)


DriftPhase = enum('A_SWIM_FORWARD', 'B_ANTICIPATE_RIGHT', 'C_TURN_LEFT', 'D_DRIFT', 'E_TERMINAL')


# TODO: implement linear acceleration mode
# TODO: implement yaw acceleration mode
class PolicyParam:
  def __init__(self, A_forward_rate, A_duration, B_forward_rate, B_yaw_rate, B_duration, C_forward_rate, C_yaw_rate, C_duration, D_duration, E_forward_rate, E_duration):
    self.A_forward_rate = A_forward_rate
    self.A_duration = A_duration
    self.B_forward_rate = B_forward_rate
    self.B_yaw_rate = B_yaw_rate
    self.B_duration = B_duration
    self.C_forward_rate = C_forward_rate
    self.C_yaw_rate = C_yaw_rate
    self.C_duration = C_duration
    self.D_duration = D_duration
    self.E_forward_rate = E_forward_rate
    self.E_duration = E_duration
    
  def __str__(self):
    return 'A.spd=%.2f A.dur=%.2f\nB.spd=%.2f B.yawr=%.2f B.dur=%.2f\nC.spd=%.2f C.yawr=%.2f C.dur=%.2f\nD.dur=%.2f\nE.spd=%.2f E.dur=%.2f' % (self.A_forward_rate, self.A_duration, self.B_forward_rate, self.B_yaw_rate, self.B_duration, self.C_forward_rate, self.C_yaw_rate, self.C_duration, self.D_duration, self.E_forward_rate, self.E_duration)
    
  def __getitem__(self, key):
    if not isinstance(key, int):
      raise TypeError('index into policy params must be a valid positive integer')
    if key == 0:
      return self.A_forward_rate
    elif key == 1:
      return self.A_duration
    elif key == 2:
      return self.B_forward_rate
    elif key == 3:
      return self.B_yaw_rate
    elif key == 4:
      return self.B_duration
    elif key == 5:
      return self.C_forward_rate
    elif key == 6:
      return self.C_yaw_rate
    elif key == 7:
      return self.C_duration
    elif key == 8:
      return self.D_duration
    elif key == 9:
      return self.E_forward_rate
    elif key == 10:
      return self.E_duration
    raise IndexError('policy param index out of range')
    
  def __setitem__(self, key, value):
    if not isinstance(key, int):
      raise TypeError('index into policy params must be a valid positive integer')
    if key == 0:
      self.A_forward_rate = value
    elif key == 1:
      self.A_duration = value
    elif key == 2:
      self.B_forward_rate = value
    elif key == 3:
      self.B_yaw_rate = value
    elif key == 4:
      self.B_duration = value
    elif key == 5:
      self.C_forward_rate = value
    elif key == 6:
      self.C_yaw_rate = value
    elif key == 7:
      self.C_duration = value
    elif key == 8:
      self.D_duration = value
    elif key == 9:
      self.E_forward_rate = value
    elif key == 10:
      self.E_duration = value
    else:
      raise IndexError('policy param index out of range')
    

def genPolicyParamDB(dbfile):
  policies = dict()
  policies['ramius_anqi_pool_init'] = PolicyParam(
    1.0, 5.0,
    1.0, 0.3, 2.0,
    1.0, -0.3, 5.0,
    1.0,
    1.0, 5.0)
  policies['ramius_anqi_pool_opt'] = PolicyParam(
    1.0, 5.0,
    1.0, 0.5, 2.0,
    1.0, -1.0, 5.0,
    10.0,
    1.0, 5.0)
  policies['sim_init'] = PolicyParam(
    1.0, 10.0,
    1.0, 0.1, 3.0,
    1.0, -0.1, 6.0,
    1.0,
    1.0, 10.0)
  policies['sim_opt'] = PolicyParam(
    1.0, 10.0,
    1.0, 0.5151, 3.0,
    1.0, -0.9798, 6.0,
    20.0,
    1.0, 10.0)
  policies['0'] = policies['ramius_anqi_pool_init']
  pickle.dump(policies, open(dbfile, "w"))
  print 'Wrote %d policies to %s' % (len(policies), dbfile)
