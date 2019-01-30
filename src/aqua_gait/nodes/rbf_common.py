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


import h5py
import numpy as np
from drift_common import *


# Sample usages:
'''
default_params = RBFPolicyParams()
default_params.save('default_params.h5')
rand_params = RBFPolicyParams.rand(with_vel=True, num_rbfs=10)
rand_params.save('rand_params.h5')
loaded_params = RBFPolicyParams.load('rand_params.h5')
loaded_params.targets[0][1] += 0.1 # add 0.1 to 2nd target (assuming that there are at least 2 targets)
loaded_params.save('modified_rand_params.h5')
'''
class RBFPolicyParams:
  def __init__(self, T=10.0, angi=1.0, cost_W=[[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 4.0, 0.0], [0.0, 0.0, 0.0, 4.0]], cost_z=[math.pi/2, 0.0, math.sin(math.pi/2), math.cos(math.pi/2)], dt=0.5, hyp_loglengths=[0.0, math.log(0.7), math.log(0.7)], inputs=[[0.0, 0.0], [math.sin(0.0), math.sin(math.pi)], [math.cos(0.0), math.cos(math.pi)]], maxU = 1.2533141373155001, poli = [[2.0], [3.0], [4.0]], state_dims = 2.0, targets=[math.sqrt(math.pi/2), math.sqrt(math.pi/2)]):
    self.T = np.asarray([[T]], dtype='float64')
    self.angi = np.asarray([[angi]], dtype='float64')
    self.cost_W = np.asarray(cost_W, dtype='float64')
    self.cost_z = np.asarray([cost_z], dtype='float64')
    self.dt = np.asarray([[dt]], dtype='float64')
    hyp = hyp_loglengths
    hyp.append(0)
    hyp.append(math.log(0.01))
    self.hyp = np.asarray([hyp], dtype='float64')
    self.inputs = np.asarray(inputs, dtype='float64')
    self.maxU = np.asarray([[maxU]], dtype='float64') # what is this number? close to pi/2.5, but not exact
    self.poli = np.asarray(poli, dtype='float64')
    self.state_dims = np.asarray([[state_dims]], dtype='float64')
    self.targets = np.asarray([targets], dtype='float64')
  
  def save(self, h5filepath):
    f = h5py.File(h5filepath, 'w')
    f.create_dataset("T", data=self.T)
    f.create_dataset("angi", data=self.angi)
    f.create_dataset("cost_W", data=self.cost_W)
    f.create_dataset("cost_z", data=self.cost_z)
    f.create_dataset("dt", data=self.dt)
    f.create_dataset("hyp", data=self.hyp)
    f.create_dataset("inputs", data=self.inputs)
    f.create_dataset("maxU", data=self.maxU)
    f.create_dataset("poli", data=self.poli)
    f.create_dataset("state_dims", data=self.state_dims)
    f.create_dataset("targets", data=self.targets)
    f.close()
  
  def getParamLen(self): # return number of modifiable parameters
    return self.targets.size + self.inputs.size + self.hyp.size - 2

  def getParamRange(self, max_state_vel_radps=math.pi/20, max_policy_ampl_rad=math.pi/2): # return sensible (min, max) ranges for tunable parameter values
    sqrt_max_policy_ampl_rad = math.sqrt(max_policy_ampl_rad)
    if self.inputs.shape[0] == 3:
      minRanges = [-sqrt_max_policy_ampl_rad]*self.targets.size + [-max_state_vel_radps]*self.targets.shape[1] + [-1.0]*self.targets.shape[1] + [-1.0]*self.targets.shape[1] + [math.log(0.01)]*(self.hyp.size-2)
      maxRanges = [sqrt_max_policy_ampl_rad]*self.targets.size + [max_state_vel_radps]*self.targets.shape[1] + [1.0]*self.targets.shape[1] + [1.0]*self.targets.shape[1] + [math.log(1.0)]*(self.hyp.size-2)
      return (minRanges, maxRanges)
    elif self.inputs.shape[0] == 2:
      minRanges = [-sqrt_max_policy_ampl_rad]*self.targets.size + [-1.0]*self.targets.shape[1] + [-1.0]*self.targets.shape[1] + [math.log(0.01)]*(self.hyp.size-2)
      maxRanges = [sqrt_max_policy_ampl_rad]*self.targets.size + [1.0]*self.targets.shape[1] + [1.0]*self.targets.shape[1] + [math.log(1.0)]*(self.hyp.size-2)
      return (minRanges, maxRanges)
    else:
      raise Exception('RBFPolicyParams.inputs has unexpected number of rows: ' + str(self.inputs.shape[0]))
    
  def __getitem__(self, key):
    if not isinstance(key, int):
      raise TypeError('index into RBFPolicyParams must be a valid positive integer')
    if key < 0:
      raise IndexError('RBFPolicyParams index out of range')
    elif key < self.targets.size:
      return self.targets[0][key]
    elif key < self.targets.size + self.inputs.size:
      return self.inputs.flat[key-self.targets.size]
    elif key < self.targets.size + self.inputs.size + self.hyp.size - 2:
      return self.hyp[0][key-self.targets.size-self.inputs.size]
    else:
      raise IndexError('RBFPolicyParams index out of range')
    
  def __setitem__(self, key, value):
    if not isinstance(key, int):
      raise TypeError('index into RBFPolicyParams must be a valid positive integer')
    if key < 0:
      raise IndexError('RBFPolicyParams index out of range')
    elif key < self.targets.size:
      self.targets[0][key] = value
    elif key < self.targets.size + self.inputs.size:
      raise Exception('Cannot set values to self.inputs since we cannot guarantee that sine and cosine of angles are met') # TODO: DO NOT ALLOW TWIDDLING inputs! sine and cosine must be tied together!!!
      self.inputs.flat[key-self.targets.size] = value
    elif key < self.targets.size + self.inputs.size + self.hyp.size - 2:
      self.hyp[0][key-self.targets.size-self.inputs.size] = value
    else:
      raise IndexError('RBFPolicyParams index out of range')

  @staticmethod
  def load(h5filepath):
    f = h5py.File(h5filepath, 'r')
    hyp = f['hyp'][0]
    params = RBFPolicyParams(T=f['T'][0][0], angi=f['angi'][0][0], cost_W=f['cost_W'], cost_z=f['cost_z'][0], dt=f['dt'][0][0], hyp_loglengths=f['hyp'][0][:-2].tolist(), inputs=f['inputs'], poli=f['poli'], state_dims=f['state_dims'][0][0], targets=f['targets'][0])
    f.close()
    return params
  
  @staticmethod
  def rand(with_vel=False, num_rbfs=20, max_state_vel_radps=math.pi/20, max_policy_ampl_rad=math.pi/2):
    if with_vel:
      cost_W = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 4.0, 0.0], [0.0, 0.0, 0.0, 4.0]]
      cost_z = [math.pi/2, 0.0, math.sin(math.pi/2), math.cos(math.pi/2)]
      hyp_loglengths = [0.0, math.log(0.7), math.log(0.7)]
      inputs = [[(random.random()-0.5)*max_state_vel_radps for k in xrange(num_rbfs)],
        [math.sin(float(k)/(num_rbfs-1)*math.pi*2) for k in xrange(num_rbfs)],
        [math.cos(float(k)/(num_rbfs-1)*math.pi*2) for k in xrange(num_rbfs)]]
      targets = [(random.random()-0.5)*max_policy_ampl_rad for k in xrange(num_rbfs)]
    else:
      cost_W = [[0.0, 0.0, 0.0], [0.0, 4.0, 0.0], [0.0, 0.0, 4.0]]
      cost_z = [math.pi/2, math.sin(math.pi/2), math.cos(math.pi/2)]
      hyp_loglengths = [math.log(0.7), math.log(0.7)]
      inputs = [[math.sin(float(k)/(num_rbfs-1)*math.pi*2) for k in xrange(num_rbfs)],
        [math.cos(float(k)/(num_rbfs-1)*math.pi*2) for k in xrange(num_rbfs)]]
      targets = [(random.random()-0.5)*max_policy_ampl_rad for k in xrange(num_rbfs)]
    return RBFPolicyParams(cost_W=costW, cost_z=cost_z, hyp_loglengths=hyp_loglengths, inputs=inputs, targets=targets)
