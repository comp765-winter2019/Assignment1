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
import getopt
import scipy.io


header = '# updateFlexibleSineMotorTargets() END'
footer = '# updateFlexibleSineMotorTargets() END - DONE'


def parseRDAXLog(path):
  entries = dict()

  f = open(path)
  mode = 0 # 0 == scan for header, 1 == scan for content / footer
  count = 0
  for l in f.readlines():
    if mode == 0:
      if l.find(header) >= 0:
        mode = 1
        count += 1
    else:
      if l.find(footer) >= 0:
        mode = 0
      else:
        [key, val] = l.split(':')
        if key not in entries:
          entries[key] = [float('inf')] * (count-1)
        while len(entries[key]) < count - 1:
          entries[key].append(float('inf'))
        entries[key].append(float(val))
  f.close()

  mat_entries = dict()
  for key in entries:
    while(len(entries[key])) < count:
      entries[key].append(float('inf'))
    new_key = key.replace('[', '_').replace(']', '_').replace('.', '_').replace('__', '_')
    mat_entries[new_key] = entries[key]

  dstpath = path + '.mat'
  scipy.io.savemat(dstpath, mat_entries, oned_as='column')
  print 'Saved %d entries to %s' % (count, dstpath)


class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg


def main(argv=None):
  if argv is None:
    argv = sys.argv
  try:
    try:
      opts, args = getopt.getopt(argv[1:], "f:h")
    except getopt.error, msg:
       raise Usage(msg)

    for opt in opts:
      if opt[0] == '-h':
        raise Usage('Usage: %s -f LOG_FILENAME' % argv[0])
      elif opt[0] == '-f':
        filepath = opt[1]

      parseRDAXLog(filepath)

  except Usage, err:
    print >>sys.stderr, err.msg
    print >>sys.stderr, "for help use --help"
    return 2


if __name__ == "__main__":
  sys.exit(main())
