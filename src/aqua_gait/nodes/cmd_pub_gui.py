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

import sys
from PyQt4 import QtCore, QtGui
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from aquacore.msg import Command


class GUI(QtGui.QWidget):
  def __init__(self):
    super(GUI, self).__init__()
    self.initVars()
    self.initUI()
    self.initConnections()
    
  
  def __del__(self):
    rospy.signal_shutdown("GUI terminated")
    
    
  def initVars(self):
    rospy.init_node('cmd_pub_gui')
    
    rospy.loginfo('Waiting for /aqua/calibrate service...')
    rospy.wait_for_service('/aqua/calibrate')
    rospy.loginfo('Found /aqua/calibrate service')

    self.cmdPub = rospy.Publisher('/aqua/command', Command)
    self.calibrateSrv = rospy.ServiceProxy('/aqua/calibrate', Empty)
    
    self.cmd = Command()
    self.cmdPubTimer = QtCore.QTimer()
    
    
  def initUI(self):
    self.layout = QtGui.QGridLayout()
    
    self.rateLabel = QtGui.QLabel("Hz:")
    self.layout.addWidget(self.rateLabel, 0, 0)
    self.rateText = QtGui.QLineEdit("0")
    self.layout.addWidget(self.rateText, 0, 1)

    self.calibrateButton = QtGui.QPushButton("Calibrate")
    self.layout.addWidget(self.calibrateButton, 0, 2)
    
    self.speedLabel = QtGui.QLabel("Speed:")
    self.layout.addWidget(self.speedLabel, 1, 0)
    self.speedText = QtGui.QLineEdit("0")
    self.layout.addWidget(self.speedText, 2, 0)
    
    self.heaveLabel = QtGui.QLabel("Heave:")
    self.layout.addWidget(self.heaveLabel, 1, 1)
    self.heaveText = QtGui.QLineEdit("0")
    self.layout.addWidget(self.heaveText, 2, 1)

    self.rollLabel = QtGui.QLabel("Roll:")
    self.layout.addWidget(self.rollLabel, 1, 2)
    self.rollText = QtGui.QLineEdit("0")
    self.layout.addWidget(self.rollText, 2, 2)

    self.pitchLabel = QtGui.QLabel("Pitch:")
    self.layout.addWidget(self.pitchLabel, 1, 3)
    self.pitchText = QtGui.QLineEdit("0")
    self.layout.addWidget(self.pitchText, 2, 3)

    self.yawLabel = QtGui.QLabel("Yaw:")
    self.layout.addWidget(self.yawLabel, 1, 4)
    self.yawText = QtGui.QLineEdit("0")
    self.layout.addWidget(self.yawText, 2, 4)

    self.setLayout(self.layout)
    
    self.resize(500, 200)
    
    
  def initConnections(self):
    self.cmdPubTimer.timeout.connect(self.cmdPubOnce)
    self.rateText.textEdited.connect(self.updateRate)
    self.speedText.textEdited.connect(self.updateSpeed)
    self.heaveText.textEdited.connect(self.updateHeave)
    self.rollText.textEdited.connect(self.updateRoll)
    self.pitchText.textEdited.connect(self.updatePitch)
    self.yawText.textEdited.connect(self.updateYaw)
    self.calibrateButton.pressed.connect(self.requestCalibrate)
    
    
  def updateRate(self, newvalstr):
    try:
      newval = float(newvalstr)
    except ValueError:
      newval = 0
    if newval < 0:
      newval = 0
    elif newval > 100:
      newval = 100
    
    self.rateText.setText(str(newval))
    self.cmdPubTimer.stop()
    
    if newval > 0:
      self.cmdPubTimer.start(1000.0/newval)
      
      
  def updateSpeed(self, newvalstr):
    try:
      newval = float(newvalstr)
    except ValueError:
      return
    self.cmd.speed = newval
      
    
  def updateHeave(self, newvalstr):
    try:
      newval = float(newvalstr)
    except ValueError:
      return
    self.cmd.heave = newval
      
    
  def updateRoll(self, newvalstr):
    try:
      newval = float(newvalstr)
    except ValueError:
      return
    self.cmd.roll = newval
      
    
  def updatePitch(self, newvalstr):
    try:
      newval = float(newvalstr)
    except ValueError:
      return
    self.cmd.pitch = newval
      
    
  def updateYaw(self, newvalstr):
    try:
      newval = float(newvalstr)
    except ValueError:
      return
    self.cmd.yaw = newval
      
    
  def cmdPubOnce(self):
    self.cmdPub.publish(self.cmd)
    

  def requestCalibrate(self):
    try:
      self.calibrateSrv()
    except rospy.ServiceException, e:
      print "Calibrate service call failed: %s" % e


if __name__ == '__main__':
  try:
    app = QtGui.QApplication(sys.argv)
    gui = GUI()
    gui.show()
    sys.exit(app.exec_())
  except rospy.ROSInterruptException: pass
