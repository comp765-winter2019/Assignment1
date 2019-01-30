/*******************************************************************************
* DO NOT MODIFY - AUTO-GENERATED
* 
* Copyright (c) 2016, McGill University / Independent Robotics Inc.
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* 
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include "aqua_gait/Gaits.hpp"
#include <ros/ros.h>
#include <chrono>


using namespace std;


int main(int argc, char** argv) {
  float speed_cmd = 1.0;
  float heave_cmd = 0.0;
  float roll_cmd = 0.0;
  float pitch_cmd = 1.0;
  float yaw_cmd = 0.0;

#define SHOW_MOTOR_TARGETS \
  ROS_INFO("- target poses:\n  0: %.4f\n  1: %.4f\n  2: %.4f\n  3: %.4f\n  4: %.4f\n  5: %.4f\n", \
      motorTargets[0].pos, motorTargets[1].pos, motorTargets[2].pos, motorTargets[3].pos, motorTargets[4].pos, motorTargets[5].pos)

  ros::init(argc, argv, "test_gaits");
  ros::NodeHandle nh;
  MotorTarget_t motorTargets[6];
  ros::Rate hz(1000);

  ROS_INFO_STREAM("Creating gait object");
  UnderwaterSwimmerGait gait;
  gait.activate();

  ROS_INFO_STREAM("Issuing gait command");
  gait.setSpeedCmd(speed_cmd);
  gait.heaveControl(heave_cmd);
  gait.rollControl(roll_cmd);
  gait.pitchControl(pitch_cmd);
  gait.yawControl(yaw_cmd);

  ROS_INFO_STREAM("Calling update for 3 seconds");
  hz.reset();
  ros::Time startT = ros::Time::now();
  PeriodicLegState_t legsCmd;
  while (true) {
    gait.updateSineCmd(legsCmd);
    gait.setPeriodicLegCmd(legsCmd);
    gait.updateMotorTarget(motorTargets);
    //gait.update(motorTargets);
    SHOW_MOTOR_TARGETS;
    hz.sleep();
    if ((ros::Time::now() - startT).toSec() > 3.0) break;
  }

  ROS_INFO_STREAM("All done!");

  return 0;
};
