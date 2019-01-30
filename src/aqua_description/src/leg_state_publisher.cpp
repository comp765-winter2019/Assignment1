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

/*
 * leg_state_publisher.cpp
 *
 *  @brief: Needs to publish joint angles that are synchronized with the aqua URDF model
 *
 *  Created on: 2012-11-22
 *      Author: dpmeger
 *
 *  Revision history:
 *
 *  Nov 22, 2012: A simple node that publishes fake sinusoidal signals for the 6 legs
 */

#include "ros/ros.h"
#include <string>
#include "angles/angles.h"
#include "sensor_msgs/JointState.h"
#include <boost/algorithm/string.hpp>
#include <vector>

#include "aquacore/Health.h"

#define NUM_LEGS 6
const char* LEG_NAMES[NUM_LEGS] =
   { "left_front_leg_joint",
	 "left_mid_leg_joint",
	 "left_rear_leg_joint",
	 "right_front_leg_joint",
	 "right_mid_leg_joint",
	 "right_rear_leg_joint" };

const char* SHOULDER_NAMES[NUM_LEGS] =
   { "left_front_shoulder_joint",
	 "left_mid_shoulder_joint",
	 "left_rear_shoulder_joint",
	 "right_front_shoulder_joint",
	 "right_mid_shoulder_joint",
	 "right_rear_shoulder_joint" };

using namespace std;

class LegStatePublisher
{
	ros::NodeHandle* nh_;
	ros::Publisher js_pub_;
	sensor_msgs::JointState joint_state_;

	ros::Subscriber aqua_health_sub_;

public:

	LegStatePublisher()
	{
	    nh_ = new ros::NodeHandle("~");
		joint_state_.name.resize(2*NUM_LEGS+1);
		joint_state_.position.resize(2*NUM_LEGS+1);

		for (size_t i=0;i<NUM_LEGS;++i) {
			joint_state_.name[i] = SHOULDER_NAMES[i];
			joint_state_.position[i] = 0.0;
		}
        // this is a hack so that rviz does not complain about the depth sensor, hip and leg adapter joints
		for (size_t i=0;i<NUM_LEGS;++i) {
			joint_state_.name[i+6] = LEG_NAMES[i];
			joint_state_.position[i+6] = 0.0;
        }
        joint_state_.name[2*NUM_LEGS] = "depth_sensor_joint";
		joint_state_.position[2*NUM_LEGS] = 0.0;
        
        // for some reason, remapping was not working, so we had to add these parameters to set the topic names.
        std::string js_topic, aqua_health_topic;
        nh_->param<std::string>("joint_states_topic", js_topic, "/joint_states"); 
        nh_->param<std::string>("aqua_health_topic", aqua_health_topic, "/aqua/health");
        std::cout<<js_topic<<std::endl;
        std::cout<<aqua_health_topic<<std::endl;
		js_pub_ = nh_->advertise<sensor_msgs::JointState>(js_topic, 10);
		aqua_health_sub_ = nh_->subscribe(aqua_health_topic, 10, &LegStatePublisher::aquaHealthCallback, this);
	}

	~LegStatePublisher()
	{
	}

	// The main work is done here. Update the joint angles to match those published by the aquahw node.
	void aquaHealthCallback(const aquacore::HealthConstPtr& health)
	{
		for (size_t i=0;i<NUM_LEGS;++i) {
			joint_state_.position[i] = health->positions[i];
		}

		joint_state_.header.stamp = ros::Time::now();
		js_pub_.publish(joint_state_);
	}

	// In order to keep the legs updated in the visual representation, re-publish the
	// latest values at a faster rate, spinOnce frequently to make sure we get published values.
	void spin()
	{
		ros::Rate r(100);

		while (nh_->ok()) {
			ros::spinOnce();
//			for (size_t i=0;i<NUM_LEGS;++i) {
//				if( i==0 || i==2 || i==3 || i==5 )
//				{
//					joint_state_.position[i] = 0.75*sin(10.0*ros::Time::now().toSec());
//				}
//				else{
//					joint_state_.position[i] = 0.75*sin(10.0*ros::Time::now().toSec()+3.1415);
//				}
//
//			}
			joint_state_.header.stamp = ros::Time::now();
			js_pub_.publish(joint_state_);
			r.sleep();
		}
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "leg_state_publisher");

	LegStatePublisher lsp;
	lsp.spin();

	return 0;
}


