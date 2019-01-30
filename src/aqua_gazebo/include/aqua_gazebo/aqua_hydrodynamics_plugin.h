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

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <stdio.h>
#include <chrono>
#include <vector>
#include "Eigen/Dense"
#include "aqua_gazebo/HydrodynamicsConfig.h"
#include "aqua_gazebo/HydrodynamicsParams.h"
#include "aqua_gazebo/ThrustParams.h"
#include "aqua_gazebo/HydrodynamicsParamsList.h"
#include "dynamic_reconfigure/server.h"

#include <aquacore/GetPeriodicLegCommand.h>
#include <aquacore/SetPeriodicLegCommand.h>
#include <aquacore/GetTargetLegAngles.h>
#include <aquacore/SetTargetLegAngles.h>

#define TWO_M_PI 2*M_PI
#define NUM_LEGS 6
const char* JOINT_NAMES[NUM_LEGS] =
       { "left_front_shoulder_joint",
	 "left_mid_shoulder_joint",
	 "left_rear_shoulder_joint",
	 "right_front_shoulder_joint",
	 "right_mid_shoulder_joint",
	 "right_rear_shoulder_joint"};

const char* LEG_NAMES[NUM_LEGS] =
       { "left_front_leg",
	 "left_mid_leg",
	 "left_rear_leg",
	 "right_front_leg",
	 "right_mid_leg",
	 "right_rear_leg"};

const char* SHOULDER_NAMES[NUM_LEGS] =
{ "left_front_shoulder",
    "left_mid_shoulder",
    "left_rear_shoulder",
    "right_front_shoulder",
    "right_mid_shoulder",
    "right_rear_shoulder"};

struct HydrodynamicParameters{
    std::vector<double> K; // Kirchoff tensor (only including added mass)
    std::vector<double> D; // Drag tensor
    std::vector<double> I; // Inertia tensor (currently unused, as we let gazebo's physics engine deal with this part)
    ignition::math::Vector3<double> com; // center of mass (currently unused, as we let gazebo's physics engine deal with this part)
    ignition::math::Vector3<double> cob; // center of buoyancy
    double lambda=1;        // a scaling factor that allows us to vary the frequency of wobbling in submerged bodies
    double drag_scaling=1;        // a scaling factor that allows us to change the frequency of wobbling due to drag
    double mass=0;
    double fluid_density=1;
    double fluid_viscosity=1.004e-6;
    double volume=0;
};

struct MVNormal{
    Eigen::VectorXd mean;
    Eigen::MatrixXd cov;
    Eigen::MatrixXd cholcov;

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> Nd{0.0, 1.0};
    Eigen::VectorXd _noise;

    void init(Eigen::VectorXd m, Eigen::MatrixXd c){
        mean = m;
        cov = c;
        cholcov = cov.llt().matrixL();
        _noise = Eigen::VectorXd::Zero(m.size());
    };

    Eigen::VectorXd sample(){

        return mean;
    }
};

struct PeriodicDisturbance{
    std::vector<double> freqs;
    std::vector<double> ampls;
    double noise;
};

class AquaHydrodynamicsPlugin: public gazebo::ModelPlugin
{
  public:
    AquaHydrodynamicsPlugin();
    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate(const gazebo::common::UpdateInfo & info);
    void DynamicReconfigureCallback(aqua_gazebo::HydrodynamicsConfig &config, uint32_t level);
    void InitDisturbances(int freq_components, double freq_noise, Eigen::MatrixXd vel_mean, Eigen::MatrixXd vel_cov);
    
    // flipper methods
    bool SetPeriodicLegCommand_cb(aquacore::SetPeriodicLegCommand::Request  &req, aquacore::SetPeriodicLegCommand::Response &res);
    bool GetPeriodicLegCommand_cb(aquacore::GetPeriodicLegCommand::Request  &req, aquacore::GetPeriodicLegCommand::Response &res);
    bool SetTargetLegAngles_cb(aquacore::SetTargetLegAngles::Request  &req, aquacore::SetTargetLegAngles::Response &res);
    bool GetTargetLegAngles_cb(aquacore::GetTargetLegAngles::Request  &req, aquacore::GetTargetLegAngles::Response &res);

  private:
    gazebo::common::Time current_time;
    gazebo::physics::ModelPtr model;
    gazebo::physics::WorldPtr world;
    gazebo::physics::LinkPtr base_link;
    std::vector<gazebo::physics::LinkPtr> leg_links;
    std::vector<gazebo::physics::LinkPtr> shoulder_links;
    gazebo::event::ConnectionPtr updateConnection;
    ignition::math::Pose3<double> initialPose, current_pose;
    ignition::math::Matrix3<double> M_inv;

    std::string robot_namespace;
    double surface_level, fluid_density, aqua_volume, fluid_viscosity, wobble,drag_scaling;
    bool planar;
    ignition::math::Vector3<double> drag_coeffs, leg_drag_coeffs;
    ignition::math::Box leg_bbox, aqua_bbox;
    std::map<std::string,HydrodynamicParameters> hydrodynamic_parameters;

    dynamic_reconfigure::Server<aqua_gazebo::HydrodynamicsConfig> *server_;
    ros::NodeHandle* nh_;
    ros::Publisher hparams_pub, tparams_pub;
    
    MVNormal disturbance_vel;
    PeriodicDisturbance disturbance_spectrum;
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<> Ud{0.0, 1.0};
    std::normal_distribution<> Nd{0.0, 1.0};
    Eigen::Vector3d dist_force;
    Eigen::Vector3d prev_vel;

    // flipper attributes
    gazebo::common::Time last_update_time;
    std::vector<gazebo::physics::JointPtr> motor_joints;
    std::vector<gazebo::common::PID> pid;

    ros::ServiceServer get_leg_command_service, set_leg_command_service;
    ros::ServiceServer get_target_angles_service, set_target_angles_service;
    
    boost::array<double,NUM_LEGS> target_angles;
    ignition::math::Vector3<double> pid_gains;
    
    boost::array<double,NUM_LEGS> frequency_cmd;
    boost::array<double,NUM_LEGS> phase_offsets_cmd;
    boost::array<double,NUM_LEGS> leg_offsets_cmd;
    boost::array<double,NUM_LEGS> amplitude_cmd;

    double thrustK1=0.25,thrustK2=0;
};
