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

#include <aqua_gazebo/aqua_hardware_emulator.h>
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#define NUM_LEGS 6

std::random_device rd;
std::mt19937 gen(rd());


AquaHWPlugin::AquaHWPlugin(){

}

void AquaHWPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf){
    ROS_INFO("Loading the AquaHW emulator plugin");
    model = _parent;
    world = model->GetWorld();
    robot_namespace.clear();

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                << "Make sure gzserver is running, or load the Gazebo system plugin 'libgazebo_ros_api_plugin.so')");
        return;
    }

    // get parameters from SDF file (gazebo tags in the URDF file)
    if (_sdf->HasElement("robotNamespace"))
        robot_namespace = _sdf->Get<std::string>("robotNamespace") + "/";
    else{
        ROS_INFO("aquahw plugin missing <robotNameSpace>, defaults to /aqua");
        robot_namespace = "aqua";
    }

    if (_sdf->HasElement("imuTopic")){
        imu_topic = _sdf->Get<std::string>("imuTopic") + "/";
    } else {
        ROS_INFO("aquahw plugin missing <topicName>, defaults to /default_imu");
        imu_topic = "/default_imu";
    }

    if (_sdf->HasElement("depthSensorNoise")){
        depth_sensor_noise = _sdf->Get<double>("depthSensorNoise");
    } else {
        depth_sensor_noise = 0.0;
        ROS_INFO("aquahw plugin missing <depthSensorNoise>, defaults to %f", depth_sensor_noise);
    }
    gaussian_noise = std::normal_distribution<>(0,depth_sensor_noise);

    if (_sdf->HasElement("surfaceLevel")){
        surface_level = _sdf->Get<double>("surfaceLevel");
    } else {
        surface_level = 0.0;
        ROS_INFO("aquahw plugin missing <surfaceLevel>, defaults to %f", surface_level);
    }

    // Initialize the ROS node handle
    nh = new ros::NodeHandle(robot_namespace);

    // Get parameters
    double state_publish_rate, health_publish_rate, leg_amplitude, leg_period, failsafe_timer_period, controller_period;
    nh->param<double>("state_publish_rate", state_publish_rate, 10.0); //10Hz
    nh->param<double>("health_publish_rate", health_publish_rate, 2.0); //2Hz
    nh->param<double>("leg_amplitude", leg_amplitude, 20); 
    nh->param<double>("leg_period", leg_period, 0.4); 
    nh->param<double>("failsafe_timer_period_secs", failsafe_timer_period, 1.0); 
    nh->param<double>("controller_period_secs", controller_period, 0.001); 
    nh->param<bool>("debug_print", _debug_print, false); 

    // Initialize publishers and subscribers
    state_pub = nh->advertise<aquacore::StateMsg>("state", 1);
    health_pub = nh->advertise<aquacore::Health>("health", 1);
    rate_pub = nh->advertise<geometry_msgs::Twist>("positioning/angular_velocity", 1);

    imu_sub = nh->subscribe(imu_topic, 1, &AquaHWPlugin::ImuCallback, this);
    cmd_sub = nh->subscribe("command", 1, &AquaHWPlugin::process_command, this);
    keepalive_sub = nh->subscribe("keepalive", 1, &AquaHWPlugin::keepalive, this);
    periodic_leg_command_sub = nh->subscribe("periodic_leg_command", 1, &AquaHWPlugin::process_periodic_leg_command, this);
    new_episode_sub = nh->subscribe("/rl/reset_robot", 1, &AquaHWPlugin::reset_world, this);

    // Initialize Timers
    failsafe_timer = nh->createTimer(ros::Duration(failsafe_timer_period), &AquaHWPlugin::failsafe, this,false);
    state_broadcast_timer = nh->createTimer(ros::Duration(1.0/state_publish_rate), &AquaHWPlugin::publish_state, this);
    health_broadcast_timer = nh->createTimer(ros::Duration(1.0/health_publish_rate), &AquaHWPlugin::publish_health, this);
    // This is for ensuring that the UnderwaterSwimmerGait::update function gets called at a 1KHz rate
    uwsg_controller_timer = nh->createTimer(ros::Duration(controller_period), &AquaHWPlugin::uwsg_controller_update, this);

    // Initialize services
    pause_service = nh->advertiseService("pause", &AquaHWPlugin::pause,this);
    zerodepth_service = nh->advertiseService("zerodepth", &AquaHWPlugin::zerodepth,this);
    calibrate_service = nh->advertiseService("calibrate", &AquaHWPlugin::do_calibrate,this);
    is_calibrated_service = nh->advertiseService("is_calibrated", &AquaHWPlugin::is_calibrated,this);
    set_targetdepth_service = nh->advertiseService("set_targetdepth", &AquaHWPlugin::set_targetdepth,this);
    set_targetangles_service = nh->advertiseService("set_targetangles", &AquaHWPlugin::set_targetangles,this);
    set_gait_service = nh->advertiseService("set_gait", &AquaHWPlugin::set_gait,this);
    get_autopilot_state_service = nh->advertiseService("get_autopilot_state", &AquaHWPlugin::get_autopilot_state,this);
    get_autopilot_params_service = nh->advertiseService("get_autopilot_params", &AquaHWPlugin::get_autopilot_params,this);
    set_autopilot_param_service = nh->advertiseService("set_autopilot_params", &AquaHWPlugin::set_autopilot_params,this);
    dump_all_vars_service = nh->advertiseService("dump_all_variables", &AquaHWPlugin::dump_all_vars,this);
    set_leg_params_service = nh->advertiseService("set_leg_params", &AquaHWPlugin::set_leg_params,this);
    get_leg_params_service = nh->advertiseService("get_leg_params", &AquaHWPlugin::get_leg_params,this);
    set_direction_service = nh->advertiseService("set_direction", &AquaHWPlugin::set_direction,this);
    set_autopilot_mode_service = nh->advertiseService("set_autopilot_mode", &AquaHWPlugin::set_autopilot_mode,this);
    get_state_service = nh->advertiseService("get_state", &AquaHWPlugin::get_state,this);
    step_simulation_service = nh->advertiseService("step_simulation", &AquaHWPlugin::step_simulation,this);
    run_simulation_until_time_service = nh->advertiseService("run_until_time", &AquaHWPlugin::run_simulation_until_time,this);

    set_leg_command_client = nh->serviceClient<aquacore::SetPeriodicLegCommand>("set_leg_command", true);
    get_leg_command_client = nh->serviceClient<aquacore::GetPeriodicLegCommand>("get_leg_command", true);
    set_target_angles_client = nh->serviceClient<aquacore::SetTargetLegAngles>("set_target_joint_angles", true);
    get_target_angles_client = nh->serviceClient<aquacore::GetTargetLegAngles>("get_target_joint_angles", true);

    // initialize the leg joint angles vector.
    health_msg.positions = std::vector<float>(6,0);

    // this event is triggered called on every simulation iteration
    updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&AquaHWPlugin::OnUpdate, this, _1));

    //// get the starting leg positions
    //aqua_gazebo::GetTargetLegAngles angles_srv;
    //float start_angles[6];
    //if(get_target_angles_client.call(angles_srv)){
    //    for(unsigned int i =0; i< angles_srv.response.target_angles.size(); i++){
    //        start_angles[i] = angles_srv.response.target_angles[i];
    //    }
    //}
    //// start the robot controller
    //uwsg_controller.activate(start_angles);
    uwsg_controller.activate();
    state = SWIM;
    world->SetPaused(false);
    integrated_velocity.fill(0.0);
}

void AquaHWPlugin::OnUpdate(const gazebo::common::UpdateInfo & info){
    auto previous_time = current_time;
#ifdef ROS_MELODIC
    current_time = model->GetWorld()->SimTime();
#else
    current_time = model->GetWorld()->GetSimTime();
#endif

    // get joint positions
    for (int i=0; i< NUM_LEGS; i++){
#ifdef ROS_MELODIC
        health_msg.positions[i] = model->GetJoint(JOINT_NAMES[i])->Position(1);
#else
        health_msg.positions[i] = model->GetJoint(JOINT_NAMES[i])->GetAngle(1).Radian();
#endif
    }

#ifdef ROS_MELODIC
    state_msg.Depth = surface_level - model->GetLink("depth_sensor")->WorldPose().Pos().Z();
#else
    state_msg.Depth = surface_level - model->GetLink("depth_sensor")->GetWorldPose().pos.z;
#endif

    // add some noise
    state_msg.Depth += gaussian_noise(gen);

    // publish current state and health messages only if the simulation is not paused
    auto now =  ros::Time::now();
    health_msg.header.stamp = now;
    state_msg.header.stamp = now;
    
    health_pub.publish(health_msg);
    if (!world->IsPaused()){
        state_pub.publish(state_msg);
    }
}

void AquaHWPlugin::uwsg_controller_update(const ros::TimerEvent& e){
    if( !periodic_leg_command_active){
        uwsg_controller.updateSineCmd(latest_periodic_leg_command);
    }

    uwsg_controller.setPeriodicLegCmd(latest_periodic_leg_command);
    uwsg_controller.updateMotorTarget(_motor_targets);

    /*for(int i=0;i<6;i++){
      ROS_INFO("Leg [%d] -> ampl: [%f] freq: [%f] phase: [%f] offset: [%f]",i,latest_periodic_leg_command.amplitudes[i],latest_periodic_leg_command.frequencies[i],latest_periodic_leg_command.phase_offsets[i],latest_periodic_leg_command.leg_offsets[i]);
      }
      */
    //ROS_INFO("Motor targets are: [%f %f %f %f %f %f]",_motor_targets[0].pos,_motor_targets[1].pos,_motor_targets[2].pos,_motor_targets[3].pos,_motor_targets[4].pos,_motor_targets[5].pos);

    for (int i=0; i<6; i++){
        ta_srv.request.target_angles[i] = _motor_targets[i].pos;
        plc_srv.request.amplitudes[i] = latest_periodic_leg_command.amplitudes[i];
        plc_srv.request.frequencies[i] = latest_periodic_leg_command.frequencies[i];
        plc_srv.request.leg_offsets[i] = latest_periodic_leg_command.leg_offsets[i];
        plc_srv.request.phase_offsets[i] = latest_periodic_leg_command.phase_offsets[i];
    }
    set_target_angles_client.call(ta_srv);
    set_leg_command_client.call(plc_srv);
}

void AquaHWPlugin::ImuCallback(const sensor_msgs::ImuConstPtr &msg){
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation,q);
    
    tf::Transform transform;
    transform.setOrigin( tf::Vector3( 0,0,0 ) );
    transform.setRotation( q );
    tf_br.sendTransform( tf::StampedTransform( transform, msg->header.stamp, "/latest_fix", "/aqua_base" ));
    
    geometry_msgs::Twist current_velocity;
    current_velocity.linear.x = 0;
    current_velocity.linear.y = 0;
    current_velocity.linear.z = 0;

    current_velocity.angular.x = msg->angular_velocity.x;
    current_velocity.angular.y = msg->angular_velocity.y;
    current_velocity.angular.z = msg->angular_velocity.z;

    rate_pub.publish(current_velocity);
    
    double roll,pitch,yaw;
    tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
    state_msg.RollAngle = roll * 180.0 / M_PI;
    state_msg.PitchAngle = -pitch * 180.0 / M_PI;
    state_msg.YawAngle = -yaw * 180.0 / M_PI;
    
    double dt = (msg->header.stamp - latest_imu_msg.header.stamp).toSec();

    integrated_velocity[0] += msg->linear_acceleration.x*dt;
    integrated_velocity[1] += msg->linear_acceleration.y*dt;
    integrated_velocity[2] += msg->linear_acceleration.z*dt;

    latest_imu_msg = (*msg.get());
}

bool AquaHWPlugin::set_direction(aquacore::SetDirection::Request  &req,
        aquacore::SetDirection::Response &res){
    //
    //float direction = req.forward?1.0:-1.0;
    //uwsg_controller.foreaftControl(direction);
    return true;
} 

bool AquaHWPlugin::get_leg_params(aquacore::GetLegParams::Request  &req,
        aquacore::GetLegParams::Response &res){
    aquacore::GetPeriodicLegCommand srv;
    if(get_leg_command_client.call(srv)){
        res.amplitude = srv.response.amplitudes[0];
        res.period = 1.0/srv.response.frequencies[0];
        return true;
    }
    return false;
}

bool AquaHWPlugin::set_leg_params(aquacore::SetLegParams::Request  &req,
        aquacore::SetLegParams::Response &res){

    //return lite.SetLegParams(req.amplitude, req.period);
    aquacore::SetPeriodicLegCommand srv;
    srv.request.frequencies.fill(1.0/req.period);
    srv.request.amplitudes.fill(req.amplitude);
    return set_leg_command_client.call(srv);
}


bool AquaHWPlugin::set_autopilot_mode(aquacore::SetAutopilotMode::Request  &req,
        aquacore::SetAutopilotMode::Response &res){
    ROS_INFO("setting autopilot mode %d",req.mode);
    //bool ok = lite.SetAutopilotMode(req.mode);
    //res.response = ok;
    //return ok;
    res.response = true;
    return true;
}


bool AquaHWPlugin::get_autopilot_params(aquacore::GetAutopilotParams::Request  &req,
        aquacore::GetAutopilotParams::Response &res){

    //res.params = lite.GetAutopilotParams();
    // TODO
    return true;
}

bool AquaHWPlugin::get_autopilot_state(aquacore::GetAutopilotState::Request  &req,
        aquacore::GetAutopilotState::Response &res){

    //res.ap_state = lite.GetCurCommands().AutopilotSelect;
    //res.ap_mode = lite.GetAutopilotMode();
    //res.params = lite.GetAutopilotParams();
    //res.target_depth = lite.GetCurParams().GuiTargetDepth;
    //res.target_roll = lite.GetCurParams().ApTargetRollCmd;
    //res.target_pitch = lite.GetCurParams().ApTargetPitchCmd;
    //res.target_yaw = lite.GetCurParams().ApTargetYawCmd;
    // TODO
    return true;
}


bool AquaHWPlugin::set_autopilot_params(aquacore::SetAutopilotParams::Request  &req,
        aquacore::SetAutopilotParams::Response &res){

    //for (int i = 0; i < 9; i++) {
    //  autopilot_param_f[i]=req.values[i];
    //}
    //
    //lite.SetAutopilotParams(autopilot_param_f);
    //sleep(1);
    // TODO

    return true;
}


bool AquaHWPlugin::set_targetdepth(aquacore::SetTargetDepth::Request  &req, aquacore::SetTargetDepth::Response &res){
    ROS_INFO("Setting target depth at %f m", req.depth);
    //res.response = lite.SetTargetDepth(req.depth);
    //sleep(1);
    //return res.response;
    // TODO

    return true;
}

bool AquaHWPlugin::set_targetangles(aquacore::SetTargetAngles::Request  &req, aquacore::SetTargetAngles::Response &res){
    if(_debug_print)
        ROS_INFO("Setting target angles: roll %f, pitch %f, yaw %f degrees", req.target_roll, req.target_pitch, req.target_yaw);
    //lite.SetTargetAngles(req.target_roll, req.target_pitch, req.target_yaw);
    //res.response = true;
    //sleep(1);
    //return res.response;
    // TODO
    uwsg_controller.rollControl(req.target_roll);
    uwsg_controller.pitchControl(req.target_pitch);
    uwsg_controller.yawControl(req.target_yaw);

    state_msg.RollTargetAngle = req.target_roll;
    state_msg.PitchTargetAngle = req.target_pitch;
    state_msg.YawTargetAngle= req.target_yaw;
    state_msg.AvgRollCommand = req.target_roll;
    state_msg.AvgPitchCommand = req.target_pitch;
    state_msg.AvgYawCommand = req.target_yaw;

    return true;
}

bool AquaHWPlugin::set_gait(aquacore::SetGait::Request  &req, aquacore::SetGait::Response &res){
    ROS_INFO("Setting new gait %s", req.gait.c_str());
    if(req.gait=="flexible-sine"){
        periodic_leg_command_active = true;
    }
    else if(req.gait=="hover-midoff"){
        periodic_leg_command_active = false;
    }
    return true;
}

bool AquaHWPlugin::pause(aquacore::SetPauseMode::Request  &req,
        aquacore::SetPauseMode::Response &res){

    //if (req.value) {
    //  ROS_INFO("Pausing robot");
    //  lite.SetPause(true);
    //  state = STOP;
    //
    //} else {
    //  ROS_INFO("Unpausing robot");
    //  lite.SetPause(false);
    //  state = SWIM;
    //  return true;
    //}

    if (req.value) {
        ROS_INFO("Pausing robot");
        state = STOP;
        //uwsg_controller.pauseSR();
    } else {
        ROS_INFO("Unpausing robot");
        state = SWIM;
        //// get the starting leg positions
        //aqua_gazebo::GetTargetLegAngles angles_srv;
        //float start_angles[6];
        //if(get_target_angles_client.call(angles_srv)){
        //    for(unsigned int i =0; i< angles_srv.response.target_angles.size(); i++){
        //        start_angles[i] = angles_srv.response.target_angles[i];
        //    }
        //}
        //// resume the robot controller
        //uwsg_controller.resumeSR(start_angles);
    }
    return true;
}

bool AquaHWPlugin::dump_all_vars(aquacore::DumpAllVars::Request  &req,
        aquacore::DumpAllVars::Response &res){
    //UnderwaterSwimmerModeCommands_t cur_commands = lite.GetCurCommands();
    //UnderwaterSwimmerModeCommands_t pending_commands = lite.GetPendingCommands();
    //UnderwaterSwimmerModeParams_t cur_params = lite.GetCurParams();
    //UnderwaterSwimmerModeParams_t  pending_params = lite.GetPendingParams();
    //UnderwaterSwimmerModeState_t state = lite.GetState();
    //
    //if (req.current) {
    //  lite2ros(cur_commands, cur_params, res);
    //} else {
    //  lite2ros(pending_commands, pending_params, res);
    //}
    // TODO
    return true;
}


bool AquaHWPlugin::zerodepth(std_srvs::Empty::Request  &req,
        std_srvs::Empty::Response &res){
    return true;
}

bool AquaHWPlugin::is_calibrated(aquacore::IsCalibrated::Request  &req,
        aquacore::IsCalibrated::Response &res){
    res.value = true;
    return true;
}

bool AquaHWPlugin::do_calibrate(std_srvs::Empty::Request  &req,
        std_srvs::Empty::Response &res){
    return true;
}

void AquaHWPlugin::publish_health(const ros::TimerEvent& e){
    //StateProxyStructure_t health= lite.GetHealthState();
    //aquacore::Health::Ptr hmsg (new aquacore::Health);
    //hmsg->positions.insert(hmsg->positions.end(),health.positions, health.positions+6);
    //msg->temperatures.insert(hmsg->temperatures.end(),health.temperatures, health.temperatures+6);
    //hmsg->accelerations.insert(hmsg->accelerations.end(),health.accelerations, health.accelerations+3);
    //power
    //hmsg->voltage = health.voltage;
    //hmsg->current = health.current;
    //hmsg->avgvoltage = health.avgvoltage;
    //hmsg->avgcurrent = health.avgcurrent;
    //hmsg->avgpower = health.avgpower;
    //hmsg->battery_dod = health.battery_dod;
    //hmsg->hs_temp.insert(hmsg->hs_temp.end(), health.hs_temp, health.hs_temp+4);
    ////compass
    ////hmsg->heading = health.heading;
    ////    hmsg->pitch = health.pitch;
    ////    hmsg->roll = health.roll;
    ////    hmsg->dip = health.dip;

    health_pub.publish(health_msg);
}


aquacore::StateMsg AquaHWPlugin::current_state(){
    //state_msg.LED = state.LEDsOn;
    //state_msg.Gait = cmds.gaitselect;

    //state_msg.RollAngle = state.RollAngle;
    //state_msg.PitchAngle = state.PitchAngle;
    //state_msg.YawAngle = state.YawAngle;

    //state_msg.RollTargetAngle = params.ApTargetRollCmd;
    //state_msg.PitchTargetAngle = params.ApTargetPitchCmd;
    //state_msg.YawTargetAngle= params.ApTargetYawCmd;

    //state_msg.AutopilotMode = cmds.AutopilotMode;
    //state_msg.DepthTarget = params.GuiTargetDepth;   
    //state_msg.Depth = state.Depth;

    //state_msg.AvgRollCommand=state.AvgRollCommand;
    //state_msg.AvgPitchCommand=state.AvgPitchCommand;
    //state_msg.AvgYawCommand=state.AvgYawCommand;
    //state_msg.AvgHeaveCommand=state.Display1; //thats where Robodevel's UnderwaterSimmmerMode.cc puts em.
    //state_msg.AvgSurgeCommand=state.Display2;
    //state_msg.Speed = lite.GetSpeed();

    state_msg.header.stamp = ros::Time::now();

    return state_msg;
}

bool AquaHWPlugin::get_state(aquacore::GetState::Request  &req,
        aquacore::GetState::Response &res){
    res.state=current_state();
    return true;
}

void AquaHWPlugin::publish_state(const ros::TimerEvent& e){
    if (!world->IsPaused()){
        state_pub.publish(current_state());
    }
}

void AquaHWPlugin::keepalive(const aquacore::KeepAlive::ConstPtr& msg){
    keepalive(msg->keepalive);
}

void AquaHWPlugin::keepalive(bool k){
    if(k){
        m_last_command_time = boost::get_system_time();
    }
    else{
        zerospeed();
    }
}

void AquaHWPlugin::process_command(const aquacore::Command::ConstPtr& msg)
{
    if (state==STOP){
        ROS_WARN("Must unstop the robot first!");
        //state=SWIM;
        return;
    }
    if (periodic_leg_command_active){
       //ROS_INFO("Cannot send body commands while sending periodic state commands");
       return;
    }
    keepalive(true);
    float speed = msg->speed;
    float yaw = msg->yaw;
    float pitch = msg->pitch;
    float roll = msg->roll;
    float heave = msg->heave;
    if(_debug_print)
        ROS_INFO("Setting speed: %f  yaw: %f  pitch: %f  roll: %f  heave: %f", speed, yaw, pitch, roll, heave);

    uwsg_controller.rollControl(roll);
    uwsg_controller.pitchControl(pitch);
    uwsg_controller.yawControl(yaw);
    uwsg_controller.setSpeedCmd(speed);
    uwsg_controller.heaveControl(heave);

    state_msg.RollTargetAngle = roll;
    state_msg.PitchTargetAngle = pitch;
    state_msg.YawTargetAngle = yaw;
    state_msg.Speed = speed;
    state_msg.AvgRollCommand=roll;
    state_msg.AvgPitchCommand=pitch;
    state_msg.AvgYawCommand=yaw;
    state_msg.AvgHeaveCommand=heave;  

    //sleep(1);
    return;
}

void AquaHWPlugin::process_periodic_leg_command(const aquacore::PeriodicLegCommand::ConstPtr& msg){
    for (int i=0; i<6; i++){
        latest_periodic_leg_command.amplitudes[i] = msg->amplitudes[i];
        latest_periodic_leg_command.frequencies[i] = msg->frequencies[i];
        latest_periodic_leg_command.leg_offsets[i] = msg->leg_offsets[i];
        latest_periodic_leg_command.phase_offsets[i] = msg->phase_offsets[i];
    }
    keepalive(true);
}

void AquaHWPlugin::reset_world(const std_msgs::Time::ConstPtr &msg){
   ROS_INFO("Resetting simulation");
   world->SetPaused(true);
   // add small random noise to the initial state
   auto base_link = model->GetLink("aqua_base");
#ifdef ROS_MELODIC
   auto p0 = base_link->InitialRelativePose();
   auto p = ignition::math::Pose3d(p0);
    p.Pos().Set( p.Pos().X() + ignition::math::Rand::DblNormal(0,0.01),
              p.Pos().Y() + ignition::math::Rand::DblNormal(0,0.01),
              p.Pos().Z() + ignition::math::Rand::DblNormal(0,0.01));
   auto rot = p.Rot().Euler();
   rot.Set( rot.X() + ignition::math::Rand::DblNormal(0,0.01),
            rot.Y() + ignition::math::Rand::DblNormal(0,0.01),
            rot.Z() + ignition::math::Rand::DblNormal(0,0.01));
   p.Rot().Euler(rot);
#else
   auto p0 = base_link->GetInitialRelativePose();
   auto p = gazebo::math::Pose(p0);
    p.pos.Set( p.pos.x + gazebo::math::Rand::GetDblNormal(0,0.01),
               p.pos.y + gazebo::math::Rand::GetDblNormal(0,0.01),
               p.pos.z + gazebo::math::Rand::GetDblNormal(0,0.01));
    auto rot = p.rot.GetAsEuler();
    rot.Set( rot.x + gazebo::math::Rand::GetDblNormal(0,0.01),
             rot.y + gazebo::math::Rand::GetDblNormal(0,0.01),
             rot.z + gazebo::math::Rand::GetDblNormal(0,0.01));
    p.rot.SetFromEuler(rot);
#endif
   base_link->SetInitialRelativePose(p);
   world->ResetEntities(gazebo::physics::Base::ENTITY);
   world->SetPaused(false);
   usleep(10);
   //restore original initial pose
   base_link->SetInitialRelativePose(p0);
}

void AquaHWPlugin::zerospeed(){
    if(state!=STOP){
        if (_debug_print)
            ROS_INFO("Sending ZERO speed command");
        uwsg_controller.rollControl(0);
        uwsg_controller.pitchControl(0);
        uwsg_controller.yawControl(0);
        uwsg_controller.setSpeedCmd(0);
        uwsg_controller.heaveControl(0);
        for (int i=0; i<6; i++){
            latest_periodic_leg_command.amplitudes[i] = 0;
            latest_periodic_leg_command.frequencies[i] = 0;
            latest_periodic_leg_command.leg_offsets[i] = 0;
            latest_periodic_leg_command.phase_offsets[i] = 0;
        }
    }
}

//failsafe function for the robot. if no command hac been received in some time, then stop.
void AquaHWPlugin::failsafe(const ros::TimerEvent& e){
    if((boost::get_system_time() - m_last_command_time).total_milliseconds() > 3141 && state !=STOP){
        m_last_command_time = boost::get_system_time();
        zerospeed();
    }
}

bool AquaHWPlugin::step_simulation(aquacore::StepSimulation::Request  &req, aquacore::StepSimulation::Response &res)
{ 
    if (_debug_print)
        ROS_INFO("Stepping simulation for [%lf] seconds",req.duration);
#ifdef ROS_MELODIC
    int n_steps = std::floor(req.duration/(world->Physics()->GetMaxStepSize()));
#else
    int n_steps = std::floor(req.duration/(world->GetPhysicsEngine()->GetMaxStepSize()));
#endif
    n_steps = (n_steps>0)?n_steps:1;
    integrated_velocity.fill(0.0);
    world->Step(n_steps);
    //world->StepWorld(n_steps);
    res.timestamp = ros::Time::now();
    res.imu_data = latest_imu_msg;
    res.integrated_velocity[0] = integrated_velocity[0];
    res.integrated_velocity[1] = integrated_velocity[1];
    res.integrated_velocity[2] = integrated_velocity[2];
    for (int i=0; i<6; i++){
        res.amplitudes[i] = latest_periodic_leg_command.amplitudes[i];
        res.frequencies[i] = latest_periodic_leg_command.frequencies[i];
        res.leg_offsets[i] = latest_periodic_leg_command.leg_offsets[i];
        res.phase_offsets[i] = latest_periodic_leg_command.phase_offsets[i];
        res.phase_offsets[i] = latest_periodic_leg_command.phase_offsets[i];
#ifdef ROS_MELODIC
        res.joint_angles[i] = model->GetJoint(JOINT_NAMES[i])->Position(1);
#else
        res.joint_angles[i] = model->GetJoint(JOINT_NAMES[i])->GetAngle(1).Radian();
#endif
    }
    state_pub.publish(state_msg);

    return true;
};

bool AquaHWPlugin::run_simulation_until_time(aquacore::RunSimulationUntilTime::Request  &req, aquacore::RunSimulationUntilTime::Response &res)
{ 
    if (_debug_print)
        ROS_INFO_STREAM("Running simulation until "<<req.desired_time);
    double duration  = (req.desired_time - ros::Time::now()).toSec();
#ifdef ROS_MELODIC
    int n_steps = std::floor(duration/(world->Physics()->GetMaxStepSize()));
#else
    int n_steps = std::floor(duration/(world->GetPhysicsEngine()->GetMaxStepSize()));
#endif
    n_steps = (n_steps>0)?n_steps:1;
    integrated_velocity.fill(0.0);
    world->Step(n_steps);
    //world->StepWorld(n_steps);

    res.timestamp = ros::Time::now();
    res.imu_data = latest_imu_msg;
    res.integrated_velocity[0] = integrated_velocity[0];
    res.integrated_velocity[1] = integrated_velocity[1];
    res.integrated_velocity[2] = integrated_velocity[2];
    for (int i=0; i<6; i++){
        res.amplitudes[i] = latest_periodic_leg_command.amplitudes[i];
        res.frequencies[i] = latest_periodic_leg_command.frequencies[i];
        res.leg_offsets[i] = latest_periodic_leg_command.leg_offsets[i];
        res.phase_offsets[i] = latest_periodic_leg_command.phase_offsets[i];
        res.phase_offsets[i] = latest_periodic_leg_command.phase_offsets[i];
#ifdef ROS_MELODIC
        res.joint_angles[i] = model->GetJoint(JOINT_NAMES[i])->Position(1);
#else
        res.joint_angles[i] = model->GetJoint(JOINT_NAMES[i])->GetAngle(1).Radian();
#endif
    }

    state_pub.publish(state_msg);

    return true;
};

GZ_REGISTER_MODEL_PLUGIN(AquaHWPlugin);
