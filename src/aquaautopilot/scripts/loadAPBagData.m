function [ap_status, aqua_state, targets, commands, depths, rpy_targets, first_time, last_time ] = loadAPBagData(basename,ver);

if nargin < 2
    ver = 3;
end

if ver == 3
    timestamp = 1;
    roll_p_gain = 2;
    pitch_p_gain = 3;
    yaw_p_gain = 4;
    roll_i_gain = 5;
    pitch_i_gain = 6;
    yaw_i_gain = 7;
    roll_d_gain = 8;
    pitch_d_gain = 9;
    yaw_d_gain = 10;
    roll_const_gain = 11;
    roll_error = 12;
    pitch_error = 13;
    yaw_error = 14;
    depth_error = 15;
    roll_error_integral = 16;
    pitch_error_integral = 17;
    yaw_error_integral = 18;
    roll_p_contrib = 19;
    pitch_p_contrib = 20;
    yaw_p_contrib = 21;
    roll_i_contrib = 22;
    pitch_i_contrib = 23;
    yaw_i_contrib = 24;
    roll_d_contrib = 25;
    pitch_d_contrib = 26;
    yaw_d_contrib = 27;
    roll_const_contrib = 28;
    depth_p_gain = 29;
    depth_d_gain = 30;
    depth_derivative = 31;
    depth_p_contrib = 32;
    depth_d_contrib = 33;
    resultant_roll = 34;
    resultant_pitch = 35;
    resultant_yaw = 36;
    filtered_depth_derivative = 37;
    depth_d_filter_period = 38;
    filtered_roll_deriv = 39;
    filtered_pitch_deriv = 40;
    filtered_yaw_deriv = 41;
    roll_d_filter_period = 42;
    pitch_d_filter_period = 43;
    yaw_d_filter_period = 44;
    roll_target = 45;
    pitch_target = 46;
    yaw_target = 47;
    current_roll = 48;
    current_pitch = 49;
    current_yaw = 50;
elseif ver == 2
    timestamp = 1;
    roll_p_gain = 2;
    pitch_p_gain = 3;
    yaw_p_gain = 4;
    roll_i_gain = 5;
    pitch_i_gain = 6;
    yaw_i_gain = 7;
    roll_d_gain = 8;
    pitch_d_gain = 9;
    yaw_d_gain = 10;
    roll_error = 11;
    pitch_error = 12;
    yaw_error = 13;
    depth_error = 14;
    roll_error_integral = 15;
    pitch_error_integral = 16;
    yaw_error_integral = 17;
    roll_p_contrib = 18;
    pitch_p_contrib = 19;
    yaw_p_contrib = 20;
    roll_i_contrib = 21;
    pitch_i_contrib = 22;
    yaw_i_contrib = 23;
    roll_d_contrib = 24;
    pitch_d_contrib = 25;
    yaw_d_contrib = 26;
    depth_p_gain = 27;
    depth_d_gain = 28;
    depth_derivative = 29;
    depth_p_contrib = 30;
    depth_d_contrib = 31;
    resultant_roll = 32;
    resultant_pitch = 33;
    resultant_yaw = 34;
    filtered_depth_derivative = 35;
    depth_d_filter_period = 36;
    filtered_roll_deriv = 37;
    filtered_pitch_deriv = 38;
    filtered_yaw_deriv = 39;
    roll_d_filter_period = 40;
    pitch_d_filter_period = 41;
    yaw_d_filter_period = 42;
elseif ver == 1
    timestamp = 1;
    roll_p_gain = 2;
    pitch_p_gain = 3;
    yaw_p_gain = 4;
    roll_i_gain = 5;
    pitch_i_gain = 6;
    yaw_i_gain = 7;
    roll_d_gain = 8;
    pitch_d_gain = 9;
    yaw_d_gain = 10;
    roll_error = 11;
    pitch_error = 12;
    yaw_error = 13;
    roll_error_integral = 14;
    pitch_error_integral = 15;
    yaw_error_integral = 16;
    roll_p_contrib = 17;
    pitch_p_contrib = 18;
    yaw_p_contrib = 19;
    roll_i_contrib = 20;
    pitch_i_contrib = 21;
    yaw_i_contrib = 22;
    roll_d_contrib = 23;
    pitch_d_contrib = 24;
    yaw_d_contrib = 25;
    depth_p_gain = 26;
    depth_d_gain = 27;
    depth_derivative = 28;
    depth_p_contrib = 29;
    depth_d_contrib = 30;
    resultant_roll = 31;
    resultant_pitch = 32;
    resultant_yaw = 33;
end

ap_status = csvread( sprintf('%s_status.csv',basename), 2 );
aqua_state = csvread( sprintf('%s_state.csv',basename),2 );
targets = csvread( sprintf('%s_target_pose.csv',basename),2 );
commands = csvread( sprintf('%s_command.csv',basename),2 );
depths = csvread( sprintf('%s_filtered_depth.csv',basename),2 );

rpy_targets = [];
for r=1:length(targets)

   rpy_targets = [ rpy_targets; targets(r,1) QUAT_2_RPY_bt(targets(r,8:11)) ];
   
   
%     for r=1:length(rpy_targets)
%        if rpy_targets(r,4) < 0
%            rpy_targets(r,4) = rpy_targets(r,4)+2*pi;
%        end
%     end
% 
%     for r=1:length(aqua_state)
%        if aqua_state(r,9) > 0
%            aqua_state(r,9) = aqua_state(r,9)-360.0;
%        end
%     end
end

first_time = ap_status(1,timestamp);
last_time = ap_status(end,timestamp);
