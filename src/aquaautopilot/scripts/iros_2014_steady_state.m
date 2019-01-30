function [] = plot_ap_bag_data(basename,load,ver)

if nargin < 3
    ver = 3;
end

if nargin < 4
    load = true;
end

global ap_status;
global aqua_state;
global targets;
global rpy_targets;
global commands;
global depths;

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

close all;
 
if load
    ap_status = csvread( sprintf('%s_status.csv',basename), 2 );
    aqua_state = csvread( sprintf('%s_state.csv',basename),2 );
    targets = csvread( sprintf('%s_target_pose.csv',basename),2 );
    commands = csvread( sprintf('%s_command.csv',basename),2 );
    depths = csvread( sprintf('%s_filtered_depth.csv',basename),2 );
    
    
    rpy_targets = [];
    for r=1:length(targets)

       rpy_targets = [ rpy_targets; targets(r,1) QUAT_2_RPY_bt(targets(r,8:11)) ];

    end


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

function out = unClampRange( in, threshold )
    
    for idx=1:length(in)
        if in(idx) > threshold
            out(idx) = in(idx) -360;
        else
            out(idx) = in(idx);
        end
    end

end


first_time = ap_status(1,timestamp);
last_time = ap_status(end,timestamp);

limits = [120 155];

figure();
plot( (targets(:,1)-first_time).*1e-9, targets(:,7), 'LineWidth',1.5 );
hold on;
plot((aqua_state(:,1)-first_time)*1e-9,aqua_state(:,20),'r-','LineWidth',1.5);
set(gca,'YDir','reverse');
grid on;
xlim(limits);
legend( 'Target Depth', 'Robot Depth' );
title('Depth');
xlabel('Time (sec)');
ylabel('Depth (m)');

target_indexes = find( (targets(:,1)-first_time).*1e-9 > 125.0 );
target_depth = targets(target_indexes(1),7)

depth_indexes = find( (aqua_state(:,1)-first_time).*1e-9 > 125.0 & (aqua_state(:,1)-first_time).*1e-9 < 155.0 );
depths = aqua_state(depth_indexes,20);

depth_errs = abs(depths-target_depth)
mean(depth_errs)
var(depth_errs)

return

figure();
plot((rpy_targets(:,1)-first_time).*1e-9,rpy_targets(:,2)*180.0/pi,'LineWidth',1.5);
hold on;
%plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,resultant_roll),'g-');
plot((aqua_state(:,1)-first_time).*1e-9,aqua_state(:,7),'r-','LineWidth',1.5);
grid on;
title('Roll Angles');
xlim(limits);
%legend( 'User Target Roll', 'Depth Modified Roll', 'Aqua Roll' );
legend( 'User Target Roll', 'Aqua Roll' );
ylabel('Angle (deg)');
xlabel('Time (sec)');

figure();
plot((rpy_targets(:,1)-first_time)*1e-9,rpy_targets(:,3)*180.0/pi,'LineWidth',1.5);
hold on;
plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,resultant_pitch),'g-','LineWidth',1.5);
plot((aqua_state(:,1)-first_time)*1e-9,-aqua_state(:,8),'r-','LineWidth',1.5);
grid on;
title('Pitch Angles');
legend( 'User Target Pitch', 'Depth Modified Pitch', 'Aqua Pitch' );
xlim(limits);
ylabel('Angle (deg)');
xlabel('Time (sec)');

figure();
%plot((rpy_targets(:,1)-first_time).*1e-9,unClampRange(rpy_targets(:,4)*180.0/pi,90));
plot((rpy_targets(:,1)-first_time).*1e-9,rpy_targets(:,4)*180.0/pi,'LineWidth',1.5);
hold on;
%plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,resultant_yaw),'g-');
plot((aqua_state(:,1)-first_time).*1e-9,-aqua_state(:,9),'r-','LineWidth',1.5);
grid on;
title('Yaw Angles');
%legend( 'User Target Yaw', 'Depth Modified Yaw', 'Aqua Yaw' );
legend( 'User Target Yaw', 'Aqua Yaw' );
xlim(limits);
ylabel('Angle (deg)');
xlabel('Time (sec)');

end


