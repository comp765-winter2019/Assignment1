function [r,p,y,depth] = compute_tracking_errors( start_time, end_time, tag, basename )

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
    
    [ap_status, aqua_state, targets, commands, depths, rpy_targets, time_offset, last_time ] = loadAPBagData(basename);
    
    indices = find( (ap_status(:,1)-time_offset).*1e-9 > start_time & (ap_status(:,1)-time_offset).*1e-9 < end_time  );
    
    roll_errors = ap_status(indices,roll_error).*180.0/pi;% ap_status(indices,current_roll) - ap_status(indices,roll_target);
    pitch_errors = ap_status(indices,pitch_error).*180.0/pi;%ap_status(indices,current_pitch) - ap_status(indices,pitch_target);
    yaw_errors = ap_status(indices,yaw_error).*180.0/pi;%ap_status(indices,current_yaw) - ap_status(indices,yaw_target);
    depth_errors = ap_status(indices,depth_error);
    
    figure();
    subplot(4,1,1);
    plot((ap_status(indices,1)-time_offset).*1e-9, roll_errors);
    title('Roll');
    grid on;
    ylabel(tag);
    subplot(4,1,2);
    plot((ap_status(indices,1)-time_offset).*1e-9,pitch_errors);
    title('Pitch');
    grid on;
    subplot(4,1,3);
    plot((ap_status(indices,1)-time_offset).*1e-9,yaw_errors);
    title('Yaw');
    grid on;
    subplot(4,1,4);
    plot((ap_status(indices,1)-time_offset).*1e-9,depth_errors);
    title('Depth');
    grid on;
        
    r = mean(abs(roll_errors));
    p = mean(abs(pitch_errors));
    y = mean(abs(yaw_errors));
    depth = mean(abs(depth_errors));
    
    
    
    