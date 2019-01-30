function [] = plot_ap_bag_data(basename,load,ver)


if nargin < 3
    ver = 3;
end

if nargin < 4
    load = true;
end

function out = clamp180(in)

   for idx=1:length(in)
      elem=in(idx);
      while elem < -180
         elem = elem + 360.0; 
      end
      
      while elem > 180
          elem = elem - 360.0;
      end
      out(idx) = elem;      
   end

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

% 
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

plot_gain_trials( ap_status(:,pitch_p_gain), ap_status(:,timestamp), rpy_targets(:,1), aqua_state(:,1), rpy_targets(:,4), -aqua_state(:,9) );
%plot_gain_trials( ap_status(:,roll_p_gain), ap_status(:,timestamp), rpy_targets(:,1), aqua_state(:,1), rpy_targets(:,2), aqua_state(:,7) );

    function [] = plot_gain_trials( gain_vector, gain_times, target_times, state_times, target_vals, state_vals )

        gains = unique(gain_vector);
        error_summary = [];
        
        for gidx=1:length(gains)
            gain = gains(gidx);
            indices = find( gain_vector == gain );
            times = gain_times(indices);
            gain_start_time = times(1);
            gain_end_time = times(end);

            target_indices = find( target_times > gain_start_time & target_times < gain_end_time );
            state_indices = find( state_times > gain_start_time & state_times < gain_end_time );

            center_val = target_vals(target_indices(1))*180.0/pi;
            
            raw_errors = clamp180(state_vals(state_indices)-center_val);
            error_summary(gidx) = sum(abs(raw_errors))/length(raw_errors);
            
            %subplot(length(gains),1,gidx);
            f = figure();
            plot((target_times(target_indices)-target_times(target_indices(1)))*1e-9,clamp180(target_vals(target_indices)*180.0/pi-center_val),'r--');
            hold on;
            plot((state_times(state_indices)-state_times(state_indices(1)))*1e-9,clamp180(state_vals(state_indices)-center_val),'LineWidth',1.5);
            title(sprintf( 'Roll P Gain: %f',gain ));
            ylim([-90 90]);
            grid on;
            ylabel('Roll Angle (deg)');
            xlabel('Time (s)');
            legend('Target Roll','Actual Robot Roll');
            saveas( f, sprintf('/home/dpmeger/shadows/mrl/iros2014_uberpilot/images/ml_analysis/roll_gain_%f.fig',gain),'fig');
            saveas( f, sprintf('/home/dpmeger/shadows/mrl/iros2014_uberpilot/images/ml_analysis/roll_gain_%f.png',gain),'png');
        end
        
        f = figure();
        plot(gains,error_summary,'LineWidth',1.5);
        ylabel('Average Error (deg)');
        xlabel('Gain Value (unitless)');
        title('Roll Error vs Roll P Gain - Linesearch Result');
        grid on;
        saveas( f, '/home/dpmeger/shadows/mrl/iros2014_uberpilot/images/ml_analysis/roll_summary.fig','fig');
        saveas( f, '/home/dpmeger/shadows/mrl/iros2014_uberpilot/images/ml_analysis/roll_summary.png','png');
    end

% 
% 
% 
% 
% figure();
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,depth_p_gain),'r-');
% hold on;
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,depth_d_gain),'b-');
% title('Depth gains');
% legend('Depth p gain', 'Depth d gain');
% 
% figure();
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,roll_p_gain),'r-');
% hold on;
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,roll_i_gain),'g-');
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,roll_d_gain),'b-');
% legend( 'Roll P Gain', 'Roll I Gain', 'Roll D Gain');
% title( 'Roll Gains' );
% 
% figure();
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,pitch_p_gain),'r-');
% hold on;
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,pitch_i_gain),'g-');
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,pitch_d_gain),'b-');
% legend( 'Pitch P Gain', 'Pitch I Gain', 'Pitch D Gain');
% title( 'Pitch Gains' );
% 
% figure();
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,yaw_p_gain),'r-');
% hold on;
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,yaw_i_gain),'g-');
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,yaw_d_gain),'b-');
% legend( 'Yaw P Gain', 'Yaw I Gain', 'Yaw D Gain');
% title( 'Yaw Gains' );
% 
% figure();
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,roll_error));
% hold on;  
% % plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,roll_error_integral),'r-');  
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,roll_p_contrib),'g-');  
% % plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,roll_i_contrib),'c-');
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,roll_d_contrib),'k-');
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,roll_p_contrib)+ap_status(:,roll_i_contrib)+ap_status(:,roll_d_contrib),'b--','LineWidth',2);
% title( 'Roll Control Status' );
% grid on;
% legend( 'Roll Error', 'P contribution', 'D contribution', 'Output command' );
% 
% figure();
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,pitch_error));
% hold on;  
% % plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,pitch_error_integral),'r-');  
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,pitch_p_contrib),'g-');  
% % plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,pitch_i_contrib),'c-');
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,pitch_d_contrib),'k-');
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,pitch_p_contrib)+ap_status(:,pitch_i_contrib)+ap_status(:,pitch_d_contrib),'b--','LineWidth',2);
% grid on;
% title( 'Pitch Control Status' );
% legend( 'Pitch Error', 'P contribution', 'D contribution', 'Output command' );
% 
% figure();
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,yaw_error));
% hold on;  
% % plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,yaw_error_integral),'r-');  
% plot((ap_status(:,timestamp)-first_time)*1e-9,-ap_status(:,yaw_p_contrib),'g-');  
% plot((ap_status(:,timestamp)-first_time)*1e-9,-ap_status(:,yaw_i_contrib),'c-');
% plot((ap_status(:,timestamp)-first_time)*1e-9,-ap_status(:,yaw_d_contrib),'k-');
% plot((ap_status(:,timestamp)-first_time)*1e-9,-(ap_status(:,yaw_p_contrib)+ap_status(:,yaw_i_contrib)+ap_status(:,yaw_d_contrib)),'b--','LineWidth',2);
% grid on;
% title( 'Yaw Control Status' );
% legend( 'Yaw Error', 'P contribution', 'I contribution', 'D contribution', 'Output command' );
% 
% figure();
% plot((rpy_targets(:,1)-first_time).*1e-9,rpy_targets(:,2)*180.0/pi);
% hold on;
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,resultant_roll),'g-');
% plot((aqua_state(:,1)-first_time).*1e-9,aqua_state(:,7),'r-');
% grid on;
% title('Roll Angles');
% legend( 'User Target Roll', 'Depth Modified Roll', 'Aqua Roll' );
% 
% figure();
% plot((rpy_targets(:,1)-first_time)*1e-9,rpy_targets(:,3)*180.0/pi);
% hold on;
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,resultant_pitch),'g-');
% plot((aqua_state(:,1)-first_time)*1e-9,-aqua_state(:,8),'r-');
% grid on;
% title('Pitch Angles');
% legend( 'User Target Pitch', 'Depth Modified Pitch', 'Aqua Pitch' );
% 
% figure();
% plot((rpy_targets(:,1)-first_time).*1e-9,rpy_targets(:,4)*180.0/pi);
% hold on;
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,resultant_yaw),'g-');
% plot((aqua_state(:,1)-first_time).*1e-9,-aqua_state(:,9),'r-');
% grid on;
% title('Yaw Angles');
% legend( 'User Target Yaw', 'Depth Modified Yaw', 'Aqua Yaw' );
% 
% limits = [3350 3450];
% limits = [ 0 (last_time-first_time)*1e-9 ];
% 
% figure();
% subplot(2,1,1);
% %plot( (ap_status(:,1)-first_time).*1e-9, ap_status(:,depth_error) );
% plot( (targets(:,1)-first_time).*1e-9, targets(:,7) );
% hold on;
% plot((aqua_state(:,1)-first_time)*1e-9,aqua_state(:,20),'r-');
% set(gca,'YDir','reverse');
% grid on;
% xlim(limits);
% legend( 'Depth Error' );
% 
% subplot(2,1,2);
% plot( (rpy_targets(:,1)-first_time).*1e-9, 180.0/pi*rpy_targets(:,2),'b--' );
% hold on;
% plot((ap_status(:,timestamp)-first_time)*1e-9,180.0/pi*ap_status(:,depth_p_contrib),'c-');
% plot((ap_status(:,timestamp)-first_time)*1e-9,180.0/pi*ap_status(:,depth_d_contrib),'k-');
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,resultant_roll),'g-');
% grid on;
% xlim(limits);
% legend('User Roll Command', 'Depth P Contribution', 'Depth D Contribution', 'Target Roll Angle'); 
% 
% 
% figure();
% subplot(2,1,1);
% plot((rpy_targets(:,1)-first_time).*1e-9,rpy_targets(:,2)*180.0/pi);
% hold on;
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,resultant_roll),'g-');
% plot((aqua_state(:,1)-first_time).*1e-9,aqua_state(:,7),'r-');
% grid on;
% title('Roll Angles');
% legend( 'User Target Roll', 'Depth Modified Roll', 'Aqua Roll' );
% xlim([0 350]);
% ylim([-20 150]);
% 
% subplot(2,1,2);
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,roll_p_gain),'r-');
% hold on;
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,roll_i_gain),'g-');
% plot((ap_status(:,timestamp)-first_time)*1e-9,ap_status(:,roll_d_gain),'b-');
% legend( 'Roll P Gain', 'Roll I Gain', 'Roll D Gain');
% title( 'Roll Gains' );
% xlim([0 350]);
% grid on;
% 
end
 
 
