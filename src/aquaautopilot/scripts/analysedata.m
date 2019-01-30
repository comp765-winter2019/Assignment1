if (0)
    datas = csvread('aqua_state_13_46.csv',2);
    datadepth = csvread('filtered_depth_13_46.csv',2);
    datatarget = csvread('target_pose_13_46.csv',2);
    datastatus = csvread('status_14_11.csv',2);
    datacommad = csvread('command_14_11.csv',2);


    time = 1;
    roll_p_gain = 2;
    pitch_p_gain =3;
    yaw_p_gain    =4;
    roll_i_gain   =5;
    pitch_i_gain  =6;
    yaw_i_gain    =7;
    roll_d_gain   =8;
    pitch_d_gain  =9;
    yaw_d_gain   =10;
    roll_error   =11;
    pitch_error =12;
    yaw_error =13;
    roll_error_integral=14;
    pitch_error_integral=15;
    yaw_error_integral=16;
    roll_p_contrib=17;
    pitch_p_contrib=18;
    yaw_p_contrib=19;
    roll_i_contrib=20;
    pitch_i_contrib=21;
    yaw_i_contrib=22;
    roll_d_contrib=23;
    pitch_d_contrib=24;
    yaw_d_contrib=25;
    depth_p_gain=26;
    depth_d_gain=27;
    depth_derivative=28;
    depth_p_contrib=29;
    depth_d_contrib=30;
    resultant_roll=31;1300
    resultant_pitch=32;
    resultant_yaw=33;


    FirstTime = datas(1,1);
    targetdepth = datatarget(:,7);
    targettime  = (datatarget(:,1)-FirstTime)*1e-9;

    yawcommandtime = (datacommad(:,1)-FirstTime)*1e-9;
    yawcommand = datacommad(:,3);

    
    AngleTime  = (datas(:,1)-FirstTime)*1e-9;
    Roll = datas(:,7);
    Pitch = datas(:,8);
    Yaw = datas(:,9);

    DepthDerivative.time = (datastatus(:,1)-FirstTime)*1e-9;
    DepthDerivative.value = datastatus(:,depth_derivative);
    DepthPGain =  datastatus(:,depth_p_gain);
    DepthDGain =  datastatus(:,depth_d_gain);
    YawPGain = datastatus(:,yaw_p_gain);
    YawDGain = datastatus(:,yaw_d_gain);
    
    
    DepthTime = (datadepth(:,1)-FirstTime)*1e-9;

    % Compensate for location of depth sensor at the back of the robot, at
    % approx distance of 0.25 meters from CG
    CorrectedDepth = zeros(size(datadepth,1),1);
    if (0)
        for index = 1:size(datadepth,1)
            PitchVal = interp1(AngleTime,Pitch,DepthTime(index),'nearest');
            CorrectedDepth(index) = datadepth(index,2) - 0.25*sind(PitchVal);
            if (mod(index,100)==0) index
            end
        end
    end
end

Limits = [1255 1280];
%Limits = [1110 1130];


clf
subplot(3,1,1);
hl(1) = plot(DepthTime,datadepth(:,2));
hold on;
hl(2) = plot(DepthTime,CorrectedDepth,'g-');
hl(3) = plot(targettime,targetdepth,'r-');
hl(4) = plot(DepthDerivative.time,DepthDerivative.value);
grid on
set(gca,'YDir','reverse');
ylabel('Depth (m)','FontSize',16);
xlim(Limits);
ylim([-0.5 2]);

subplot(3,1,2);
plot(AngleTime,Pitch);
hold on;
plot(yawcommandtime,20*yawcommand,'r-');
grid on;
ylabel('Pitch angle (deg)','FontSize',16);
xlim(Limits);


if (0)
    subplot(3,1,3);
    IntegPitch = cumsum(Pitch)*mean(AngleTime(2:end)-AngleTime(1:end-1));
    hk(1) = plot(AngleTime,IntegPitch);
    hk(2) = plot(DepthTime,-200*CorrectedDepth+5700,'g-');
    hold on;
    hk(3) = plot(DepthTime,-200*datadepth(:,2)+5700,'r-');
    grid on;
    xlim(Limits);
    legend(hk,{'Integral of pitch','depth scaled corrected','depth scaled'},'Location','NorthWest');
else
    subplot(3,1,3);
    kd(1) = plot(DepthDerivative.time,DepthPGain);
    hold on;
    kd(2) = plot(DepthDerivative.time,DepthDGain,'r:');
    kd(3) = plot(DepthDerivative.time,YawPGain,'g-.');
    kd(4) = plot(DepthDerivative.time,YawDGain,'ks');
    xlim(Limits);
    
    % Find the index for the current time in the plot
    [idx idx] = min(abs(DepthDerivative.time-mean(Limits)));
    hdl = title(sprintf('KPDepth=%.2f KDerDepth=%.2f KPYaw=%.2f KDerYaw=%.2f',DepthPGain(idx),DepthDGain(idx),YawPGain(idx),YawDGain(idx)));
    set(hdl,'FontName','Courier','FontSize',15);
end
