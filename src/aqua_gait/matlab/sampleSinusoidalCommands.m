function motor_targets = sampleSinusoidalCommands(times, cmds, params)
% Sample Usage: motor_targets = sampleSinusoidalCommands('sample')

% Parameters for sample usage
sample = false;
if nargin == 1 && strcmp(times, 'sample'),
  times = linspace(0, 4, 4000);
  default_cmd = struct('amplitude', 20/180*pi, 'frequency', 2.5, 'phase_offset', 0, 'leg_offset', 0, 'start_time', 0);
  cmds = repmat(default_cmd, 3, 1);
  cmds(1).leg_offset = 50/180*pi;
  cmds(2).leg_offset = -90/180*pi;
  cmds(2).start_time = 1;
  cmds(3).leg_offset = 0/180*pi;
  cmds(3).frequency = 3;
  cmds(3).start_time = 2.5;
  params = [];
  sample = true;
end


% Validate arguments
if nargin < 3 & ~sample,
    params = [];
end


% Set initial output values
motor_targets.times = times;
motor_targets.pos = zeros(size(times));
motor_targets.vel = zeros(size(times));
motor_targets.acc = zeros(size(times));
motor_targets.phases = zeros(size(times));
motor_targets.cmd_i = zeros(size(times));
motor_targets.ideal = isempty(params);
num_cmds = length(cmds);
two_pi = 2*pi;


if motor_targets.ideal, % Batch sample ideal path
  tsinstart = 0;
  for i = 1:num_cmds,
    % Determine indices in times corresponding to current sinusoidal command
    if i < num_cmds,
      next_time = cmds(i+1).start_time;
    else
      next_time = times(end)+1;
    end
    cmd = cmds(i);
    times_i = ((times >= cmd.start_time) & (times < next_time));
    motor_targets.cmd_i(times_i) = i;
    
    % Compute phases for sinusoidal pattern, then compute motor target pos/vel/acc
    omega = two_pi*cmd.frequency;
    phases_i = omega*(times(times_i) - tsinstart) + cmd.phase_offset + 0; % assume no phase carry over from previous commands
    motor_targets.phases(times_i) = phases_i;
    motor_targets.pos(times_i) = cmd.amplitude*cos(phases_i) + cmd.leg_offset;
    motor_targets.vel(times_i) = -omega*cmd.amplitude*sin(phases_i);
    motor_targets.acc(times_i) = -omega*omega*cmd.amplitude*cos(phases_i);
    
    % Update tsinstart
    if cmd.frequency > 0,
      curr_cmd_period = 1.0/cmd.frequency;
      dtmax = times(find(times_i, 1, 'last')) - tsinstart;
      tsinstart = floor(dtmax/curr_cmd_period)*curr_cmd_period + tsinstart;
    end
  end
  
  
else % Sequentially sample path with requested filtering / conditions
  % NOTE: code follows logic in aqua_gait->Gaits.cpp / RoboDevel->UnderwaterSwimmer.cc
  
  % Create more fields in output for logging purposes
  motor_targets.FStsinstart = zeros(size(times));
  motor_targets.tsin = zeros(size(times));
  motor_targets.new_cmd_received = false(size(times));
  motor_targets.phaseOffsetChanged = false(size(times));
  motor_targets.velDownscaleFactor = ones(size(times));
  motor_targets.accDownscaleFactor = ones(size(times));
  motor_targets.filtered_pos_updated = false(size(times));
  
  % Setup initial variables
  cmd_i = 0;
  FSCurrPeriodicLegState = struct('amplitude', 0, 'frequency', 0, 'phase_offset', 0, 'leg_offset', 0, 'start_time', -inf);
  FSLatestMotorTargets = struct('pos', 0, 'vel', 0, 'acc', 0);
  FStsinstart = 0;
  FSLatestUpdateMotorTargetsTime = -1;
  currPhase = 0;
  %prev_vel_change_desired = 0;
  %prev_pos_change_desired = 0;
  
  for time_i = 1:length(times),
    now = times(time_i);
    new_cmd_received = false;
    tar.pos = 0;
    tar.vel = 0;
    tar.acc = 0;
    
    % Skip sample times until first command is in effect
    if now < cmds(1).start_time,
      continue;
    end
    
    
    % // 0. Determine time since last call
    dt = 0;
    if (FSLatestUpdateMotorTargetsTime >= 0),
      dt = now - FSLatestUpdateMotorTargetsTime;
    end
    FSLatestUpdateMotorTargetsTime = now;
    % // Do not update motor targets if time has not progressed
    if (dt <= 0),
      tar.pos = FSLatestMotorTargets.pos;
      tar.vel = FSLatestMotorTargets.vel;
      tar.acc = 0;
      motor_targets.pos(time_i) = tar.pos;
      motor_targets.vel(time_i) = tar.vel;
      motor_targets.acc(time_i) = tar.acc;
      motor_targets.phases(time_i) = currPhase;
      motor_targets.cmd_i(time_i) = cmd_i;
      continue;
    end

    
    % // 1. Update current leg commands with target commands
    % // Check for command switch, and determine current command
    if (cmd_i <= 0) || ((cmd_i < num_cmds) && (now >= cmds(cmd_i+1).start_time)),
      cmd_i = cmd_i + 1;
      new_cmd_received = true;
    end
    FSTargetPeriodicLegState = cmds(cmd_i);
    
    %// Update phase offset
    phaseOffsetChanged = false;
    if (~params.phase_offset_use_static && ...
        (FSCurrPeriodicLegState.phase_offset ~= FSTargetPeriodicLegState.phase_offset)),
      FSCurrPeriodicLegState.phase_offset = FSTargetPeriodicLegState.phase_offset;
      phaseOffsetChanged = true;
    end

    %// Update amplitude, frequency, and leg offset
    if (~params.zero_crossing || ...
        (FSCurrPeriodicLegState.amplitude == 0) || ...
        (FSCurrPeriodicLegState.frequency == 0)),
      FSCurrPeriodicLegState.amplitude = FSTargetPeriodicLegState.amplitude;
      FSCurrPeriodicLegState.frequency = FSTargetPeriodicLegState.frequency;
      FSCurrPeriodicLegState.leg_offset = FSTargetPeriodicLegState.leg_offset;
    else
      % // WARNING: The following logic probably does not work when phase_offsets change,
      % //          since we either need to compute new FStsinstart[i] based on matching
      % //          cos(params1) == cos(params2) & sign(sin(params1)) == sign(sign(params2));
      % //          or we immediately accept new phase offset, but then the analytical
      % //          velocity would be arbitrarily different from the true velocity
      % //          needed to implement the phase shift.
      % //
      % //          To see this, consider a simple example where amplitude and frequency
      % //          does not change, and phase offset changes by M_PI between 2 consecutive
      % //          calls:
      % //          - for any given tsin, what is the proper velocity?
      % //          - do we need to update FStsinstart? if so, to what?
      tsin = now - FStsinstart;
      % // WARNING: zeroCrossing() only work properly if updateMotorTarget() is called faster than 200Hz, and ideally faster than 400Hz
      if ~phaseOffsetChanged && ...
          FSZeroCrossing(tsin, FSCurrPeriodicLegState.frequency),
        % // May need to shift new FStsinstart if frequency changes, in order to match frequencys
        % // WARNING: this may in fact destroy phase offsets between legs...
        if params.zero_crossing_update_tsinstart_to_match_phase,
          newDTSin = tsin * FSCurrPeriodicLegState.frequency / FSTargetPeriodicLegState.frequency;
          FStsinstart = now - newDTSin;
        end

        FSCurrPeriodicLegState.amplitude = FSTargetPeriodicLegState.amplitude;
        FSCurrPeriodicLegState.frequency = FSTargetPeriodicLegState.frequency;
        FSCurrPeriodicLegState.leg_offset = FSTargetPeriodicLegState.leg_offset;
      end
    end
    
    % // 2. Compute sinusoidal motion, and check for end-of-sine-period condition
    % // Generate target pose and velocity based on ideal sinusoidal pattern
    tsin = now - FStsinstart;
    omega = two_pi*FSCurrPeriodicLegState.frequency;
    if (omega == 0),
      currPhase = 0;
      tar.pos = FSCurrPeriodicLegState.leg_offset;
      tar.vel = 0;
    else
      currPhase = omega * tsin + FSCurrPeriodicLegState.phase_offset;
      tar.pos = FSCurrPeriodicLegState.amplitude*cos(currPhase) + FSCurrPeriodicLegState.leg_offset;
      tar.vel = -FSCurrPeriodicLegState.amplitude*omega*sin(currPhase);
    end
    
    % // WARNING: this logic only works if updateMotorTarget() is called at a much faster rate than the period of the sine!
    if (omega == 0),
      FStsinstart = now; % // Reset counter
    elseif (tsin > 1.0/FSCurrPeriodicLegState.frequency),
      currPeriod = 1.0/FSCurrPeriodicLegState.frequency;
      tsinExtra = tsin - floor(tsin/currPeriod)*currPeriod;
      FStsinstart = now - tsinExtra; % // Reset counter
    end
    
    % // 3. Filter motor targets (pos and vel) to ensure safe current-limited
    % //  operations
    filteredVelUseEmpirical = false;
    velDownscaleFactor = 1;
    accDownscaleFactor = 1;
    filtered_pos_updated = false;
    
    % // 3.1 Switch to empirical velocity during transient state between
    % // two different periodic leg commands
    % //
    % // In particular, accept empirical velocity and ignore sinusoidal
    % // velocity when the former has larger magnitude, and when the
    % // former asks the leg position to move in the opposite direction
    % // of the latter
    velEmp = FSAngularMag(tar.pos, FSLatestMotorTargets.pos)/dt;
    if (fabs(velEmp) > fabs(tar.vel)) || (sign(velEmp) ~= sign(tar.vel)),
      tar.vel = velEmp;
      filteredVelUseEmpirical = true;
    end
    speedAllLegs = fabs(tar.vel);

    % // 3.2 Limit maximum velocity via saturation or sigmoid squashing
    if params.motor_velocity_max >= 0, % allow saturation or squashing
      if params.motor_velocity_squash_max, % Squash velocity
        velDownscaleFactor = sigmoidSquashGain(speedAllLegs, ...
          params.motor_velocity_max, ...
          params.motor_velocity_squash_gain);
        
      else % Saturate velocity
        if (speedAllLegs > params.motor_velocity_max),
          velDownscaleFactor = params.motor_velocity_max / speedAllLegs;
        end
      end
      tar.vel = tar.vel * velDownscaleFactor;
    end
    
    % // 3.3 Limit maximum acceleration via saturation or sigmoid squashing
    accMagnAllLegs = fabs(tar.vel - FSLatestMotorTargets.vel)/dt;
    accLeg = (tar.vel - FSLatestMotorTargets.vel)/dt;
    if params.motor_acceleration_max >= 0, % allow saturation or squashing
      if params.motor_acceleration_squash_max, % Squash acceleration
        accDownscaleFactor = sigmoidSquashGain(accMagnAllLegs, ...
          params.motor_acceleration_max, ...
          params.motor_acceleration_squash_gain);
        
      else % Saturate acceleration
        if (accMagnAllLegs > params.motor_acceleration_max),
          accDownscaleFactor = params.motor_acceleration_max/accMagnAllLegs;
        end
      end
      accLeg = accLeg * accDownscaleFactor;
    end
    tar.vel = FSLatestMotorTargets.vel + accLeg*dt;
    
    %{
    % // 3.3.5 Filter change in target velocity using PD feed-forward control
    %
    % NOTE: this first attempt at implementing a feed-forward controller
    % failed. No matter what sign is the Kd term, the empirical
    % acceleration results in wild oscillations
    vel_change_desired = accLeg*dt;
    vel_change_ff = params.motor_velocity_feedforward_kp * vel_change_desired - ...
      params.motor_velocity_feedforward_kd * (vel_change_desired - prev_vel_change_desired);
    tar.vel = FSLatestMotorTargets.vel + vel_change_ff;
    prev_vel_change_desired = vel_change_desired;
    %}
    
    % // 3.4 Filter change in target position using PD feed-forward control
    % // NOTE: only proceed if either:
    % //   A) during transient period when switching to a new periodic leg
    % //      command, empirical velocity is used rather than sinuosidal velocity;
    % //   B) at the start and end of transient phase, the acceleration
    % //      is saturated to prevent spiking
    if filteredVelUseEmpirical || accDownscaleFactor < 1,
      tar.pos = FSLatestMotorTargets.pos + tar.vel*dt;
      filtered_pos_updated = true;
    end
    
    
    % // 4. Save filtered motor targets
    FSLatestMotorTargets.pos = tar.pos;
    FSLatestMotorTargets.vel = tar.vel;
    FSLatestMotorTargets.acc = tar.acc;
    
    % Save data into output
    motor_targets.pos(time_i) = tar.pos;
    motor_targets.vel(time_i) = tar.vel;
    motor_targets.acc(time_i) = tar.acc;
    motor_targets.phases(time_i) = currPhase;
    motor_targets.cmd_i(time_i) = cmd_i;
    motor_targets.FStsinstart(time_i) = FStsinstart;
    motor_targets.tsin(time_i) = tsin;
    motor_targets.new_cmd_received(time_i) = new_cmd_received;
    motor_targets.phaseOffsetChanged(time_i) = phaseOffsetChanged;
    
    motor_targets.filtered_vel_use_empirical(time_i) = filteredVelUseEmpirical;
    motor_targets.velDownscaleFactor(time_i) = velDownscaleFactor;
    motor_targets.accDownscaleFactor(time_i) = accDownscaleFactor;
    motor_targets.filtered_pos_updated(time_i) = filtered_pos_updated;
  end
end


% Plot sample
if sample,
  colors = hsv(num_cmds);
  figure(1);
  clf;
  subplot(3, 1, 1);
  hold on;
  for i = 1:num_cmds,
    idx = (motor_targets.cmd_i==i);
    plot(motor_targets.times(idx), motor_targets.pos(idx)/pi*180, 'Color', colors(i, :));
  end
  hold off;
  xlabel('Time (sec)');
  ylabel('Leg Position (deg)');
  subplot(3, 1, 2);
  hold on;
  for i = 1:num_cmds,
    idx = (motor_targets.cmd_i==i);
    plot(motor_targets.times(idx), motor_targets.vel(idx)/pi*180, 'Color', colors(i, :));
  end
  hold off;
  xlabel('Time (sec)');
  ylabel('Leg Velocity (deg/s)');
  subplot(3, 1, 3);
  hold on;
  for i = 1:num_cmds,
    idx = (motor_targets.cmd_i==i);
    plot(motor_targets.times(idx), motor_targets.acc(idx)/pi*180, 'Color', colors(i, :));
  end
  hold off;
  xlabel('Time (sec)');
  ylabel('Leg Acceleration (deg/s^2)');
end

end



function o = fabs(i)
o = abs(i);
end



function s = sign(v)
if v > 0,
  s = 1;
elseif v < 0,
  s = -1;
else
  s = 0;
end
end


function dRad = FSAngularMag(bRad, aRad)
dRad = bRad - aRad + pi;
if dRad > 0,
  dRad = dRad - floor(dRad/2/pi)*2*pi - pi;
else
  dRad = dRad - (floor(dRad/2/pi) + 1)*2*pi + pi;
end
end


function zc = FSZeroCrossing(time, freq)
if (abs(time - 3.0/4/freq) <= 0.005),
  zc = true;
elseif (abs(time - 1.0/4/freq) <= 0.005),
  zc = true;
else
  zc = false;
end
end


function squashed = sigmoidSquash(val, thresh, gain)
if nargin < 3,
  gain = 2;
end
squashed = thresh*(2./(1 + exp(-gain/thresh*val)) - 1);
end

function downscaleGain = sigmoidSquashGain(val, thresh, gain)
if nargin < 3,
  gain = 2;
end
downscaleGain = 0;
if abs(val) > eps*10,
  downscaleGain = thresh/val*(2./(1 + exp(-gain/thresh*val)) - 1);
end
end
