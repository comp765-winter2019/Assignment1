% ANALYZE_FLEXIBLE_SINE.m

%clear all;

%% Define conditions for scenario
num_secs = 5;
times = linspace(0, num_secs, num_secs*1000);
default_cmd = struct('amplitude', 20/180*pi, 'frequency', 2.5, 'phase_offset', 0, 'leg_offset', 0, 'start_time', 0);
cmds = repmat(default_cmd, 3, 1);
cmds(1).leg_offset = 50/180*pi;

cmds(2).leg_offset = -90/180*pi;
cmds(2).start_time = 1.26;

cmds(3).leg_offset = 0/180*pi;
cmds(3).amplitude = 35/180*pi;
cmds(3).frequency = 4;
cmds(3).start_time = 2.55;

params.zero_crossing = false;
params.zero_crossing_update_tsinstart_to_match_phase = false; % this will give a discrepancy from ideal case only when frequency changes

params.phase_offset_use_static = false;

% NOTE: empirically, adding a acceleration cap, whether via saturation or
% sigmoid squashing, adds a temporal lag when switching frequencies. Thus,
% suggest first we try no acceleration limiting, and only if that still
% causes the robot to surge, then we apply saturation (which seems to give
% less oscillatory response in velocity)
params.motor_acceleration_squash_max = false; % if false, saturate
params.motor_acceleration_squash_gain = 2.0;
params.motor_acceleration_max = -1; % disable saturation and squashing
%params.motor_acceleration_max = (20/180*pi)*(2*pi*2.5)^2*2.0; % 20', 2.5Hz, hover-midoff case
%params.motor_acceleration_max = (20/180*pi)*(2*pi*4)^2*1.0; % 20', 4Hz, slightly conservative worst case
%params.motor_acceleration_max = (35/180*pi)*(2*pi*4)^2*1.0; % 35', 4Hz, worst case

params.motor_velocity_squash_max = true; % if false, saturate
params.motor_velocity_squash_gain = 2.0;
%params.motor_velocity_max = -1; % disable saturation and squashing
params.motor_velocity_max = (20/180*pi)*(2*pi*2.5)*2.0; % 20', 2.5Hz, hover-midoff case
%params.motor_velocity_max = (20/180*pi)*(2*pi*4)*1.0; % 20', 4Hz, slightly conservative worst case
%params.motor_velocity_max = (35/180*pi)*(2*pi*4)*1.0; % 35', 4Hz, worst case

%% Generate ideal and filtered motor target commands, and compute empirical estimates for filtered velocity and acceleration
ideal = sampleSinusoidalCommands(times, cmds, []);
filtered = sampleSinusoidalCommands(times, cmds, params);

dt = [0, times(2:end)-times(1:end-1)];
vel_empirical = [filtered.pos(1), filtered.pos(2:end) - filtered.pos(1:end-1)]./dt;
acc_empirical = [vel_empirical(1), vel_empirical(2:end) - vel_empirical(1:end-1)]./dt;

%% Plot results
num_cmds = length(cmds);
colors = hsv(num_cmds);
colors = [colors/3 + 0.66; colors/3 + 0.33];
color_specified = [0.8, 0.8, 0.8];
color_empirical = [0, 0, 0];
line_width_ideal = 5;
line_width_specified = 3;
line_width_empirical = 2;

figure(1);
clf;

subplot(3, 1, 1);
hold on;
plot(-1, -1, '--', 'LineWidth', line_width_ideal, 'Color', colors(1, :));
plot(-1, -1, '-', 'LineWidth', line_width_specified, 'Color', color_specified);
plot(-1, -1, ':', 'LineWidth', line_width_empirical, 'Color', color_empirical);

for i = 1:num_cmds,
  idx = (ideal.cmd_i==i);
  plot(ideal.times(idx), ideal.pos(idx)/pi*180, ...
    '--', 'LineWidth', line_width_ideal, 'Color', colors(i, :));
end
plot(filtered.times, filtered.pos/pi*180, ...
  '-', 'LineWidth', line_width_specified, 'Color', color_specified);
hold off;
xlabel('Time (sec)');
ylabel('Leg Position (deg)');
legend('Ideal (colors)', 'Computed', 'Empirical');
ax_data = ideal.pos/pi*180;
ax = axis();
yrange = max(ax_data) - min(ax_data);
ymid = (max(ax_data) + min(ax_data))/2;
ax(1) = 0; ax(2) = num_secs;
ax(3) = ymid - yrange*1.2/2; ax(4) = ymid + yrange*1.2/2;
axis(ax);

subplot(3, 1, 2);
hold on;
for i = 1:num_cmds,
  idx = (ideal.cmd_i==i);
  plot(ideal.times(idx), ideal.vel(idx)/pi*180, ...
    '--', 'LineWidth', line_width_ideal, 'Color', colors(i, :));
end
plot(filtered.times, filtered.vel/pi*180, ...
  '-', 'LineWidth', line_width_specified, 'Color', color_specified);
plot(filtered.times, vel_empirical/pi*180, ...
  ':', 'LineWidth', line_width_empirical, 'Color', color_empirical);
if ~isinf(params.motor_velocity_max),
  vel_sat = params.motor_velocity_max/pi*180;
  plot([filtered.times(1), filtered.times(end)], [vel_sat, vel_sat], '--k');
  plot([filtered.times(1), filtered.times(end)], -[vel_sat, vel_sat], '--k');
end
hold off;
xlabel('Time (sec)');
ylabel('Leg Velocity (deg/s)');
ax_data = ideal.vel/pi*180;
%ax_data = vel_empirical/pi*180;
ax = axis();
yrange = max(ax_data) - min(ax_data);
ymid = (max(ax_data) + min(ax_data))/2;
ax(1) = 0; ax(2) = num_secs;
ax(3) = ymid - yrange*1.2/2; ax(4) = ymid + yrange*1.2/2;
axis(ax);

subplot(3, 1, 3);
hold on;
for i = 1:num_cmds,
  idx = (ideal.cmd_i==i);
  plot(ideal.times(idx), ideal.acc(idx)/pi*180, ...
    '--', 'LineWidth', line_width_ideal, 'Color', colors(i, :));
end
%plot(filtered.times, filtered.acc/pi*180, ...
%  '-', 'LineWidth', line_width_specified, 'Color', color_specified);
plot(filtered.times, acc_empirical/pi*180, ...
  ':', 'LineWidth', line_width_empirical, 'Color', color_empirical);
if ~isinf(params.motor_acceleration_max),
  acc_sat = params.motor_acceleration_max/pi*180;
  plot([filtered.times(1), filtered.times(end)], [acc_sat, acc_sat], '--k');
  plot([filtered.times(1), filtered.times(end)], -[acc_sat, acc_sat], '--k');
end
hold off;
xlabel('Time (sec)');
ylabel('Leg Acceleration (deg/s^2)');
ax_data = ideal.acc/pi*180;
%ax_data = acc_empirical/pi*180;
ax = axis();
yrange = max(ax_data) - min(ax_data);
ymid = (max(ax_data) + min(ax_data))/2;
ax(1) = 0; ax(2) = num_secs;
ax(3) = ymid - yrange*1.2/2; ax(4) = ymid + yrange*1.2/2;
axis(ax);

%% Sigmoid squashing

if false;
  acc_max_asked = 400;
  acc_asked = linspace(-acc_max_asked, acc_max_asked, 1000);
  acc_max_thresh = 100;
  squash_gain = 2.0;

  acc_squashed = acc_max_thresh*(2./(1+exp(-squash_gain/acc_max_thresh*acc_asked)) - 1);

  figure(2);
  clf;
  subplot(211);
  hold on;
  plot(acc_asked, acc_asked, '-k');
  plot(acc_asked, acc_squashed, '--b');
  plot([-acc_max_asked, acc_max_asked], [acc_max_thresh, acc_max_thresh], ':r');
  plot([-acc_max_asked, acc_max_asked], -[acc_max_thresh, acc_max_thresh], ':r');
  hold off;
  xlabel('value asked');
  ylabel('value squashed');
  legend('Asked', 'Squashed');

  subplot(212);
  stem(acc_asked, acc_squashed-acc_asked);
  xlabel('value asked');
  ylabel('squashed - asked');
end
