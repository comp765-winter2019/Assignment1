% PLOT_ANIMATED_LEG_COMMANDS.m

clear all;

%%
% Generate realistic motor targets
times = linspace(0, 10, 10000);

default_cmd = struct('amplitude', 30/180*pi, 'frequency', 2.5, 'phase_offset', 0, 'leg_offset', 0, 'start_time', 0);
cmds = repmat(default_cmd, 5, 1);
cmds(1).leg_offset = 0/180*pi;
cmds(2).leg_offset = -30/180*pi;
cmds(2).start_time = 2;
cmds(3).leg_offset = 30/180*pi;
cmds(3).start_time = 4;
cmds(4).leg_offset = 0/180*pi;
cmds(4).start_time = 6;
cmds(5).leg_offset = -30/180*pi;
cmds(5).start_time = 8;
  
params.zero_crossing = false;
params.zero_crossing_update_tsinstart_to_match_phase = false;
params.phase_offset_use_static = false;
params.motor_velocity_max = (1.0 * (20.0/180.0*pi)*(2*pi*2.5)*2.0);
params.motor_velocity_squash_max = true;
params.motor_velocity_squash_gain = 2.0;
params.motor_acceleration_max = -1;
params.motor_acceleration_squash_gain = 2.0;
params.motor_acceleration_squash_max = false;

motor_targets = sampleSinusoidalCommands(times, cmds, params);

%%

figure(1);
set(gcf, 'Position', [100, 100, 500, 200]);
clf;

times = linspace(0, 12, 12000);
motor_pos = repmat(motor_targets.pos(2001:8000), 1, 2)/pi*180;

if false, % 1kHz pos cmds
  num_colors = 400;
  colors = colormap(hsv(num_colors));
  hold on;
  for i = 1:length(times),
    plot(times(i), motor_pos(i), '.', 'Color', colors(mod(i-1, num_colors)+1, :));
  end
  hold off;
  
elseif false, % 50Hz pos cmds
  num_colors = 200;
  colors = colormap(hsv(num_colors));
  hold on;
  plot(times, motor_pos, 'k:');
  for i = 1:20:length(times),
    plot(times(i), motor_pos(i), '.', 'Color', colors(mod(i-1, num_colors)+1, :));
  end
  hold off;
  
elseif false, % 20Hz cmd stems
  num_colors = 10;
  hz = 20;
  colors = colormap(hsv(num_colors));
  hold on;
  plot(times, motor_pos, 'k:');
  for i = 1:(1000/hz):length(times),
    stem(times(i), motor_pos(i), '.', 'Color', colors(mod((i-1)/(1000/hz), num_colors)+1, :));
  end
  hold off;
  
elseif false, % 0.5Hz PLC cmds
  num_colors = 3;
  colors = colormap(hsv(num_colors));
  hold on;
  for i = 1:6,
    time_range = (i-1)*2000+(1:2000);
    plot(times(time_range), motor_pos(time_range), '-', 'Color', colors(mod(i-1, num_colors)+1, :));
  end
  hold off;

elseif true, % 0.5Hz PLC blocks
  num_colors = 3;
  colors = colormap(spring(num_colors));
  hold on;
  for i = 1:6,
    rectangle('Position', [2*(i-1), mod(i+1, 3)*-30, 2, 60], 'FaceColor', colors(mod(i-1, num_colors)+1, :), 'EdgeColor', 'none');
  end
  plot(times, motor_pos, '-k');
  hold off;
  
end

%hxlabel = xlabel('Time');
ylabel('Flipper Angle (deg)');
set(gca, 'YTick', -60:30:60, 'XTick', 0:1:12, 'XTickLabel', [], 'color', 'none');

ax = axis();
ax(1) = 0;
ax(2) = 6;
ax(3) = -75;
ax(4) = 75;
axis(ax);

%%

filename = 'raw.gif';
speed_up_gain = 2.0;
fps = 30;

t_prev = 0;
start = true;
for t_start=linspace(0, 6, round(6000/fps/speed_up_gain)),
  if t_start == 0,
    continue;
  end  
  
  t_end = t_start + 6;
  ax(1) = t_start;
  ax(2) = t_end;
  axis(ax);
  
  drawnow;
  frame = getframe(1);
  im = frame2im(frame);
  im2 = im;
  im2(im2 == 204) = 255;
  [A, map] = rgb2ind(im2, 256);
  if start,
    imwrite(A, map, filename, 'gif', 'LoopCount', Inf, 'DelayTime', (t_start-t_prev)/speed_up_gain);
    start = false;
  else
    imwrite(A, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', (t_start-t_prev)/speed_up_gain);
  end
  t_prev = t_start;
end
