function [trial] = analyzeDriftLearningBag(filepath)
% trial = analyzeDriftLearningBag('/localdata/anqixu/DriftLearner/DriftLearner_2014-09-02-19-20-00.bag.mat');

load(filepath);

% TODO: parse ap_cmd_data, plc_data, aqua_state_data

episode.num_steps = -1;
episode.end_wall_time = -1;
episode.phase_A_yaw = nan;
episode.phase_E_yaw = nan;
episode.alignment_reward = nan;
episode.drift_length_reward = nan;
episode.cumulative_reward = nan;
episode.twiddle_best_reward = nan;
episode.next_param_idx = -1;
episode.next_twiddle_state = nan;
episode.curr_params = zeros(1, 11)*nan;
episode.delta_params = zeros(1, 11)*nan;

num_episodes = size(eoe_data, 2); %#ok<*USENS>
trial.episodes = repmat(episode, num_episodes, 1);

for i=1:num_episodes,
  trial.episodes(i).num_steps = eoe_data{i}.step_idx;
  trial.episodes(i).end_wall_time = eoe_data{i}.wall_time;
  trial.episodes(i).phase_A_yaw = eoe_data{i}.phase_A_yaw;
  trial.episodes(i).phase_E_yaw = eoe_data{i}.phase_E_yaw;
  trial.episodes(i).alignment_reward = eoe_data{i}.alignment_reward;
  trial.episodes(i).drift_length_reward = eoe_data{i}.drift_length_reward;
  trial.episodes(i).cumulative_reward = eoe_data{i}.cumulative_reward;
  trial.episodes(i).twiddle_best_reward = eoe_data{i}.twiddle_reward_best;
  trial.episodes(i).next_param_idx = eoe_data{i}.param_idx;
  trial.episodes(i).next_twiddle_state = eoe_data{i}.twiddle_state;
  trial.episodes(i).curr_params = eoe_data{i}.curr_params_vec;
  trial.episodes(i).delta_params = eoe_data{i}.delta_params;
end

end
