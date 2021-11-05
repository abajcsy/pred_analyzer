clear all
close all

%% Load up all the info for robot.
robot_params = conf_robot_crosswalk();

%% Load up all the info for the human predictors.
% Setup what kind of predictors we have
coll_check = 'conf'; 

% Load pre-computed predictor.
load('conf_preds.mat');
repo = what('pred_analyzer');
data_path = strcat(repo.path, '/matlab/case_study_conf/');

% Re-compute and re-save predictor.
% human_params = conf_pred_crosswalk();
% save(strcat(data_path, 'confAware_preds.mat'), 'human_params');

%% Setup human start state. 
h_start = [1, 2]; 

human = human_params.predictor.dyn_sys;
human_control = {"U", "U", "U", "D", "D", "D", "D"};
traj = human.sim_traj({h_start(1),h_start(2)}, human_control);

%% Setup robot start state. 
r_start = [3.5, 2, pi, 3]; % max robot speed is 6mph

%% Setup start state.
h_xcurr = h_start;
r_xcurr = r_start;

%% Setup prior over betas.
%pbeta = [traj(3,1), 1-traj(3,1)]; %
pbeta = robot_params.belief; 

%% Extract timesteps. 
r_dt = robot_params.dt;
h_dt = human_params.dt;

%% Branch times!
% ttl for 10-90 prior: 1.818s
% ttl for 50-50 prior: 0.9091s
ttl = 0.9091;
branch_t = ttl;

% Compute simulation time based on the time-to-learn number of steps.
simT = length(traj)-1; %round(ttl/h_dt);

min_d_to_h = norm(h_xcurr - r_xcurr(1:2));

%% Save variables.
all_preds = {};
all_plans = {};
all_h_states = {};
all_r_states = {};
all_beliefs = {};
all_dists = {};

all_h_states{end+1} = h_xcurr;
all_r_states{end+1} = r_xcurr;
all_beliefs{end+1} = pbeta;
all_dists{end+1} = min_d_to_h;

planned = false;

tidx_branch = [];

for t=1:simT
    % Predict human!
    fprintf('Predicting...\n');
    human_preds = ...
        human_params.predictor.predict(h_xcurr, human_params.T, pbeta);

    %% Plan for the robot!
    fprintf('Planning...\n');
    robot_plan = ...
        robot_params.planner.plan(r_xcurr, robot_params.goal, ...
            human_preds, human_params.real_times, human_params.pred_g, ...
            coll_check);
    
    % Update human state.
    zt = traj(:,t+1);
    h_xnext = zt(1:2)';

    % Update robot state. 
    r_xnext = [robot_plan{1}(2), robot_plan{2}(2), robot_plan{5}(2), robot_plan{3}(2)];
    
    all_preds{t} = human_preds;
    all_plans{t} = robot_plan;
    all_h_states{t} = h_xcurr;
    all_r_states{t} = r_xcurr;
    all_beliefs{t} = pbeta;
    all_dists{t} = min_d_to_h;
    
    % Update belief over betas.
    pbeta = human_params.predictor.belief_update(h_xnext, h_xcurr, pbeta);

    % Finally update states.
    h_xcurr = h_xnext; 
    r_xcurr = r_xnext;
end

%% Finally update states.
h_xcurr = h_xnext; 
r_xcurr = r_xnext;

%% Save out the minimum distance between the robot and human.  
d_to_h = norm(h_xcurr - r_xcurr(1:2));
if d_to_h < min_d_to_h
    min_d_to_h = d_to_h;
end

all_preds{end+1} = human_preds;
all_plans{end+1} = robot_plan;
all_h_states{end+1} = h_xcurr;
all_r_states{end+1} = r_xcurr;
all_beliefs{end+1} = pbeta;
all_dists{end+1} = min_d_to_h;
    
fprintf('Min dist between robot and human: %f\n', min_d_to_h);
save(strcat(data_path,'conf_full_sim.mat'));
