clear all
close all

load('conf_example_v5.mat');
load('exp_2_opt_human_traj_5050.mat');

%% Load up all the info for robot.
robot_params = exp2_planner_baselines();

%% Load up all the info for the human.
% Setup what kind of collision checking we will do
coll_check = 'conf'; 
%human_params = exp2_conf_pred();

%% Setup human start state. 
h_start = [1.4, 1.7]; 

%% Setup robot start state. 
r_start = [2.0, 2.9, -pi/6, 0.5]; %[2.0, 3, -pi/2, 0.5]; %[1.7, 2.8, -pi/4, 0.5]; 

%% Setup start state.
h_xcurr = h_start;
r_xcurr = r_start;

%% Setup prior over betas.
pbeta = [traj(3,1), 1-traj(3,1)]; %human_params.beta_prior; 

%% Extract timesteps. 
r_dt = robot_params.dt;
h_dt = human_params.dt;

%% Compute simulation time based on the time-to-learn number of steps.
ttl = 1.818;
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

for t=1:simT
    % Predict human!
    fprintf('Predicting...\n');
    human_preds = ...
        human_params.predictor.predict(h_xcurr, human_params.T, pbeta);
    
    % Plan for the robot! Re-plan every time we get a new observation.
    fprintf('Planning...\n');
    robot_plan = ...
        robot_params.planner.plan(r_xcurr, robot_params.goal, ...
            human_preds, human_params.real_times, human_params.pred_g, coll_check);
    
    % Update human state.
    %h_ctrl = pi/4;
    zt = traj(:,t+1);
    h_xnext = zt(1:2)';% [h_xcurr(1) + human_params.dt * cos(h_ctrl), ...
               % h_xcurr(2) + human_params.dt * sin(h_ctrl)];
    
    % Update robot state. Forward simulate the robot for the time it takes
    % to get the next observation. 
    ntsteps = round(human_params.dt/robot_params.dt);
    r_xnext = [robot_plan{1}(ntsteps), robot_plan{2}(ntsteps), ...
                robot_plan{5}(ntsteps), robot_plan{3}(ntsteps)];
    
    %% Update belief over betas.
    pbeta = human_params.predictor.belief_update(h_xnext, h_xcurr, pbeta);
    
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

end

fprintf('Min dist between robot and human: %f\n', min_d_to_h);
save('conf_example_v8.mat');
