clear all
close all

%% Load up all the info for robot.
robot_params = contingency_robot_crosswalk();
% load('exp_2_opt_human_traj_1090.mat');

%% Load up all the info for the human predictors.
% Setup what kind of predictors we have
opt_human_params = opt_pred_crosswalk();
frs_human_params = frs_pred_crosswalk();

%% Setup human start state. 
h_start = [1, 2]; 

human = opt_human_params.dyn_sys;
human_control = {"U", "U", "U", "U"};
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
h_dt = opt_human_params.dt;

%% Branch times!
% ttl for 10-90 prior: 1.818s
% ttl for 50-50 prior: 0.9091s
ttl = 0.9091;
branch_t = ttl;

% Compute simulation time based on the time-to-learn number of steps.
simT = length(traj)-1; %round(ttl/h_dt);

min_d_to_h = norm(h_xcurr - r_xcurr(1:2));

%% Save variables.
all_opt_preds = {};
all_frs_preds = {};
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
    opt_preds = ...
        opt_human_params.predictor.predict(h_xcurr, opt_human_params.T);

    r_color = linspace(0.1,0.9,length(opt_preds));
    for i=length(opt_preds):-1:1
        levels = [-0.1,0.1];
        preds = opt_preds{i};
        threshold = robot_params.pthresh;
        locs = find(opt_preds{i} > 0.0);
        Xs = opt_human_params.pred_g.xs{1}(locs); 
        Ys = opt_human_params.pred_g.xs{2}(locs); 
        h = scatter(Xs(:), Ys(:), 'r', 'markeredgecolor', [1,r_color(i),r_color(i)]);
        %all_contour_h = [all_contour_h, h];
    end

    frs_preds = ...
        frs_human_params.predictor.predict(h_xcurr, frs_human_params.T);

    % Plan for the robot!
    if ~planned 
        fprintf('Planning...\n');
        robot_plan = ...
            robot_params.planner.contingency_plan(r_xcurr, robot_params.goal, ...
                                                   opt_preds, frs_preds, ...
                                                   opt_human_params.real_times, ...
                                                   opt_human_params.pred_g, ...
                                                   pbeta, ...
                                                   branch_t); 
        planned = true;   % false for replanning?                                     
    end

    % extract parts of contingency plan.
    shared_plan = robot_plan{1};
    opt_plan = robot_plan{2};
    frs_plan = robot_plan{3};

    % Update human state.
    %h_ctrl = pi/4;
    zt = traj(:,t+1);
    h_xnext = zt(1:2)';% [h_xcurr(1) + human_params.dt * cos(h_ctrl), ...
               % h_xcurr(2) + human_params.dt * sin(h_ctrl)];

    % Update robot state. Forward simulate the robot for the time it takes
    % to get the next observation. 
    ntsteps = round(opt_human_params.dt/robot_params.dt);
    % Update robot state.
    if t*ntsteps < length(shared_plan{1})
        r_xnext = [shared_plan{1}(t*ntsteps), shared_plan{2}(t*ntsteps), ...
            shared_plan{5}(t*ntsteps), shared_plan{3}(t*ntsteps)];
    else
        if isempty(tidx_branch)
            tidx_branch = t-1;
        end
        tidx = t*ntsteps - tidx_branch*ntsteps;
        r_xnext = [frs_plan{1}(tidx), frs_plan{2}(tidx), ...
                    frs_plan{5}(tidx), frs_plan{3}(tidx)];
    end
    
    all_opt_preds{t} = opt_preds;
    all_frs_preds{t} = frs_preds;
    all_plans{t} = robot_plan;
    all_h_states{t} = h_xcurr;
    all_r_states{t} = r_xcurr;
    all_beliefs{t} = pbeta;
    all_dists{t} = min_d_to_h;
end
%% Update belief over betas.
%pbeta = conf_human_params.predictor.belief_update(h_xnext, h_xcurr, pbeta);

%% Finally update states.
h_xcurr = h_xnext; 
r_xcurr = r_xnext;

%% Save out the minimum distance between the robot and human.  
d_to_h = norm(h_xcurr - r_xcurr(1:2));
if d_to_h < min_d_to_h
    min_d_to_h = d_to_h;
end

all_opt_preds{end+1} = opt_preds;
all_frs_preds{end+1} = frs_preds;
all_plans{end+1} = robot_plan;
all_h_states{end+1} = h_xcurr;
all_r_states{end+1} = r_xcurr;
all_beliefs{end+1} = pbeta;
all_dists{end+1} = min_d_to_h;
    
fprintf('Min dist between robot and human: %f\n', min_d_to_h);
load('conf_preds.mat');
repo = what('pred_analyzer');
data_path = strcat(repo.path, '/matlab/case_study_conf/');
save(strcat(data_path,'contingency_full_sim.mat'));
