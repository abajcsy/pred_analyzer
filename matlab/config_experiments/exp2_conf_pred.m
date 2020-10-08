function params = exp2_conf_pred()
params.gmin = [-6,-6];
params.gmax = [6,6];
params.gnums = [30,30];
params.pred_g = createGrid(params.gmin, params.gmax, params.gnums);
params.gdisc = (params.gmax-params.gmin) ./ (params.gnums-1);

%pts = [params.pred_g.xs{1}(:), params.pred_g.xs{2}(:)];

% Compute timestep.
params.vel = 0.6;
params.dt = params.gdisc(1)/params.vel;

%% Create all beta values.
params.betas = [0.1, 1];

%% Prediction time info.
params.pred_hor = 8;                   % prediction horizon (in sec)
params.T = floor(params.pred_hor/params.dt);         % number of tsteps to predict for
params.discrete_times = 0:1:params.T;
params.real_times = params.discrete_times*params.dt;

%% Initial state and goal of human (in m)!
params.x0 = [-1, 0]; %[2, 0];                    % initial position of human (in m)
params.goal = [-3.5, 0];
params.beta_prior = [0.5,0.5]; % b(beta = low_conf), b(beta = high_conf)

%% Create CONF-AWARE predictor.
fprintf('Setting up CONFIDENCE-AWARE PREDICTOR...\n');
params.predictor = ConfAwarePredictor(params.pred_g, params.goal, params.betas);
fprintf('Done.\n');

end