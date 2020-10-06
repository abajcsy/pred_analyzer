clear all
close all

%% Create grid.
gmin = [-6,-6];
gmax = [6,6];
gnums = [30,30];
pred_g = createGrid(gmin, gmax, gnums);
gdisc = (gmax-gmin) ./ (gnums-1);

pts = [pred_g.xs{1}(:), pred_g.xs{2}(:)];

% Compute timestep.
vel = 0.6;
dt = gdisc(1)/vel;

% %% Load map.
% repo = what('pred_analyzer');
% data_path = strcat(repo.path, '/matlab/data/');
% map_name = 'emptier_map.png'; 
% obs_data_3d = imread(strcat(data_path, map_name));
% obs_data_2d = rgb2gray(obs_data_3d);
% obs_map_full = (obs_data_2d == 0) .* 1.0 + (obs_data_2d > 0) .* 0.0;
% gimg = createGrid(gmin, gmax, size(obs_map_full));
% obs_map = eval_u(gimg, obs_map_full, pts);
% obs_map = reshape(obs_map, gnums);
% 
% bandw_cmap = [1,1,1;0,0,0];
% colormap(bandw_cmap)
% ph = pcolor(g.xs{1}, g.xs{2}, obs_map);
% set(ph, 'EdgeColor', 'none');

%% Create all beta values.
betas = [0.1, 1];

%% Prediction time info.
pred_hor = 8;                   % prediction horizon (in sec)
T = floor(pred_hor/dt);         % number of tsteps to predict for
discrete_times = 0:1:T;
real_times = discrete_times*dt;

%% Initial state and goal of human (in m)!
x0 = [2, 0];                    % initial position of human (in m)
goal = [-3.5, 0];
beta_prior = [0.1,0.9];

%% Create CONF-AWARE predictor.
fprintf('Predicting with CONFIDENCE-AWARE PREDICTOR...\n');
conf_predictor = ConfAwarePredictor(pred_g, goal, betas);
% Predict!
preds = conf_predictor.predict(x0, T, beta_prior);
% Plot preds!
figure(1)
conf_predictor.plot_preds(preds);

save('conf_preds_1090.mat', 'preds', 'discrete_times', 'real_times', 'x0', 'pred_g');

% %% Create OPT predictor.
% fprintf('Predicting with OPT PREDICTOR...\n');
% opt_predictor = OptPredictor(pred_g, goal);
% % Predict!
% preds = opt_predictor.predict(x0, T);
% % Plot preds!
% %figure(2)
% %opt_predictor.plot_preds(preds);
% 
% save('opt_preds.mat', 'preds', 'discrete_times', 'real_times', 'x0', 'pred_g');
% 
% %% Create FRS predictor.
% fprintf('Predicting with FRS PREDICTOR...\n');
% frs_predictor = FRSPredictor(pred_g);
% % Predict!
% preds = frs_predictor.predict(x0, T);
% % Plot preds!
% %figure(3)
% %frs_predictor.plot_preds(preds);
% 
% save('frs_preds.mat', 'preds', 'discrete_times', 'real_times', 'x0', 'pred_g');