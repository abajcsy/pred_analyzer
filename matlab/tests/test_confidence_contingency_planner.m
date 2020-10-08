clear all
close all

%% Load up all the info.
params = exp2_contingency_planner();

%% Load up all the info for the human predictors.
% Setup what kind of predictors we have
opt_human_params = exp2_opt_pred();
frs_human_params = exp2_frs_pred();
conf_human_params = exp2_conf_pred();

%% Robot start.
r_start = [1, 4, -pi/4, 0.01]; 

%% Setup human start state. 
h_start = [-1, 0];

%% Prior
pbeta = params.belief;

%% Predict human!
fprintf('Predicting...\n');
opt_preds = ...
    opt_human_params.predictor.predict(h_start, opt_human_params.T);

frs_preds = ...
    frs_human_params.predictor.predict(h_start, frs_human_params.T);

%% PLOT!
figure(1)
hold on

% Plot environment.
bandw_cmap = [0,0,0;1,1,1]; %[1,1,1;0,0,0];
colormap(bandw_cmap)
%ph = pcolor(params.g2d.xs{1}, params.g2d.xs{2}, params.sd_obs);
ph = pcolor(params.gimg.xs{1}, params.gimg.xs{2}, params.obs_map_full);
set(ph, 'EdgeColor', 'none');

% plot robot goal.
qrg = quiver(params.goal(1), params.goal(2), cos(params.goal(3)), sin(params.goal(3)), 'k');
qrg.Marker = 'o';
qrg.MarkerFaceColor = 'k';
% plot human goal.
scatter(opt_human_params.goal(1), opt_human_params.goal(2), 'r');
% plot human state
scatter(h_start(1), h_start(2), 'r', 'filled');
xlim([params.gmin(1), params.gmax(1)])
ylim([params.gmin(2), params.gmax(2)])
            

%% Branch times!
%{-1, 0, 0.5}           --> TTE: 1.379310 s
%{-0.3103, 0, 0.6009}   --> TTE: 1.379310 s
%{0.3793, 0, 0.6954}    --> TTE: 0.6897 s
%{1.069, 0, 0.7762}     --> TTE: 0.6897 s
%{1.7586, 0, 0.841}     --> TTE: 0 (this could also be close enough to conf)
%{2.4483, 0, 0.8898}    --> TTE: 0 (already confident enough)
branch_times = [1.379310, 1.379310, 0.6897, 0.6897, 0, 0];
branch_t = branch_times(1);

%% Plan!
opt_plan = ...
    params.planner.contingency_plan(r_start, params.goal, ...
                                   opt_preds, frs_preds, ...
                                   opt_human_params.real_times, ...
                                   opt_human_params.pred_g, ...
                                   pbeta, ...
                                   branch_t);

shared_spline = opt_plan{1};
spline_opt = opt_plan{2};
spline_frs = opt_plan{3};

%% Plot!
figure(1)
q = quiver(shared_spline{1}, shared_spline{2}, ...
    cos(shared_spline{5}), sin(shared_spline{5}), 'k');
q.Marker = 'o';
q.MarkerFaceColor = 'k';
q.MarkerEdgeColor = 'k';
q.ShowArrowHead = 'off';
q.AutoScale = 'off';
q.AutoScaleFactor = 0.5;

q = quiver(spline_opt{1}, spline_opt{2}, ...
    cos(spline_opt{5}), sin(spline_opt{5}), 'r');
q.Marker = 'o';
q.MarkerFaceColor = 'r';
q.MarkerEdgeColor = 'r';
q.ShowArrowHead = 'off';
q.AutoScale = 'off';
q.AutoScaleFactor = 0.5;

q = quiver(spline_frs{1}, spline_frs{2}, ...
    cos(spline_frs{5}), sin(spline_frs{5}), 'b');
q.Marker = 'o';
q.MarkerFaceColor = 'b';
q.MarkerEdgeColor = 'b';
q.ShowArrowHead = 'off';
q.AutoScale = 'off';
q.AutoScaleFactor = 0.5;

xlim([params.gmin(1), params.gmax(1)])
ylim([params.gmin(2), params.gmax(2)])
set(gcf, 'color', 'w');
set(gcf, 'position', [0,0,800,800])
