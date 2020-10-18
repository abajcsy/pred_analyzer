%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

%% Random seed.
rng(13);

%% Load all the parameters for this computation!
%  See possible configuration files and create new ones in /matlab/config/
params = exp2_conf_reachability();

%params.dyn_sys.plot_opt_policy(1);

figure
bandw_cmap = [1,1,1;0,0,0]; %[0,0,0;1,1,1]; %[1,1,1;0,0,0];
colormap(bandw_cmap)
%ph = pcolor(params.gimg.xs{1}, params.gimg.xs{2}, params.obs_map_full);
ph = pcolor(params.reward_info.obstacles.g{1}, params.reward_info.obstacles.g{2}, params.reward_info.obstacles.data);
set(ph, 'EdgeColor', 'none');
xlim([params.gmin(1),params.gmax(1)])
ylim([params.gmin(2),params.gmax(2)])
box on
grid on

%% Randomly Sample Initial Conditions!
% Number of cells to divide x and y into.

num_init_cells_spatial = 45;
num_init_cells = num_init_cells_spatial; %8;

init_res_x = (params.gmax(1) - params.gmin(1))/num_init_cells;
init_res_y = (params.gmax(2) - params.gmin(2))/num_init_cells;
x = params.gmin(1):init_res_x:params.gmax(1);
y = params.gmin(2):init_res_y:params.gmax(2);
cell_bounds = {};
for yi = y(1:end-1)
    for xi = x(1:end-1)
        cell_bounds{end+1} = [xi; yi; xi+init_res_x; yi+init_res_x];
    end
end

% (Randomly sample) set of initial conditions from where to simulate the human. 
human_init_conds = {};
g2d = createGrid(params.reward_info.obstacles.gmin, ...
                params.reward_info.obstacles.gmax, ...
                params.reward_info.obstacles.gnums);
relevant_cell_bounds = {};            
for i=1:length(cell_bounds)
    bounds = cell_bounds{i};
    % generate random number in the interval (a,b) with 
    % the formula r = a + (b-a) .* rand(1,1).
    rx = bounds(1) + (bounds(3)-bounds(1))/2.; %* rand(1,1);
    ry = bounds(2) + (bounds(4)-bounds(2))/2.; %* rand(1,1);
    
    % don't include this initial condition if its in obstacle.
    occupancy_val = eval_u(g2d, ...
                params.reward_info.obstacles.data, ...
                [rx, ry]);
    if occupancy_val > 0
        continue;
    end
    relevant_cell_bounds{end+1} = bounds;
    human_init_conds{end+1} = [rx; ry];
    
    hold on;
    scatter(rx, ry, 'filled');
end
scatter(params.theta(1), params.theta(2), 'r')

%% Plot optimal control policy starting from initial condition.
%params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 1);

%% Solve for the discrete-time value function!
[value_funs, tauOut, extraOuts] = ...
    DiscTimeHJIPDE_solve(params.initial_value_fun, ...
                         params.tau, ...
                         params.schemeData, ...
                         params.minWith, ...
                         params.extraArgs);

%% Extract min TTE for each init cond 
binit = 0.1; %0.5;             
all_ttes = {};
all_trajs = {};

% Precompute once all the likely masks to save on compute time!
mask_grid = Grid(params.g.min, params.g.max, params.g.N); % for converting from real to linear index
all_states = mask_grid.get_grid();
likelyMasks = params.dyn_sys.getLikelyMasks(all_states);

% ----- TEST! ------ %
% init_state = {1.4, 1.7, 0.5};
% [traj, traj_tau, ctrls] = computeOptTraj(init_state, ....
%                       params.g, ...
%                       value_funs, ...
%                       tauOut, ...
%                       params.dyn_sys, ...
%                       params.uMode);
% % ----- TEST! ------ %

for i = 1:length(human_init_conds)
    init_xy = human_init_conds{i};
    init_state = {init_xy(1), init_xy(2), binit};

    % Find and plot optimal control sequence (if reachable by computed BRS)
    fprintf("Computing opt traj for xinit %d / %d ...\n", i, length(human_init_conds));
    traj_tau = computeTTE(init_state, ....
                          params.g, ...
                          value_funs, ...
                          tauOut, ...
                          params.dyn_sys, ...
                          params.uMode, ...
                          mask_grid, ...
                          likelyMasks);

    all_ttes{end+1} = (traj_tau(end)-1)*params.dt;        
    all_trajs{end+1} = cell2mat(init_state);
    fprintf('===> TTE: %f\n', (traj_tau(end)-1)*params.dt);   
    %text(init_xy(1), init_xy(2), num2str((traj_tau(end)-1)*params.dt));
    %plot(traj(1,:), traj(2,:), 'b-o')
end

save('tte_spatial_fine.mat', 'all_ttes', 'all_trajs', 'relevant_cell_bounds');
%save('tte_spatial.mat', 'all_ttes', 'all_trajs', 'relevant_cell_bounds');

% % Plot all TTEs!
% for i = 1:length(human_init_conds)
%     xy_pos = human_init_conds{i};
%     tte = all_ttes{i};
%     text(xy_pos(1), xy_pos(2), num2str(tte));
% end

% fprintf('===> TTE: %f\n', traj_tau(end)*params.dt);
% ====================================== %
% hold on
% pt = plot3(traj(1,:), traj(2,:), traj(3,:), '-o');
% pt.LineWidth = 2;
% pt.Color = 'k';
% pt.MarkerFaceColor = 'b';
% pt.MarkerEdgeColor = 'b';
% hold on
% Plot environment.
% bandw_cmap = [0,0,0;1,1,1]; %[1,1,1;0,0,0];
% colormap(bandw_cmap)
% %ph = pcolor(params.g2d.xs{1}, params.g2d.xs{2}, params.sd_obs);
% ph = pcolor(params.gimg.xs{1}, params.gimg.xs{2}, params.obs_map_full);
% set(ph, 'EdgeColor', 'none');
% xlim([params.gmin(1),params.gmax(1)])
% ylim([params.gmin(2),params.gmax(2)])
% box on
% grid on
% ====================================== %