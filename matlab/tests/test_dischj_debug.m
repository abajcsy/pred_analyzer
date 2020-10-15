clear all
clf

load('tmp_max_1.mat');
%load('tmp_max_15.mat');
%load('tmp_min.mat'); %needs: flip(extraOuts.all_opt_ctrl_idxs);

grid = Grid(params.g.min, params.g.max, params.g.N); % for converting from real to linear index
all_states = grid.get_grid();
%likelyMasks = params.dyn_sys.getLikelyMasks(all_states);

z0 = {-3.5, -2.69, 0.5}; %params.initial_state;
% Grid initial condition
z0 = grid.RealToCoords(z0);

opt_traj = params.dyn_sys.get_opt_policy_from_x0(z0, params.trueThetaIdx);

upper = tauOut(end);
lower = 1;

% Determine the earliest time that the current state is in the reachable set
tEarliest = findEarliestBRSInd(grid, value_funs, z0, upper, lower);

all_ctrls = extraOuts.all_opt_ctrl_idxs; %flip(extraOuts.all_opt_ctrl_idxs);
state_traj = zeros(3, upper-tEarliest+1);
z = z0;
t = tEarliest;
state_traj(:,1) = cell2mat(z)';
idx = 2;
while t < tauOut(end)
    uopt_idx = eval_u(params.g, all_ctrls{t}, cell2mat(z), 'nearest');
    uopt = params.dyn_sys.controls{uopt_idx};
    z = params.dyn_sys.dynamics(z, uopt);
    state_traj(:,idx) = cell2mat(z)';
    t = t+1;
    idx = idx +1;
end
hold on
%% Plot the "optimal policy"
for i=2:length(opt_traj)
    c = 'k';
    plot([opt_traj(1,i-1),opt_traj(1,i)], ...
        [opt_traj(2,i-1),opt_traj(2,i)], ...
        'o-', 'Color', c, ...
        'markerfacecolor', 'w', ...
        'markeredgecolor', c, ...
        'linewidth', 2);
end

%% GENERATE AND PLOT THE COMPUTEOPTTRAJ TRAJECTORY. 
[computed_traj, computed_traj_tau, computed_ctrls] = ...
                computeOptTraj(z0, ....
                                  params.g, ...
                                  value_funs, ...
                                  tauOut, ...
                                  params.dyn_sys, ...
                                  params.uMode, ...
                                  params.extraArgsCtrl);
plot3(computed_traj(1,:), computed_traj(2,:), computed_traj(3,:), 'o-b');

%% PLOT THE NEW TRAJ.
plot3(state_traj(1,:), state_traj(2,:), state_traj(3,:), 'o-r');
xlim([params.g.min(1),params.g.max(1)]);
ylim([params.g.min(2),params.g.max(2)]);
zlim([params.g.min(3),params.g.max(3)]);
box on
grid on