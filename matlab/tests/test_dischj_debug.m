clear all
clf

%load('tmp_max.mat');
load('tmp_min.mat');

opt_traj = params.dyn_sys.get_opt_policy_from_x0(params.initial_state, params.trueThetaIdx);

grid = Grid(params.g.min, params.g.max, params.g.N); % for converting from real to linear index
all_states = grid.get_grid();
%likelyMasks = params.dyn_sys.getLikelyMasks(all_states);

upper = tauOut(end);
lower = 1;

z0 = params.initial_state;

% Grid initial condition
z0 = grid.RealToCoords(z0);

% Determine the earliest time that the current state is in the reachable set
tEarliest = findEarliestBRSInd(grid, value_funs, z0, upper, lower);

all_ctrls = flip(extraOuts.all_opt_ctrl_idxs);
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
%% plot the "optimal policy"
for i=2:length(opt_traj)
    c = 'k';
    plot([opt_traj(1,i-1),opt_traj(1,i)], ...
        [opt_traj(2,i-1),opt_traj(2,i)], ...
        'o-', 'Color', c, ...
        'markerfacecolor', 'w', ...
        'markeredgecolor', c, ...
        'linewidth', 2);
end

plot3(state_traj(1,:), state_traj(2,:), state_traj(3,:), 'o-r');
xlim([params.g.min(1),params.g.max(1)]);
ylim([params.g.min(2),params.g.max(2)]);
zlim([params.g.min(3),params.g.max(3)]);
box on
grid on