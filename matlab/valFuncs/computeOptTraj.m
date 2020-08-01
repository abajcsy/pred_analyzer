function [traj, traj_tau] = computeOptTraj(z0, g, value_funs, tau, dynSys, uMode)
% [traj, traj_tau] = computeOptTraj(g, data, tau, dynSys, extraArgs)
%   Computes the optimal trajectories given the optimal value function
%   represented by (g, data), associated time stamps tau, dynamics given in
%   dynSys.
%
% Inputs:
%   z0      - initial condition
%   g, data - grid and value function
%   tau     - time stamp (must be the same length as size of last dimension of
%                         data)
%   dynSys  - dynamical system object for which the optimal path is to be
%             computed
%   uMode   - specifies whether the control u aims to minimize or
%             maximize the value function

grid = Grid(g.min, g.max, g.N); % for converting from real to linear index
all_states = grid.get_grid();
likelyMasks = dynSys.getLikelyMasks(all_states);

upper = tau(end);
lower = 1;

% Determine the earliest time that the current state is in the reachable set
tEarliest = findEarliestBRSInd(grid, value_funs, z0, upper, lower);

if tEarliest == 0
    fprintf("BRS never included initial state! Cannot find optimal traj.");
    return;
end

% Grid initial condition
z0 = grid.RealToCoords(z0);

% Time parameters
tauLength = length(tau)-tEarliest+1;
traj = nan(g.dim, tauLength);
traj_tau = [];
if iscell(z0)
    traj(:,1) = cell2mat(z0);
else
    traj(:,1) = z0;
end

iter = 1;
z = z0;

while iter < tauLength 

    % Find the optimal control
    vals = [];
    value_fun_next = value_funs{tEarliest+iter};
    idx_curr = grid.RealToIdx(z);
    for i=1:dynSys.num_ctrls
        u_i = dynSys.controls{i};
        idx = grid.RealToIdx(dynSys.dynamics(z, u_i));
        likelyMask = likelyMasks(num2str(u_i));
        val = value_fun_next(idx{1}) * likelyMask(idx_curr{1});
        vals = [vals, val];
    end
    if strcmp(uMode, "min")
        [~, ctrl_ind] = min(vals, [], 2);
    elseif strcmp(uMode, "max")
        [~, ctrl_ind] = max(vals, [], 2);
    else 
        error("Invalid uMode!");
    end
    
    % Apply the optimal control.
    ctrl = dynSys.controls{ctrl_ind};
    z = grid.RealToCoords(dynSys.dynamics(z,ctrl));
  
    % Record new point on nominal trajectory
    iter = iter + 1;
    if iscell(z)
        traj(:,iter) = cell2mat(z);
    else
        traj(:,iter) = z;
    end
end

traj_tau = tau(1:iter);

end