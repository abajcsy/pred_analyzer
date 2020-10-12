function [traj, traj_tau, ctrls] = computeOptTraj_old(z0, g, value_funs, tau, dynSys, uMode, extraArgs)
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

if any(diff(tau)) < 0
  error('Time stamps must be in ascending order!')
end

upper = tau(end);
lower = 1;

% Determine the earliest time that the current state is in the reachable set
tEarliest = findEarliestBRSInd(grid, value_funs, z0, upper, lower);
if tEarliest == 0
    fprintf("BRS never included initial state! Cannot find optimal traj.\n");
    return;
end

% Grid initial condition
if extraArgs.interpolate
    z0 = z0;
else
    z0 = grid.RealToCoords(z0);
end

% Time parameters
tauLength = length(tau)-tEarliest+1;
traj = nan(g.dim, tauLength);
traj_tau = [];
ctrls = cell(1, tauLength-1);
if iscell(z0)
    traj(:,1) = cell2mat(z0);
else
    traj(:,1) = z0;
end

iter = 1;
z = z0;
small = 1e-4;

while iter < tauLength 
    % Get value function at next timestep. 
    value_fun_next = value_funs{tEarliest+iter};
    
    % Find the optimal control
    vals = [];
    vals_t = [];
    
    idx_curr = grid.RealToIdx(z);
    if extraArgs.interpolate
        grid.SetData(value_fun_next);
        for i=1:dynSys.num_ctrls
            u_i = dynSys.controls{i};
            likelyMask = likelyMasks(num2str(u_i));
            if isnan(likelyMask(idx_curr{1}))
                val = nan;
            else
                next_z = dynSys.dynamics(z, u_i);
                val = grid.interpolate(next_z);
            end
            vals = [vals, val];
        end
    else
        for i=1:dynSys.num_ctrls
            u_i = dynSys.controls{i};
            likelyMask = likelyMasks(num2str(u_i));
            
            isLikley = likelyMask(idx_curr{1});
            if isnan(likelyMask(idx_curr{1}))
                val = nan;
            else
                idx = grid.RealToIdx(dynSys.dynamics(z, u_i));
                val = value_fun_next(idx{1});
            end
            vals = [vals, val];
        end
    end

    if strcmp(uMode, "min")
        [optVal, ctrl_ind] = min(vals, [], 2);
    elseif strcmp(uMode, "max")
        [optVal, ctrl_ind] = max(vals, [], 2);
    else 
        error("Invalid uMode!");
    end
    
    %Apply the optimal control.
    fprintf('Value of current state at t=-%f: %f ...\n', iter, ...
            optVal);
    ctrl = dynSys.controls{ctrl_ind};
    ctrls{iter} = ctrl;
    z = dynSys.dynamics(z,ctrl);
    if ~extraArgs.interpolate
        z = grid.RealToCoords(z);
    end
  
    % Record new point on nominal trajectory
    iter = iter + 1;
    if iscell(z)
        traj(:,iter) = cell2mat(z);
    else
        traj(:,iter) = z;
    end
    
    % CHECK if the state has reached the target set! If it has, break.
    idx = grid.RealToIdx(z);
    target_set = value_funs{end};
    vz = target_set(idx{1});
    if vz < small
      break
    end
end

traj_tau = tau(1:iter);

end