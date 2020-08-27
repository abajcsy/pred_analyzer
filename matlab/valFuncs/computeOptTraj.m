function [traj, traj_tau] = computeOptTraj(z0, g, value_funs, tau, dynSys, uMode, extraArgs)
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
if extraArgs.interpolate
    z0 = z0;
else
    z0 = grid.RealToCoords(z0);
end

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
    vals_t = [];
    value_fun_next = value_funs{tEarliest+iter};
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
%             u_i = dynSys.controls{i};
%             likelyMask = likelyMasks(num2str(u_i));
%             if isnan(likelyMask(idx_curr{1}))
%                 val = nan;
%             else
%                 idx = grid.RealToIdx(dynSys.dynamics(z, u_i));
%                 val = value_fun_next(idx{1});
%             end
%             vals = [vals, val];
            u_i = dynSys.controls{i};
            likelyMask = likelyMasks(num2str(u_i));
            l = likelyMask(idx_curr{1});
            idx = grid.RealToIdx(dynSys.dynamics(z, u_i));
            if isnan(idx{1})
                val = nan;
                val_l = nan;
            else
                val = value_fun_next(idx{1});
                val_l = value_fun_next(idx{1}) * l;
            end
            vals = [vals, val_l];
            vals_t = [vals_t, val];
        end
    end
%     z
%     vals
%     vals_t

    if strcmp(uMode, "min")
        [optVal, ctrl_ind] = min(vals, [], 2);
%         all_ctrl_inds = find(vals == optVal);
%         if sum(all_ctrl_inds == 6) > 0
%             ctrl_ind = 6;
%         end
    elseif strcmp(uMode, "max")
        [optVal, ctrl_ind] = max(vals, [], 2);
    else 
        error("Invalid uMode!");
    end
%     optVal
%     ctrl_ind
    % Apply the optimal control.
    ctrl = dynSys.controls{ctrl_ind};
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
end

traj_tau = tau(1:iter);

end