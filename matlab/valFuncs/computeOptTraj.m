function [traj, traj_tau, ctrls] = computeOptTraj(z0, g, value_funs,...
    tau, dynSys, uMode, extraArgs)
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

% Grid initial condition
z0 = grid.RealToCoords(z0);

% Time parameters
tauLength = length(tau)-tEarliest+1;
traj = nan(g.dim, tauLength);
traj_tau = [];
ctrls = cell(1, tauLength-1);
traj(:,1) = cell2mat(z0);

iter = 1;
z = z0;
small = 1e-4;

while iter < tauLength 
    
    % Find the optimal control
    val_per_ctrl = [];
    z_lin_idx = grid.RealToIdx(z);
    
    % Find which control takes system to next sub-zero level set. 
    for uidx=1:dynSys.num_ctrls
        % Get current control 
        u = dynSys.controls{uidx};
        likelyMask = likelyMasks(num2str(u));

        % If control is not likely enough at this state, continue.
        isLikley = likelyMask(z_lin_idx{1});
        if isnan(isLikley) || ~isLikley
            % mark this control as not likely -- NaNs wont be max/min'ed
            val_per_ctrl = [val_per_ctrl, NaN];
            continue;
        end
        
        % Propagate dynamics.
        znext = dynSys.dynamics(z, u);
        
        % Get BRS at next timestep.
        value_fun_next = value_funs{tEarliest+iter};
        
        % Get value at the next state after applying control.
        znext_lin_idx = grid.RealToIdx(znext);
        val = value_fun_next(znext_lin_idx{1});
        val_per_ctrl = [val_per_ctrl, val];
    end
    
    % Find the control which maximizes or minimizes next value. 
    if strcmp(uMode, "min")
        [optVal, ctrl_ind] = min(val_per_ctrl, [], 2);
    elseif strcmp(uMode, "max")
        [optVal, ctrl_ind] = max(val_per_ctrl, [], 2);
    else 
        error("Invalid uMode!");
    end
    
    fprintf('Value of current state at t=-%f: %f ...\n', iter, ...
        optVal);
        
    % Get and record the optimal control.
    opt_ctrl = dynSys.controls{ctrl_ind};
    ctrls{iter} = opt_ctrl;
    
    % Update next state after opt ctrl. 
    z = dynSys.dynamics(z,opt_ctrl);
  
    % Record new point on nominal trajectory
    iter = iter + 1;
    
    if iscell(z)
        traj(:,iter) = cell2mat(z);
    else
        traj(:,iter) = z;
    end
    
    % CHECK if the state has reached the target set! If it has, break.
    z_lin_idx = grid.RealToIdx(z);
    target_set = value_funs{end};
    vz = target_set(z_lin_idx{1});
    if vz < small
      break
    end
end

% Extract relevant times.
traj_tau = tau(1:iter);

end