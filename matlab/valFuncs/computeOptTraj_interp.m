function [traj, traj_tau, opt_ctrls] = computeOptTraj_interp(z0, g, value_funs, ...
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
likelyMasks = dynSys.getLikelyMasks(all_states); % NOTE: this could be precomputed and stored within dynSys

clns = repmat({':'}, 1, g.dim);

if any(diff(tau)) < 0
  error('Time stamps must be in ascending order!')
end

% Time parameters
subSamples = 1;
iter = 1;
tauLength = length(tau);
dtSmall = (tau(2) - tau(1))/subSamples;

% Initialize trajectory
traj = nan(g.dim, tauLength);
opt_ctrls = {};
traj(:,1) = cell2mat(z0);
tEarliest = 1;
z = z0;

% Visualize?
visualize = false;
small = 1e-4;

% Determine the earliest time that the current state is in the reachable set
% Binary search
upper = tauLength;
lower = tEarliest;
%tEarliest = find_earliest_BRS_ind(g, value_funs, cell2mat(z), upper, lower);
tEarliest = findEarliestBRSInd(g, value_funs, cell2mat(z), upper, lower);

while iter <= tauLength 
  
  % BRS at current time
  BRS_at_t = value_funs{tEarliest}(clns{:});
  
%   if tEarliest == tauLength
%     % Trajectory has entered the target
%     break
%   end
  
  % Update trajectory
  % Find which control takes system to next sub-zero level set. 
  val_per_ctrl = [];
  val_per_ctrl_uninterp = [];
  for uidx=1:dynSys.num_ctrls
    % Get current control 
    u = dynSys.controls{uidx};
    likelyMask = likelyMasks(num2str(u));
    isLikely = eval_u(g, likelyMask, cell2mat(z));
    
    % If control is not likely enough at this state, continue.
    if isnan(isLikely) || ~isLikely
        % mark this control as not likely -- NaNs wont be max/min'ed
        val_per_ctrl = [val_per_ctrl, NaN];
        val_per_ctrl_uninterp = [val_per_ctrl_uninterp, NaN];
        continue;
    end
    
    % Propagate dynamics.
    znext = dynSys.dynamics(z, u);
    
    % Get BRS at next timestep.
    tEarliest_znext = tEarliest+1; %find_earliest_BRS_ind(g, value_funs, cell2mat(znext), upper, lower);
    BRS_at_tnext = value_funs{tEarliest_znext}(clns{:});
    
    % Get value at the next state after applying control. 
    val = eval_u(g, BRS_at_tnext, cell2mat(znext));
    val_per_ctrl = [val_per_ctrl, val];
    
    % -- debug, see what uninterp value is -- %
    idx = grid.RealToIdx(znext);
    val_uninterp = BRS_at_tnext(idx{1});
    val_per_ctrl_uninterp = [val_per_ctrl_uninterp, val_uninterp];
    % -- debug, see what uninterp value is -- %
    
      % Visualize BRS corresponding to current trajectory point
      if visualize
        curr_state = cell2mat(z);
        next_state = cell2mat(znext);
        hold on
        plot(traj(1, iter), traj(2, iter), 'k.') % old state
        plot(next_state(1), next_state(2), 'r.') % next state
        [g2D, data2D] = proj(g, BRS_at_tnext, [0,0,1], next_state(3));
        [g2Dcurr, data2Dcurr] = proj(g, BRS_at_t, [0,0,1], curr_state(3));
        visSetIm(g2D, data2D, 'r');
        visSetIm(g2Dcurr, data2Dcurr, 'k');
        tStr = sprintf('t = %.3f; tEarliest = %.3f', tau(iter), tau(tEarliest));
        title(tStr)
        drawnow
        hold off
      end
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
  opt_ctrls{end+1} = opt_ctrl;
  
  % Update next state after opt ctrl. 
  z = dynSys.dynamics(z,opt_ctrl);
  
  % Record new point on nominal trajectory
  iter = iter + 1;
  tEarliest = tEarliest + 1;
  
  if iscell(z)
     traj(:,iter) = cell2mat(z);
  else
     traj(:,iter) = z;
  end
  
  % CHECK if the state has reached the target set! If it has, break.
  vz = eval_u(g, value_funs{end}(clns{:}), cell2mat(z));
  if vz < small
      break
  end
end

% Delete unused indices
traj(:,iter+1:end) = [];
% Extract relevant times.
traj_tau = tau(1:iter);
%opt_ctrls(:,iter:end) = [];

end