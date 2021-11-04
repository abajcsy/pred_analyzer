function [value_funs, tau, extraOuts] = ...
    DiscTimeHJIPDE_solve_DEBUG(initV, tau, schemeData, compMethod, extraArgs)
% [data, tau, extraOuts] = ...
%   DiscTimeHJIPDE_solve(initV, tau, schemeData, minWith, extraArgs)
%     Solves HJIPDE with initial conditions initV, at times tau, and with
%     parameters schemeData and extraArgs.
%
% ----- How to use this function -----
%
% Inputs:
%   initV           - initial value function
%   tau             - list of computation times
%   schemeData      - problem parameters passed into the Hamiltonian func
%                       .grid: grid (required!)
%                       .dynSys: dyanmical system
%                       .uMode: control mode (min or max)
%   compMethod      - Informs which optimization we're doing
%                       - 'set' or 'none' to compute reachable set 
%                       - 'minVWithL' to do min with targets
%   extraArgs       - this structure can be used to leverage other 
%                       additional functionalities within this function. 
%                       Its subfields are:
%     .obstacles:           (matrix) a single obstacle or a list of 
%                           obstacles with time stamps tau (obstacles must 
%                           have same time stamp as the solution)
%     .targets:             (matrix) a single target or a list of targets 
%                           with time stamps tau (targets must have same 
%                           time stamp as the solution). This functionality 
%                           is mainly useful when the targets are 
%                           time-varying, in case of variational inequality 
%                           for example; data0 can be used to specify the  
%                           target otherwise. This is also useful when
%                           warm-starting with a value function (data0)
%                           that is not equal to the target/cost function
%                           (l(x))
%     .stopInit:            (vector) stop the computation once the  
%                           reachable set includes the initial state
%
% Outputs:
%   value_funs - solution corresponding to grid g and time vector tau
%   tau  - list of computation times (redundant)
%   extraOuts - This structure can be used to pass on extra outputs, for
%               example:
%      .stoptau: time at which the reachable set contains the initial
%                state; tau and data vectors only contain the data till
%                stoptau time.
%      .precomp_ctrls_t: (float) time to precompute likely controls
%      .precomp_dyns_t: (float) time to precompute dynamics
%      .dp_comp_t: (float) time to do all the DP backups

% extract grid and dynamical system.
g = schemeData.grid;
gDim = g.dim;
dyn_sys = schemeData.dynSys;
controls = dyn_sys.controls;
num_ctrls = dyn_sys.num_ctrls;
num_timesteps = tau(end);
extraOuts = [];

%---Extract the information about targets----------------------------------
% TO DO: allow targets to be time-varying

% if you accidentally write target instead of targets, we got you
% covered
if isfield(extraArgs, 'target')
    warning('you wrote extraArgs.target instead of extraArgs.targets')
    extraArgs.targets = extraArgs.target;
end

if isfield(extraArgs, 'targets')
    targets = extraArgs.targets;
end

%---Extract the information about obstacles--------------------------------
if isfield(extraArgs, 'obstacles')
    obstacles = extraArgs.obstacles;
else
    obstacles = zeros(size(targets)) .* nan;
end

%---Stopping Conditions----------------------------------------------------

% Check validity of stopInit if needed
if isfield(extraArgs, 'stopInit')
    if ~isvector(extraArgs.stopInit) || gDim ~= length(extraArgs.stopInit)
        error('stopInit must be a vector of length g.dim!')
    end
end

% Store initial value function.
value_funs = cell(1, num_timesteps);
value_funs{num_timesteps} = initV;
all_opt_ctrl_idxs = cell(1, num_timesteps);
all_opt_ctrl_idxs{num_timesteps} = nan(g.N'); % CONTROL IS NOT DEFINED AT FINAL TIME!

% Compute BRS backwards in time.
tidx = num_timesteps - 1;

% Precompute likely masks over entire state space.
fprintf("Pre-computing all likely controls over state space...\n");
% store time.
ctrls_start = tic; 
likelyMasks = dyn_sys.getLikelyMasks(g.xs);
% Store time info!
extraOuts.precomp_ctrls_t = toc(ctrls_start); 
fprintf(' ------>  comp time (s): %f\n', extraOuts.precomp_ctrls_t);

% Precomputing all the next states.
fprintf("Pre-computing dynamics over state space...\n");
dyn_start = tic;
next_state_cache = containers.Map;
for ui=1:num_ctrls
    u = controls{ui};
    next_state = dyn_sys.dynamics(g.xs, u);
    next_state_cache(num2str(u)) = next_state;
end
% Store time info!
extraOuts.precomp_dyns_t = toc(dyn_start); 
fprintf(' ------>  comp time (s): %f\n', extraOuts.precomp_dyns_t);

% (Benchmarking) Timer for the overall computation.
overallStart = tic;

while tidx > 0
    
    %(Benchmarking) Timer for single computation.
    singleCompStart = tic;

    % Create grid and set data to value function at t+1
    compute_grid = Grid(g.min, g.max, g.N);
    compute_grid.SetData(value_funs{tidx + 1});
    
    % Grab the value function at the next timestep. 
    next_value_fun = value_funs{tidx + 1};
    if isfield(extraArgs, 'stopInit')
        initial_idx = compute_grid.RealToIdx(extraArgs.stopInit);
        initial_idx = initial_idx{1};
        fprintf('Value of initial state at t=-%f: %f ...\n', tidx+1, ...
           value_funs{tidx + 1}(initial_idx));
    end
    
    fprintf("Computing value function for iteration t=%f...\n", tidx);
    
    num_dims = numel(g.N);

    % Compute possible next state after one timestep and find possible
    % value functions for each control
    current_state = g.xs; %compute_grid.get_grid();
    possible_value_funs = zeros([g.N', num_ctrls]); %zeros([compute_grid.gnums, num_ctrls]);
    for ui=1:num_ctrls
        u = controls{ui};
        likelyMask = likelyMasks(num2str(u));
        
        next_state = next_state_cache(num2str(u));
        
        if num_dims == 3
            next_state_lin = [next_state{1}(:), next_state{2}(:), next_state{3}(:)];
        elseif num_dims == 4
            next_state_lin = [next_state{1}(:), next_state{2}(:), ...
                             next_state{3}(:), next_state{4}(:)];
        end
        
        value_at_next_state = eval_u(g, next_value_fun, next_state_lin, 'nearest');
        value_at_next_state = reshape(value_at_next_state, g.N');
        %compute_grid.GetDataAtReal(next_state);
        if num_dims == 3
            possible_value_funs(:,:,:,ui) = value_at_next_state .* likelyMask;
        elseif num_dims == 4
            possible_value_funs(:,:,:,:,ui) = value_at_next_state .* likelyMask;
        end                
    end

    % Minimize/Maximize over possible future value functions.
    if strcmp(schemeData.uMode, "min")
        if strcmp(compMethod, "none") || strcmp(compMethod, "set")
            value_fun = min(possible_value_funs, [], num_dims+1);
        elseif strcmp(compMethod, "minVWithL") || strcmp(compMethod, "minVwithL")
            [value_fun, opt_ctrl_idxs] = min(possible_value_funs, [], num_dims+1);
            value_fun = min(value_fun, targets);
        else 
            warning("compMethod is not supported!")
            return;
        end
    elseif strcmp(schemeData.uMode, "max")
        if strcmp(compMethod, "none") || strcmp(compMethod, "set")
            value_fun = max(possible_value_funs, [], num_dims+1);
        elseif strcmp(compMethod, "minVWithL") || strcmp(compMethod, "minVwithL")
            [value_fun, opt_ctrl_idxs] = max(possible_value_funs, [], num_dims+1);
            value_fun = min(value_fun, targets);
        else
            error("compMethod is not supported!")
        end
    else
        error("uMode not specified or not supported!")
    end
    
    % DEBUGGING!
    all_opt_ctrl_idxs{tidx} = opt_ctrl_idxs;
    %all_opt_ctrl_idxs{end+1} = opt_ctrl_idxs;
    
    % If there are obstacles, take max.
    value_fun = max(value_fun, obstacles);

    % Update value function at current time index.
    value_funs{tidx} = value_fun;

    singleCompEnd = toc(singleCompStart); 
    fprintf("    [One backup computation time: %f s]\n", singleCompEnd);

    % If commanded, stop the reachable set computation once it contains
    % the initial state.
    if isfield(extraArgs, 'stopInit')
        initial_idx = compute_grid.RealToIdx(extraArgs.stopInit);
        initial_idx = initial_idx{1};
        if value_fun(initial_idx) <= 0
            extraOuts.stoptau = tidx;
            fprintf("    Found earliest BRS containing z0!\n");
            break;
        end
    end
    
    tidx = tidx - 1;
end

% DEBUGGING!
extraOuts.all_opt_ctrl_idxs = all_opt_ctrl_idxs;

% Save data
% save('brt_min_uthresh013.mat', 'value_funs', 'gmin', 'gmax', 'gnums', ...
%     'thetas', 'dyn_sys', 'uThresh', 'trueThetaIdx', 'num_timesteps', 'uMode');

% End timing the overall computation;
extraOuts.dp_comp_t = toc(overallStart);
    
end