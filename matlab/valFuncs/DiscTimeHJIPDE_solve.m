function [value_funs, tau, extraOuts] = ...
    DiscTimeHJIPDE_solve(initV, tau, schemeData, compMethod, extraArgs)
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

% (Benchmarking) Timer for the overall computation.
overallStart = tic;

% Store initial value function.
value_funs = cell(1, num_timesteps);
value_funs{num_timesteps} = initV;

% Compute BRS backwards in time.
tidx = num_timesteps - 1;

while tidx > 0
    fprintf("Computing value function for iteration t=%f...\n", tidx);
    
    %(Benchmarking) Timer for single computation.
    singleCompStart = tic;

    % Create grid and set data to value function at t+1
    compute_grid = Grid(g.min, g.max, g.N);
    compute_grid.SetData(value_funs{tidx + 1});

    % Compute possible next state after one timestep and find possible
    % value functions for each control
    current_state = compute_grid.get_grid();
    possible_value_funs = zeros([compute_grid.gnums, num_ctrls]);
    likelyMasks = dyn_sys.getLikelyMasks(current_state);
    for i=1:num_ctrls
        u_i = controls{i};
        next_state = dyn_sys.dynamics(current_state, u_i);
        likelyMask = likelyMasks(num2str(u_i));
        data_next = compute_grid.GetDataAtReal(next_state);
        possible_value_funs(:,:,:,i) = data_next .* likelyMask;
    end

    % Minimize/Maximize over possible future value functions.
    if strcmp(schemeData.uMode, "min")
        if strcmp(compMethod, "none") || strcmp(compMethod, "set")
            value_fun = min(possible_value_funs, [], 4);
        elseif strcmp(compMethod, "minVWithL") || strcmp(compMethod, "minVwithL")
            value_fun = min(min(possible_value_funs, [], 4), targets);
        else 
            warning("compMethod is not supported!")
            return;
        end
    elseif strcmp(schemeData.uMode, "max")
        if strcmp(compMethod, "none") || strcmp(compMethod, "set")
            value_fun = max(possible_value_funs, [], 4);
        elseif strcmp(compMethod, "minVWithL") || strcmp(compMethod, "minVwithL")
            value_fun = min(max(possible_value_funs, [], 4), targets);
        else
            error("compMethod is not supported!")
        end
    else
        error("uMode not specified or not supported!")
    end
    
    value_fun = max(value_fun, obstacles);

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

% Save data
% save('brt_min_uthresh013.mat', 'value_funs', 'gmin', 'gmax', 'gnums', ...
%     'thetas', 'dyn_sys', 'uThresh', 'trueThetaIdx', 'num_timesteps', 'uMode');

% End timing the overall computation;
toc(overallStart);
    
end