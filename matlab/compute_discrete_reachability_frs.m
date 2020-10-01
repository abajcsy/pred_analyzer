%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

%% Load all the parameters for this computation!
%  See possible configuration files and create new ones in /matlab/config/

% === Setups where joint state includes direct parameter vals. === %
params = mdpHumanSGD3DSimpleEnv();

%% Solve for the discrete-time value function!
[value_funs, tauOut, extraOuts] = ...
    DiscTimeHJIPDE_solve(params.initial_value_fun, ...
                         params.tau, ...
                         params.schemeData, ...
                         params.minWith, ...
                         params.extraArgs);

%% Find if all states are reachable. 
reachable = zeros(size(params.g.xs{3}));
for t=1:length(value_funs)
    v = value_funs{t};
    
    % Mark all states which are in the FRS. 
    reachable(v <= 0.0) = 1;
end

all_ths_reachable = all(reachable,'all');
which_reachable = find(reachable == 1);
which_non_reachable = find(reachable == 0);
reachable = 'no';
if all_ths_reachable
    reachable = 'yes';
end
fprintf('==> Are all thetas reachable from the initial condition in %d timesteps? %s\n', ...
    params.tau(end), reachable);
fprintf('==> Num reachable thetas: %d\n', ...
    length(which_reachable));
fprintf('==> Num un-reachable thetas: %d\n', ...
    length(which_non_reachable));
