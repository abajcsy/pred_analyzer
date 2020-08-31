%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

%% Load all the parameters for this computation!
params = carHuman4DDrivingEnv();

%% Sanity check -- Plot the opt control policies?
params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 1);
params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 2);
% params.dyn_sys.plot_opt_policy(2);
