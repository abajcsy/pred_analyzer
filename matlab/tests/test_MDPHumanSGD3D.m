close all
clear all

params = mdpHumanSGD3DSimpleEnv();

xinit = {1,-2.5};
params.dyn_sys.plot_opt_policy_from_x0(xinit, 1)
params.dyn_sys.plot_opt_policy_from_x0(xinit, 2)
params.dyn_sys.plot_opt_policy_from_x0(xinit, 3)

% z0 = {0,0,0};
% u = 0;
% theta_n = params.dyn_sys.sgd(u,z0);