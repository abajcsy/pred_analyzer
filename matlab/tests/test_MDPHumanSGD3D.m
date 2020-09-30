close all
clear all

params = mdpHumanSGD3DSimpleEnv();

% params.dyn_sys.plot_opt_policy(1);
% params.dyn_sys.plot_opt_policy(2);
% params.dyn_sys.plot_opt_policy(15);
% % 
% xinit = {-2,-1,0.1};
% params.dyn_sys.plot_opt_policy_from_x0(xinit, 1)
% params.dyn_sys.plot_opt_policy_from_x0(xinit, 2)
% params.dyn_sys.plot_opt_policy_from_x0(xinit, 3)
% 
% xinit = {0,-1,0.1};
% params.dyn_sys.plot_opt_policy_from_x0(xinit, 1)
% params.dyn_sys.plot_opt_policy_from_x0(xinit, 2)
% params.dyn_sys.plot_opt_policy_from_x0(xinit, 3)
% 
% xinit = {2,-1,0.1};
% params.dyn_sys.plot_opt_policy_from_x0(xinit, 1)
% params.dyn_sys.plot_opt_policy_from_x0(xinit, 2)
% params.dyn_sys.plot_opt_policy_from_x0(xinit, 3)

% z0 = {-2, -2, 0}; % TODO: this is kinda funky...?
z0 = params.g.xs;

% u = [0, 0.42105]; % up(3) -- should give more evidence for theta = 0
u = [-0.4211, 0.4211]; % left_up(6) -- should give more evidence for theta = 0

theta_n = params.dyn_sys.sgd(u,z0);
