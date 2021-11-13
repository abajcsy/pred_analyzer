clear all
clf
close all

%% Load all the parameters for this computation!
params = mdpHumanRobotEnv();

%% Setup
dyn_sys = params.dyn_sys;
g_phys = params.reward_info.g;
z0 = {-3.5,-0.5,0.5};% {0,-3.6,0.5}; % {-3,3,0.5}; % {-3.5,-0.5,0.5};

v = 0.9;

%% Test belief update
u_R = dyn_sys.controls{dyn_sys.opt_robot_control(z0)};
u_H = [0 0];
b0 = dyn_sys.pugivenxtheta(u_H, u_R, z0, dyn_sys.q_funs_H{1}) .* z0{3};
b1 = dyn_sys.pugivenxtheta(u_H, u_R, z0, dyn_sys.q_funs_H{2}) .* (1-z0{3});
normalizer = b0 + b1;
bnext = b0 ./ normalizer;
bnext = min(max(bnext, dyn_sys.b_range(1)), dyn_sys.b_range(2));

%% Test dynamics I

u_R_opt = dyn_sys.controls{dyn_sys.opt_robot_control(z0)};
mat = [];
us = [];
for i=1:numel(dyn_sys.controls)
    u = dyn_sys.controls{i};
    us = [us;u];
end

for theta_i=1:2
    pus = [];
    bs = [];
    q_fun_theta = dyn_sys.q_funs_H{theta_i};
    for i=1:numel(dyn_sys.controls)
        u_H = dyn_sys.controls{i};
        pu = dyn_sys.pugivenxtheta(u_H, u_R_opt, z0, q_fun_theta);
        b = dyn_sys.belief_update(u_H, u_R_opt, z0);
        pus = [pus,pu];
        bs = [bs,b];
    end
    if theta_i==1
        mat = [mat;pus];
    else
        mat = [mat;pus;bs];
    end
end

figure
hold on
u_R_opt = dyn_sys.controls{dyn_sys.opt_robot_control(z0)};
quiver(z0{1}, z0{2}, u_R_opt(1), u_R_opt(2), 'Color', [176, 0, 0]/255, 'linewidth', 2);
for i=1:numel(dyn_sys.controls)
    u_H = dyn_sys.controls{i};
    znext = dyn_sys.dynamics(z0,u_H);
    pu1 = dyn_sys.pugivenxtheta(u_H, u_R_opt, z0, dyn_sys.q_funs_H{1});
    pu2 = dyn_sys.pugivenxtheta(u_H, u_R_opt, z0, dyn_sys.q_funs_H{2});
    b = znext{3};
    
    dx = znext{1}-z0{1};
    dy = znext{2}-z0{2};
    quiver(z0{1}+u_R_opt(1), z0{2}+u_R_opt(2), u_H(1), u_H(2), 'Color', [0, 106, 176]/255);
    t1 = text(z0{1} + 1.0*dx, ...
            z0{2} + 1.0*dy, ...
            0.55, sprintf('pu1=%f\npu2=%f\nb=%f', pu1,pu2,b));
    t1.FontSize = 11;
end

xlim([params.reward_info.g.min(1), params.reward_info.g.max(1)]);
ylim([params.reward_info.g.min(2), params.reward_info.g.max(2)]);

k = params.reward_info.k;
plot([k k],[params.reward_info.g.min(2) params.reward_info.g.max(2)]);

goal = params.goal;
scatter(goal(1), ...
        goal(2), ...
        'linewidth', 2, ...
        'marker', 'o')

us
mat
%% Test dynamics II
u = v*[0,0];
dyn_sys.dynamics(z0,u)

u = v*[0,1];
dyn_sys.dynamics(z0,u)

u = v*[-1,1];
dyn_sys.dynamics(z0,u)

%% Plot optimal robot policy
dyn_sys.plot_opt_robot_policy_from_x0(z0,1)
dyn_sys.plot_opt_robot_policy_from_x0(z0,2)

%% Plot robot value functions
for i=1:2
    figure;
    s = surf(g_phys.xs{1}, g_phys.xs{2}, dyn_sys.v_funs_R{i}.data);
    shading interp
end

%% Calculate Q-fun values for different u's and thetas
u = v*[0,1];
q1 = dyn_sys.q_funs_R{1}(num2str(u));
q2 = dyn_sys.q_funs_R{2}(num2str(u));

q1.GetDataAtReal(z0)
q2.GetDataAtReal(z0)

u = v*[-1,1];
q1 = dyn_sys.q_funs_R{1}(num2str(u));
q2 = dyn_sys.q_funs_R{2}(num2str(u));

q1.GetDataAtReal(z0)
q2.GetDataAtReal(z0)