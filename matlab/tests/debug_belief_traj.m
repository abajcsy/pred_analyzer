clear all
close all

%% Load data.
repo = what('pred_analyzer');
[p, ~] = repo.path;
% filename = 'e1_finer_grid_brt_max_uthresh013.mat';
% filename = 'e2_b50_finer_grid_brt_min_uthresh013.mat';
% filename = 'e2_b50_finer_grid_brt_max_uthresh013.mat';
filename = 'e2_b50_finer_grid_brt_min_uthresh0.mat';
load(strcat(p, '/matlab/data/', filename));

initial_state = schemeData.dynSys.z0;
extraArgsCtrl.interpolate = false;
[traj_tmp, traj_tau_tmp] = computeOptTraj(initial_state, g, value_funs, tau, schemeData.dynSys, uMode, extraArgsCtrl);
%% Plot optimal traj from reachability. 
plotOptTraj(traj_tmp, traj_tau_tmp, thetas, trueThetaIdx, ...
    gmin, gmax, gnums, 0, value_funs, extraPltArgs)
title(strcat("Test opt traj: T=", num2str(length(traj_tmp))));


%% Get dynamical system and controls.
dynSys = schemeData.dynSys;
% controls{1} = STOP
% controls{2} = DOWN
% controls{3} = UP
% controls{4} = LEFT
% controls{5} = LEFT_DOWN
% controls{6} = LEFT_UP
% controls{7} = RIGHT
% controls{8} = RIGHT_DOWN
% controls{9} = RIGHT_UP
controls_text = {'S', 'D', 'U', 'L', 'LD', 'LU', 'R', 'RD', 'RU'};
controls = dynSys.controls;
uThresh = dynSys.uThresh;

%% Setup init cond
z = dynSys.z0; %{0,0,0.5};

%% Plot the action's and their effect on belief.
figure;
hold on
scatter3(z{1}, z{2}, z{3}, 'r');
% text(z{1}, z{2}+0.025, z{3}, 'z0');
for i=1:length(controls)
    u = controls{i};
    % get probability of this action under true goal.
    pu = dynSys.pugivenxtheta(u, z, thetas{trueThetaIdx});
    
    % propagate dynamics.
    znext = dynSys.dynamics(z,u);
    
    % plot
    if pu >= uThresh
        c = 'k';
        plot3([z{1}, znext{1}], [z{2}, znext{2}], [z{3}, znext{3}], '--o', ...
                'Color', c, ...
                'markeredgecolor', c, ...
                'markerfacecolor', 'none');
        scatter3(znext{1}, znext{2}, znext{3}, c, 'filled');
    else
        c = [0.6,0.6,0.6];
        p = plot3([z{1}, znext{1}], [z{2}, znext{2}], [z{3}, znext{3}], '--o', ...
                'Color', c, ...
                'markeredgecolor', c, ...
                'markerfacecolor', 'none');
    end
    t = text(znext{1}, znext{2}-0.025, znext{3}, strcat('p(',controls_text{i},'):',num2str(pu)));
    t.Color = [0.6,0.6,0.6];
    tb = text(znext{1}, znext{2}+0.025, znext{3}, strcat('b:',num2str(znext{3})));
end
xlabel('x');
ylabel('y');
zlabel('b(g1 | x, u)');
set(gcf,'color','w');
grid on
hold off

%% Plot optimal traj from reachability. 
plotOptTraj(traj, traj_tau, thetas, trueThetaIdx, ...
    gmin, gmax, gnums, 0, value_funs, extraPltArgs)
title(strcat("Reachability opt traj: T=", num2str(length(traj))));

%% Input candidate control sequence and then plot the optimal traj. 

if strcmp(uMode, 'min')
%     u_seq = {3,3,3,3,3,3,3,1}; % this one reaches target
    u_seq = {3,3,3,3,3,3,3}; % this one does not
elseif strcmp(uMode, 'max')
    u_seq = {9,9,9,9,9,6,6,4,4,4};
else
    error("no matching uMode!");
end

z_traj = zeros(3, length(u_seq)+1);
z_traj(1:3, 1) = cell2mat(z)';
for i=1:length(u_seq)
    % propagate dynamics.
    uidx = u_seq{i};
    znext = dynSys.dynamics(z,controls{uidx});
    z_traj(1:3, i+1) = cell2mat(znext)';
    z = znext;
end
z_traj_tau = 1:length(z_traj);

plotOptTraj(z_traj, z_traj_tau, thetas, trueThetaIdx, ...
    gmin, gmax, gnums, 0, value_funs, extraPltArgs)
title(strcat("Hand-coded traj: T=", num2str(length(z_traj))));

%% Maximize the Qfunction.
zmax_init = {-2,0,0.5}; 
zmax = zmax_init;
z_traj_max = [];
z_traj_max(1:3, end+1) = cell2mat(zmax_init)';
while zmax{3} < 0.8
    max_b = -1;
    max_u = 0;
    for i=1:length(controls)
        u = controls{i};

        % get probability of this action under true goal.
        pu = dynSys.pugivenxtheta(u, zmax, thetas{trueThetaIdx});

        % propagate dynamics.
        bnext = dynSys.belief_update(u,zmax);
        
        if bnext > max_b
            max_b = bnext;
            max_u = u;
        end
    end
    % propagate dynamics.
    znext = dynSys.dynamics(zmax,max_u);
    z_traj_max(1:3, end+1) = cell2mat(znext)';
    zmax = znext;
end
z_traj_max_tau = 1:length(z_traj_max);

plotOptTraj(z_traj_max, z_traj_max_tau, thetas, trueThetaIdx, ...
    gmin, gmax, gnums, 0, value_funs, extraPltArgs)
title(strcat("Control Which Maximizes Next Belief: T=", num2str(length(z_traj_max))));
