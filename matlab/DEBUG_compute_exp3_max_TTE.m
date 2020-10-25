%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

%% IF PRECOMPUTED VALUE FUNCTION:
%load('max_tte_g1.mat');

%load('max_tte_g1_uth033.mat');
%load('max_tte_g2_uth033.mat');

%load('max_tte_g1_uth0.27.mat');
%load('max_tte_g2_uth0.27.mat');

load('max_tte_g1_uth0.25.mat');
%load('max_tte_g2_uth0.25.mat');

ctrl_names = {'STOP', 'FORWARD', 'FORWARD_CW', 'FORWARD_CCW'};

% Sanity check controls. 
for ui=1:length(params.dyn_sys.controls)
    u = params.dyn_sys.controls{ui};
    z = {6, 1.83, pi, 0.1};
    q_fun_g2 = params.dyn_sys.q_funs{2};
    q_fun_g1 = params.dyn_sys.q_funs{1};
    pug2 = params.dyn_sys.pugivenxtheta(u, z, q_fun_g2);
    pug1 = params.dyn_sys.pugivenxtheta(u, z, q_fun_g1);
    fprintf('p(u=%s | x, g1) = %f\n', ctrl_names{ui}, pug1);
    fprintf('p(u=%s | x, g2) = %f\n', ctrl_names{ui}, pug2);
end

%% All initial state and priors over the two goals.                      
all_h_x0s = {{6, 1.83, pi, 0.5}, ...
             {6, 1.83, pi, 0.2}, ...
             {6, 1.83, pi, 0.8}};  

grid = Grid(params.gmin, params.gmax, params.gnums); % for converting from real to linear index         
for i=1:length(all_h_x0s)
    init_state = all_h_x0s{i};
    upper = tauOut(end);
    lower = 1;
    
    % Get the optimal policy trajectory. 
    z_traj_g1 = params.dyn_sys.get_opt_policy_joint_traj(grid, init_state, ...
                                                        1, 0.9);
    z_traj_g2 = params.dyn_sys.get_opt_policy_joint_traj(grid, init_state, ...
                                                        2, 0.1);       
%     for j=length(tauOut):-1:1
%         data = value_funs{j};
%         [gOut, dataOut] = proj(params.g, data, [0,0,0,1], init_state{4});
%         clf
%         visSetIm(gOut, dataOut);
%         hold on; 
%         scatter3(init_state{1}, init_state{2}, init_state{3}, 'k', 'filled')
%         xlim([-7.5,7.5]);
%         ylim([-7.5,7.5]);
%         bla = 1;
%     end

    % Determine the earliest time that the current state is in the reachable set
    tEarliest = findEarliestBRSInd(grid, value_funs, init_state, upper, lower);
    
%     % Get the optimal ADVERSARIAL trajectory. 
%     fprintf('Computing optimal ADVERSARIAL trajectory...\n');
%     [traj, traj_tau, ctrls] = computeOptTraj(init_state, params.g, value_funs,...
%                                         tauOut, params.dyn_sys, params.uMode);
%     
%     % Plot stuff to debug!
%     figure
%     plot3(traj(1,:), traj(2,:), traj(4,:), 'o-b')
%     hold on
%     if params.trueThetaIdx == 1
%         opt_traj = z_traj_g1;
%     else
%         opt_traj = z_traj_g2;
%     end
%     for j=2:length(opt_traj)
%         sprev = opt_traj{j-1};
%         snext = opt_traj{j};
%         plot3([snext{1}, sprev{1}], ...
%                [snext{2}, sprev{2}], ...
%                [snext{4}, sprev{4}], 'o-k')
%     end
%     xlim([params.gmin(1), params.gmax(1)])
%     ylim([params.gmin(2), params.gmax(2)])

    fprintf('ttl for x0={%f, %f, %f, %f}: %f\n', ...
        init_state{1}, init_state{2}, init_state{3}, init_state{4}, ...
        (length(tauOut)-tEarliest+1)*params.dt);

    if params.trueThetaIdx == 1
        fprintf('TTL from opt policy for g%d: %f\n', params.trueThetaIdx, length(z_traj_g1)*params.dt);
    else
        fprintf('TTL from opt policy for g%d: %f\n', params.trueThetaIdx, length(z_traj_g2)*params.dt);
    end
end
                      