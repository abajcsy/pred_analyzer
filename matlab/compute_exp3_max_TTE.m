%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

% 'compute' or 'analysis'
mode = 'analysis';

if strcmp(mode, 'compute')
    %% Load all the parameters for this computation!
    %  See possible configuration files and create new ones in /matlab/config/
    params = exp3_reachability();

    %% Plot optimal control policy starting from initial condition.
    %params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 1);
    %params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 2);

    %% Solve for the discrete-time value function!
    fprintf('Computing value function for true theta = %d ...\n', params.trueThetaIdx);
    [value_funs, tauOut, extraOuts] = ...
        DiscTimeHJIPDE_solve(params.initial_value_fun, ...
                             params.tau, ...
                             params.schemeData, ...
                             params.minWith, ...
                             params.extraArgs);

    save(strcat('max_tte_g', num2str(params.trueThetaIdx), ...
                '_uth',num2str(params.uThresh),'.mat'), ...
        'value_funs', 'tauOut', 'params');

else
    %% IF PRECOMPUTED VALUE FUNCTION:
    %load('max_tte_g1.mat');
    
    %load('max_tte_g1_uth033.mat');
    %load('max_tte_g2_uth033.mat');
    
    load('max_tte_g1_uth0.27.mat');
    %load('max_tte_g2_uth0.27.mat');
    
    %load('max_tte_g1_uth0.25.mat');
    %load('max_tte_g2_uth0.25.mat');
    
%     figure
%     grid = Grid(params.gmin, params.gmax, params.gnums);
%     g3d = createGrid(params.gmin(1:3), params.gmax(1:3), [params.gnums(1:2), 30], 3);
%     for i=1:params.dyn_sys.num_ctrls
%         clf
%         hold on
%         quiver(params.initial_state{1}, params.initial_state{2}, cos(params.initial_state{3}), sin(params.initial_state{3}), 'k');
%         ui = params.dyn_sys.controls{i};
%         znext = params.dyn_sys.physical_dynamics(params.initial_state, ui);
%         %znext_bin = eval_u(g3d, g3d.xs, [znext{1}, znext{2}, znext{3}], 'nearest');
%         %znext_bin = {znext_bin(1), znext_bin(2), znext_bin(3)};
%         znext_bin = grid.RealToCoords(znext);
%         rand_c = [rand, rand, rand];
%         quiver(znext{1}, znext{2}, cos(znext{3}), sin(znext{3}), 'color', rand_c, 'linewidth', 2)
%         quiver(znext_bin{1}, znext_bin{2}, cos(znext_bin{3}), sin(znext_bin{3}), 'color', rand_c)
%         bla = 1;
%     end

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

        fprintf('ttl for x0={%f, %f, %f, %f}: %f\n', ...
            init_state{1}, init_state{2}, init_state{3}, init_state{4}, ...
            (length(tauOut)-tEarliest+1-1)*params.dt);
        
        if params.trueThetaIdx == 1
            fprintf('TTL from opt policy for g%d: %f\n', params.trueThetaIdx, (length(z_traj_g1)-1)*params.dt);
        else
            fprintf('TTL from opt policy for g%d: %f\n', params.trueThetaIdx, (length(z_traj_g2)-1)*params.dt);
        end
    end
         
end
                     