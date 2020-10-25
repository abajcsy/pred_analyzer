%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

%load('max_tte_g1_uth0.25_complex.mat')
%params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 1);
%params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 2);

% params = exp3_reachability_complex();
%  
% figure
% grid = Grid(params.gmin(1:3), params.gmax(1:3), params.gnums(1:3));
% g3d = createGrid(params.gmin(1:3), params.gmax(1:3), params.gnums(1:3), 3);
% z_bin = params.initial_state; %grid.RealToCoords(params.initial_state);
% hold on
% scatter3(g3d.xs{1}(:), g3d.xs{2}(:), g3d.xs{3}(:), 'b')
% scatter3(grid.g{1}(:), grid.g{2}(:), grid.g{3}(:), 'k');
% for i=1:params.dyn_sys.num_ctrls
%     %clf
%     hold on
%     quiver(z_bin{1}, z_bin{2}, cos(z_bin{3}), sin(z_bin{3}), 'k');
%     ui = params.dyn_sys.controls{i};
%     znext = params.dyn_sys.physical_dynamics(z_bin, ui);
%     znext_bin_gstruct = eval_u(g3d, g3d.xs, [znext{1}, znext{2}, znext{3}], 'nearest');
%     znext_bin_gstruct = {znext_bin_gstruct(1), znext_bin_gstruct(2), znext_bin_gstruct(3)};
%     znext_bin = grid.RealToCoords(znext);
%     rand_c = [rand, rand, rand];
%     quiver(znext{1}, znext{2}, cos(znext{3}), sin(znext{3}), 'color', rand_c, 'linewidth', 2)
%     quiver(znext_bin{1}, znext_bin{2}, cos(znext_bin{3}), sin(znext_bin{3}), 'color', rand_c)
%     scatter3(znext{1}, znext_bin{2}, znext_bin{3}, 'markerfacecolor', rand_c)
%     scatter3(znext{1}, znext{2}, znext{3}, 'markerfacecolor', rand_c, 'markeredgecolor', 'r')
%     %quiver(znext_bin_gstruct{1}, znext_bin_gstruct{2}, cos(znext_bin_gstruct{3}), sin(znext_bin_gstruct{3}), 'color', 'r')
%     xlim([4.5,6.5])
%     ylim([0.5,3])
%     view(0,90)
%     bla = 1;
% end

% 'compute' or 'analysis'
mode = 'analysis';

if strcmp(mode, 'compute')
    %% Load all the parameters for this computation!
    %  See possible configuration files and create new ones in /matlab/config/
    params = exp3_reachability_complex();

    %% Plot optimal control policy starting from initial condition.
    %params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 1);
    %params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 2);
    
    % Sanity check controls. 
%     ctrl_names = {'STOP', ...
%                'SLOW_FWD_CW', ...
%                'SLOW_FWD', ...
%                'SLOW_FWD_CCW', ...
%                'FAST_FWD_CW', ...
%                'FAST_FWD', ...
%                'FAST_FWD_CCW'};
%     for ui=1:length(params.dyn_sys.controls)
%         u = params.dyn_sys.controls{ui};
%         z = {6, 1.83, pi, 0.5};
%         
%         q_fun_g2 = params.dyn_sys.q_funs{2};
%         q_fun_g1 = params.dyn_sys.q_funs{1};
%         pug2 = params.dyn_sys.pugivenxtheta(u, z, q_fun_g2);
%         pug1 = params.dyn_sys.pugivenxtheta(u, z, q_fun_g1);
%         fprintf('p(u=%s | x, g1) = %f\n', ctrl_names{ui}, pug1);
%         fprintf('p(u=%s | x, g2) = %f\n', ctrl_names{ui}, pug2);
%     end

    %% Solve for the discrete-time value function!
    fprintf('Computing value function for true theta = %d ...\n', params.trueThetaIdx);
    [value_funs, tauOut, extraOuts] = ...
        DiscTimeHJIPDE_solve(params.initial_value_fun, ...
                             params.tau, ...
                             params.schemeData, ...
                             params.minWith, ...
                             params.extraArgs);

    save(strcat('TEST_max_tte_g', num2str(params.trueThetaIdx), ...
                '_uth',num2str(params.uThresh),'_complex.mat'), ...
        'value_funs', 'tauOut', 'params', 'extraOuts');

else
    %% IF PRECOMPUTED VALUE FUNCTION:
    
    %load('max_tte_g2_uth0.4_complex_v46.mat');
    
    %load('max_tte_g1_uth0.35_complex_v46.mat');
    %load('max_tte_g2_uth0.35_complex_v46.mat');
    
    load('TEST_max_tte_g2_uth0.25_complex.mat');
    
    %load('max_tte_g2_uth0.39_complex.mat');
    
    ctrl_names = {'STOP', ...
                   'SLOW_FWD_CW', ...
                   'SLOW_FWD', ...
                   'SLOW_FWD_CCW', ...
                   'FAST_FWD_CW', ...
                   'FAST_FWD', ...
                   'FAST_FWD_CCW'};

    % Sanity check controls. 
    for ui=1:length(params.dyn_sys.controls)
        u = params.dyn_sys.controls{ui};
        z = {6, 1.83, pi, 0.5};
        
        q_fun_g2 = params.dyn_sys.q_funs{2};
        q_fun_g1 = params.dyn_sys.q_funs{1};
        pug2 = params.dyn_sys.pugivenxtheta(u, z, q_fun_g2);
        pug1 = params.dyn_sys.pugivenxtheta(u, z, q_fun_g1);
        fprintf('p(u=%s | x, g1) = %f\n', ctrl_names{ui}, pug1);
        fprintf('p(u=%s | x, g2) = %f\n', ctrl_names{ui}, pug2);
        
        bnext = params.dyn_sys.belief_update(u,z);
        fprintf('b_next(g1 | u=%s) = %f\n', ctrl_names{ui}, bnext);
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
        
        for j=length(tauOut):-1:1
            data = value_funs{j};
            [gOut, dataOut] = proj(params.g, data, [0,0,0,1], init_state{4});
            clf
            visSetIm(gOut, dataOut);
            hold on; 
            scatter3(init_state{1}, init_state{2}, init_state{3}, 'k', 'filled')
            xlim([-7.5,7.5]);
            ylim([-7.5,7.5]);
            bla = 1;
        end

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
                     