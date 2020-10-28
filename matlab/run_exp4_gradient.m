%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

%% Load all the parameters for this computation!
params = exp4_gradient();

%% Sanity check -- Plot the opt control policies?
% params.dyn_sys.plot_opt_policy(1);
% params.dyn_sys.plot_opt_policy(2);
% params.dyn_sys.plot_opt_policy(3);

%% Plot optimal control policy starting from initial condition.
% params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 1);
% params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 2);
% params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 3);

% all_initial_states = {{1.74359,-1.94872,0.05}, ...
%                        {1.74359,-1.94872,0.25}, ... 
%                        {1.74359,-1.94872,0.5}, ... 
%                        {1.74359,-1.94872,0.75}, ...
%                        {1.74359,-1.94872,0.9}, ...
%                        {-2.815, -1.037, 0.05}, ...
%                        {-2.815, -1.037, 0.25}, ...
%                        {-2.815, -1.037, 0.5}, ...
%                        {-2.815, -1.037, 0.75}, ...
%                        {-2.815, -1.037, 0.9}, ...
%                        {-1.333, -3.407, 0.05}, ...
%                        {-1.333, -3.407, 0.25}, ...
%                        {-1.333, -3.407, 0.5}, ...
%                        {-1.333, -3.407, 0.75}, ...
%                        {-1.333, -3.407, 0.9}, ...
%                        {-0.444, 1.185, 0.05}, ...
%                        {-0.444, 1.185, 0.25}, ...
%                        {-0.444, 1.185, 0.5}, ...
%                        {-0.444, 1.185, 0.75}, ...
%                        {-0.444, 1.185, 0.9}};

% all_initial_states = {{-0.444, 1.185, 0.1}, ...
%                         {-0.444, 1.185, 0.25}, ...
%                         {-0.444, 1.185, 0.5}, ...
%                         {-0.444, 1.185, 0.75}, ...
%                         {-0.444, 1.185, 0.9}};
 
all_initial_states = {{1.74359,-1.94872, 0.1}, ...
                        {1.74359,-1.94872, 0.25}, ...
                        {1.74359,-1.94872, 0.5}, ...
                        {1.74359,-1.94872, 0.75}, ...
                        {1.74359,-1.94872, 0.9}};
                    
all_ttes = {}; 
all_value_funs = {};
for i=1:numel(all_initial_states) 
    fprintf('Computing TTE for %d / %d ....\n', i, numel(all_initial_states));
    
    init_state = all_initial_states{i};
    center = cell2mat(init_state);
    widths = [0.5,0.5,0.05];
    params.initial_value_fun = shapeRectangleByCenter(params.g, center, widths);
    params.extraArgs.targets = params.initial_value_fun;

    %% Solve for the discrete-time value function!
    [value_funs, tauOut, extraOuts] = ...
        DiscTimeHJIPDE_solve(params.initial_value_fun, ...
                             params.tau, ...
                             params.schemeData, ...
                             params.minWith, ...
                             params.extraArgs);
    
    all_value_funs{end+1} = value_funs;             
    
%     found_tte = false;                     
%     for j=numel(value_funs):-1:1
%         [gOut, dataOut] = proj(params.g, value_funs{j}, [1,1,0], 'min');
%         if all(dataOut(1:end) < 0)
%             tte = (numel(value_funs) - i)*params.dt;
%             all_ttes{end+1} = tte;
%             found_tte = true;
%             fprintf('found tte for init_cond (th = %f) : %f\n', init_state{3}, tte);
%             break;
%         end
%     end
%     if ~found_tte
%         all_ttes{end+1} = Inf;
%         fprintf('did *not* find tte for init_cond (th = %f)\n', init_state{3});
%     end
end

save('exp4_gradient_value_funs_interestingInits2.mat', 'params', 'all_value_funs', 'all_initial_states');

% if isfield(params, "save") && params.save
%     repo = what('pred_analyzer');
%     data_path = strcat(repo.path, '/matlab/data/');
%     filename = strcat(data_path,"exp4/exp4_gradient_",num2str(cell2mat(params.initial_state)),...
%         "_theta_",num2str(params.trueTheta),"_beta",num2str(params.beta),".mat");
%     save(filename, "value_funs", "params", "tauOut", "extraOuts");
% end

%% Visualize Projection of Value Funs
% row_len = 5;
% figure;
% for i=1:numel(value_funs)
%     h = subplot(floor(numel(value_funs)/row_len) + 1,row_len,i);
%     axis square
%     [gOut, dataOut] = proj(params.g, value_funs{numel(value_funs)-numel(value_funs)+i}, [1,1,0], 'min');
%     visSetIm(gOut, dataOut);
%     xlim([0,1]);
%     ylim([-0.4,0.4]);
%     t = title(['t=',num2str((i-1)),' s'], 'Interpreter', 'Latex');
%     t.FontSize = 18;
%     grid on;
% end
% 
% row_len = 5;
% figure;
% for i=1:numel(value_funs)
%     hold on;
%     h = subplot(floor(numel(value_funs)/row_len) + 1,row_len,i);
%     axis square
%     [gOut, dataOut] = proj(params.g, value_funs{numel(value_funs)-numel(value_funs)+i}, [0,0,1], 'min');
%     visSetIm(gOut, dataOut);
%     xlim([-4,4]);
%     ylim([-4,4]);
%     t = title(['t=',num2str((i-1)),' s'], 'Interpreter', 'Latex');
%     t.FontSize = 18;
%     grid on;
%     hold off;
% end


function plotControlValues(ctrls,dynSys)
    vels = [];
    ang_vels = [];
    for i=1:numel(ctrls)
        ctrl = dynSys.get_ctrl(ctrls{i});
        vels = [vels ctrl(1)];
        ang_vels = [ang_vels ctrl(2)];
    end

    figure
    plot(vels)
    xlabel("time")
    ylabel("velocity")

    figure
    plot(ang_vels)
    xlabel("time")
    ylabel("angular velocity")
end

function valid = check_traj(traj, obs)
    [num_states, t_len] = size(traj);
    c = cell(1, num_states);
    valid = 1;
    for i=1:t_len
        s = traj(:,i);
        c{1} = s(1);
        c{2} = s(2);
        c{3} = s(3);
        if obs.GetDataAtReal(c) == 1
            valid = 0;
            break;
        end
    end
end
