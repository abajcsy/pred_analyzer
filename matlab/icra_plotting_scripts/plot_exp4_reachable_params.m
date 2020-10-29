clear all
clf
close all

%% Plots the gradient results.
%load('exp4_gradient_value_funs_interestingInits.mat');
%load('exp4_gradient_value_funs_interestingInits2.mat');

%load('exp4_gradient_value_funs_interestingInits_NEW.mat');
load('exp4_gradient_value_funs_interestingInits_biggerObs.mat');

% plot opt policy.
% initial_state = all_initial_states{1};
% initial_state_low = {initial_state{1}, initial_state{2}, 0.02};
% initial_state_up = {initial_state{1}, initial_state{2}, 0.98};
% plot_opt_policy(initial_state_low, params)
% plot_opt_policy(initial_state_up, params)
    
% Color setup.
start_c = [0,0,1];
end_c = [1,0,0];
rs = linspace(start_c(1), end_c(1), numel(params.tau));
gs = linspace(start_c(2), end_c(2), numel(params.tau));
bs = linspace(start_c(3), end_c(3), numel(params.tau));
colors = [rs', gs', bs'];

params_of_interest = 0.05:0.05:0.95; %0.05:0.02:0.95; 

tte_per_p0_pstar = zeros([length(all_initial_states), length(params_of_interest)]);

for thidx=1:length(all_initial_states) 
    initial_state = all_initial_states{thidx};
    value_funs = all_value_funs{thidx};
    
    % plot opt policy.
    initial_state{3} = 0.02;
    plot_opt_policy(initial_state, params)
   
    param_init = initial_state{3};
    for pi=1:length(params_of_interest)
        param = params_of_interest(pi);
        tte = compute_tte_for_param(value_funs, param, params.g, params.dt);
        tte_per_p0_pstar(thidx, pi) = tte;
    end
end

hold off
set(0,'defaulttextInterpreter','latex') 
figure
h = heatmap(tte_per_p0_pstar);

ax = gca;
newStr = split(num2str(params_of_interest));
xlabel_txt = {};
for i=1:numel(newStr)
    xlabel_txt{end+1} = newStr{i};
end
ax.XData = xlabel_txt;
ax.YData = {'0.1','0.25','0.5','0.75','0.9'};

h.XLabel = '\theta^*'; %'interpreter', 'latex', 'fontsize', 18);
h.YLabel = '\theta^0'; %'interpreter', 'latex', 'fontsize', 18);
h.GridVisible = 'off';
colormap plasma
h.Title = 'Minimum TTL True \theta^* for Initial \theta^0';
h.FontName = 'Times New Roman';
h.FontSize = 18;
h.ColorbarVisible = 'on';
set(gcf, 'color', 'w');
set(gcf, 'position', [0,0,1500,500])

% % setup colorbar
% axs = struct(gca); %ignore warning that this should be avoided
% cb = axs.Colorbar;
% cb.TickLabels = {'0', '0.5', '1', '1.5', '2', '2.5', '3', '3.5', '4', '4.5'};


%% Finds the time-to-estimate a specific parameter value.
function tte = compute_tte_for_param(value_funs, param, grid, dt) 
    for t=length(value_funs):-1:1
        [g1D, data1D] = proj(grid, value_funs{t}, [1,1,0], 'min');
        val = eval_u(g1D, data1D, param, 'nearest');
        if val <= 0
            tte = (length(value_funs)-t)*dt;
            break;
        end
    end
end

%% Plots the optimal policy for a given [x,y,theta]
function plot_opt_policy(initial_state, params)
    % plot environment.
    figure
    hold on
    bandw_cmap = [1,1,1;0,0,0]; 
    colormap(bandw_cmap)
    ph = pcolor(params.reward_info.obstacles.g{1}, ...
                params.reward_info.obstacles.g{2}, ...
                params.reward_info.obstacles.data);
    set(ph, 'EdgeColor', 'none');
    scatter(params.reward_info.goal(1), params.reward_info.goal(2), 50, 'r', 'filled');
    gt = text(params.reward_info.goal(1), params.reward_info.goal(2), 0.1, 'goal');
    gt.Color = 'r';
    gt.FontSize = 12;
    %gt.BackgroundColor = 'w';
    zlabel('$\theta$', 'interpreter', 'latex', 'fontsize', 16);
    xlabel('$x$', 'interpreter', 'latex', 'fontsize', 16);
    ylabel('$y$', 'interpreter', 'latex', 'fontsize', 16);
    t = title(strcat('Gradient Learning: Opt Policy for w = ', num2str(initial_state{3})), 'Interpreter', 'Latex');
    t.FontSize = 16;
    % initial state plotting
    scatter(initial_state{1}, initial_state{2}, 50, 'b', 'filled');
    %it = text(initial_state{1}-2, initial_state{2}-2, 0.1, strcat('[',num2str(initial_state{1}), ',', num2str(initial_state{2}), ',', num2str(initial_state{3}), ']'));
    %it.Color = 'b';
    %it.FontSize = 12;
    %it.BackgroundColor = 'w';
    
    % Get the optimal policy
    opt_traj = params.dyn_sys.get_opt_traj(initial_state, initial_state{3});
    hold on
    for i=2:length(opt_traj)
        plot([opt_traj(1,i), opt_traj(1,i-1)], [opt_traj(2,i), opt_traj(2,i-1)], 'r-o');
    end
    set(gcf, 'color', 'w')
    
end