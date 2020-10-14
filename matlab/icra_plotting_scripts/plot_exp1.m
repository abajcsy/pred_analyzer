clear all
close all

%% Load in the data.
repo = what('pred_analyzer');
data_path = strcat(repo.path, '/matlab/data/');

%load(strcat(data_path, 'exp_1_min_uthresh_0_theta_1.mat'))
%load(strcat(data_path, 'exp_1_min_uthresh_0_theta_2.mat'))

%load(strcat(data_path, 'exp_1_min_uthresh_0.15_theta_1.mat'))
%load(strcat(data_path, 'exp_1_min_uthresh_0.15_theta_2.mat'))

%load(strcat(data_path, 'exp_1_min_uthresh_0_theta_1.mat'))
%load(strcat(data_path, 'exp_1_min_uthresh_0.05_theta_1.mat'))
%load(strcat(data_path, 'exp_1_min_uthresh_0.15_theta_1.mat'))
%load(strcat(data_path, 'exp_1_min_uthresh_0.2_theta_1.mat'))

load(strcat(data_path, 'exp_1_max_uthresh_0.15_theta_1.mat'))
%load(strcat(data_path, 'exp_1_min_uthresh_0.15_theta_1.mat'))

%params = exp1_legibility();

% Get the optimal trajectory following the policy
opt_traj = params.dyn_sys.get_opt_policy_from_x0(params.initial_state, params.trueThetaIdx);

% Setup colors.
startColor = [97., 76., 76.]/255.;
endColorRed = [255., 0., 0.]/255.;
endColorBlue = [38., 138., 240.]/255.;
endColorOptRed = [191, 151, 151]/255.;
endColorOptBlue = [112, 156, 219]/255.;
goalColors = {endColorRed, endColorBlue};
if params.trueThetaIdx == 1
    endColor = endColorRed;
    endColorOpt = endColorOptRed;
else
    endColor = endColorBlue;
    endColorOpt = endColorOptBlue;
end

% colors for goal-optimal controls
ropt = linspace(startColor(1), endColorOpt(1), length(opt_traj));
gopt = linspace(startColor(2), endColorOpt(2), length(opt_traj));
bopt = linspace(startColor(3), endColorOpt(3), length(opt_traj));

% colors for legible/deceptive controls
rreach = linspace(startColor(1), endColor(1), length(traj));
greach = linspace(startColor(2), endColor(2), length(traj));
breach = linspace(startColor(3), endColor(3), length(traj));

%% Create fig
figure(1)
hold on

%% Plot the environment
% plot obstacle.
bandw_cmap = [1,1,1;0,0,0]; 
colormap(bandw_cmap)
ph = pcolor(params.gimg.xs{1}, params.gimg.xs{2}, params.obs_map_full);
set(ph, 'EdgeColor', 'none');

%% plot the "optimal policy"
for i=2:length(opt_traj)
    c = [ropt(i), gopt(i), bopt(i)];
    plot([opt_traj(1,i-1),opt_traj(1,i)], ...
        [opt_traj(2,i-1),opt_traj(2,i)], ...
        'o-', 'Color', c, ...
        'markerfacecolor', 'w', ...
        'markeredgecolor', c, ...
        'linewidth', 2);
end

%% Plot the "legible trajectory"
[~, num_states] = size(traj);
for i=2:num_states
    c = [rreach(i), greach(i), breach(i)];
    plot([traj(1,i-1),traj(1,i)], ...
        [traj(2,i-1),traj(2,i)], ...
        'o-', 'Color', c, ...
        'markerfacecolor', c, ...
        'markeredgecolor', c, ...
        'linewidth', 2);
end
% Plot final belief and number of timesteps. 
btxt = strcat('b(g1) =',num2str(traj(3,num_states)));
t1 = text(traj(1,num_states)+0.3, traj(2,num_states), btxt, 'Interpreter', 'latex', 'fontsize', 18);
t1.FontSize = 12;
t1.Color = goalColors{params.trueThetaIdx};
mtxt = strcat('TTL=',num2str((num_states-1)*params.dt), ' s');
t1 = text(traj(1,num_states)+0.3, traj(2,num_states)+0.3, mtxt, 'Interpreter', 'latex', 'fontsize', 18);
t1.FontSize = 12;
t1.Color = goalColors{params.trueThetaIdx};

%% Plot goals
% overlay on top of it the most "legible policy"
for i=1:length(params.thetas)
    plot(params.thetas{i}(1), params.thetas{i}(2), ...
                'o', 'Color', goalColors{i}, ...
                'markeredgecolor', goalColors{i}, ...
                'markerfacecolor', goalColors{i});
    gTxt = strcat('g',num2str(i));
    t1 = text(params.thetas{i}(1)+0.3, params.thetas{i}(2), gTxt, 'Interpreter', 'latex', 'fontsize', 18);
    t1.FontSize = 12;
    t1.Color = goalColors{i};
end

xlim([params.gmin(1), params.gmax(1)])
ylim([params.gmin(2), params.gmax(2)])
set(gca,'xtick',[])
set(gca,'ytick',[])
ylabel('y', 'Interpreter', 'latex', 'fontsize', 20)
xlabel('x', 'Interpreter', 'latex', 'fontsize', 20)
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,700,700])
box on
grid off
