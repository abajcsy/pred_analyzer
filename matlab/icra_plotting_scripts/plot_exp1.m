clear all
close all

%% Load in the data.
repo = what('pred_analyzer');
data_path = strcat(repo.path, '/matlab/data/');

% --- final data --- %
%load(strcat(data_path, 'exp_1_max_uthresh_0.15_theta_2.mat'))
%load(strcat(data_path, 'exp_1_max_uthresh_0.15_theta_1.mat'))
%load(strcat(data_path, 'exp_1_min_uthresh_0_theta_1.mat'))
%load(strcat(data_path, 'exp_1_min_uthresh_0_theta_2.mat'))

%load(strcat(data_path, 'exp_1_min_uthresh_0.15_theta_1.mat'))
load(strcat(data_path, 'exp_1_min_uthresh_0.15_theta_2.mat'))
% --- final data --- %


%% Get the optimal trajectory following the policy
grid = Grid(params.g.min, params.g.max, params.g.N); % for converting from real to linear index
% Determine the earliest time that the current state is in the reachable set
z0 = grid.RealToCoords(params.initial_state);
[opt_policy_traj, opt_ctrl_idxs] = ...
    params.dyn_sys.get_opt_policy_from_x0(z0, ...
                                          params.trueThetaIdx);
% Get the TTE for optimal policy.
[tte_idx, bfinal] = params.dyn_sys.get_opt_policy_earliest_tte_conf(opt_policy_traj, ...
        opt_ctrl_idxs, ...
        params.initial_state{end}, ...
        params.trueThetaIdx);
    
    
%% TEST!
for i=3:-1:1
    [gOut, dataOut] = proj(params.g, value_funs{end-i}, [0,0,1], 0.5); %); 'max');
    clf; visSetIm(gOut, dataOut)
    hold on
    scatter(traj(1,3-i+1), traj(2,3-i+1), 'r');
    bla = 1;
end
            
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
ropt = linspace(startColor(1), endColorOpt(1), length(opt_policy_traj));
gopt = linspace(startColor(2), endColorOpt(2), length(opt_policy_traj));
bopt = linspace(startColor(3), endColorOpt(3), length(opt_policy_traj));

% colors for legible/deceptive controls
traj(:,any(isnan(traj), 1)) = []; % remove any nans
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
%ph = pcolor(params.reward_info.obstacles.g{1}, ...
%    params.reward_info.obstacles.g{2}, ...
%    params.reward_info.obstacles.data);
set(ph, 'EdgeColor', 'none');

%% Plotting + font size info
fontsz = 20;
scattersz = 10;

% %% GET & PLOT THE TRAJECTORY DURING DISCHJIPDE SOLVE
% tauOut = params.tau;
% upper = tauOut(end); %tauOut(end);
% lower = 1;
% grid = Grid(params.g.min, params.g.max, params.g.N); % for converting from real to linear index
% % Determine the earliest time that the current state is in the reachable set
% z0 = grid.RealToCoords(params.initial_state);
% tEarliest = findEarliestBRSInd(grid, value_funs, z0, upper, lower);
% all_ctrls = extraOuts.all_opt_ctrl_idxs; 
% hjb_traj = zeros(3, upper-tEarliest+1);
% z = z0;
% t = tEarliest;
% hjb_traj(:,1) = cell2mat(z)';
% idx = 2;
% while t < tauOut(end)
%     uopt_idx = eval_u(params.g, all_ctrls{t}, cell2mat(z), 'nearest');
%     uopt = params.dyn_sys.controls{uopt_idx};
%     z = params.dyn_sys.dynamics(z, uopt);
%     hjb_traj(:,idx) = cell2mat(z)';
%     t = t+1;
%     idx = idx +1;
% end
% plot3(hjb_traj(1,:), hjb_traj(2,:), hjb_traj(3,:), 'o-r');

%% plot the "optimal policy"
for i=2:length(opt_policy_traj)
    c = [ropt(i), gopt(i), bopt(i)];
    plot([opt_policy_traj(1,i-1),opt_policy_traj(1,i)], ...
        [opt_policy_traj(2,i-1),opt_policy_traj(2,i)], ...
        'o-', 'Color', c, ...
        'markerfacecolor', 'w', ...
        'markeredgecolor', c, ...
        'linewidth', 2);
end
opt_ttl_txt_x = 1; %opt_traj(1,tte_idx);
opt_ttl_txt_y = 1; %opt_traj(2,tte_idx);
bval = bfinal;
if params.trueThetaIdx == 2
    bval = 1 - bval;
end
% Plot final belief and number of timesteps. 
btxt = strcat('b(g',num2str(params.trueThetaIdx),') =',num2str(bval));
t1 = text(opt_ttl_txt_x, opt_ttl_txt_y, btxt, 'Interpreter', 'latex', 'fontsize', fontsz);
t1.FontSize = fontsz;
t1.Color = [0.5,0.5,0.5]; %goalColors{params.trueThetaIdx};
t1.BackgroundColor = 'w';
tte_opt = (tte_idx-1)*params.dt;
mtxt = strcat('TTL=',num2str(tte_opt), ' s');
t1 = text(opt_ttl_txt_x, opt_ttl_txt_y+0.8, mtxt, 'Interpreter', 'latex', 'fontsize', fontsz);
t1.FontSize = fontsz;
t1.Color = [0.5,0.5,0.5]; %goalColors{params.trueThetaIdx};
t1.BackgroundColor = 'w';

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
legible_ttl_txt_x = 1; %traj(1,num_states)
legible_ttl_txt_y = 3; %traj(2,num_states);
% Plot final belief and number of timesteps. 
bval = traj(3,num_states);
if params.trueThetaIdx == 2
    bval = 1 - bval;
end
btxt = strcat('b(g',num2str(params.trueThetaIdx),') =',num2str(bval));
t1 = text(legible_ttl_txt_x, legible_ttl_txt_y, btxt, 'Interpreter', 'latex', 'fontsize', fontsz);
t1.FontSize = fontsz;
t1.Color = goalColors{params.trueThetaIdx};
t1.BackgroundColor = 'w';
mtxt = strcat('TTL=',num2str((num_states-1)*params.dt), ' s');
t1 = text(legible_ttl_txt_x, legible_ttl_txt_y+0.8, mtxt, 'Interpreter', 'latex', 'fontsize', fontsz);
t1.FontSize = fontsz;
t1.Color = goalColors{params.trueThetaIdx};
t1.BackgroundColor = 'w';

%% Plot the state at which optimal trajectory reaches high belief.
scatter(opt_policy_traj(1,tte_idx), opt_policy_traj(2,tte_idx), ...
    'markerfacecolor', [0.5,0.5,0.5], ...
    'markeredgecolor', [0.5,0.5,0.5], ...
    'linewidth', 2);

%% Plot goals
% overlay on top of it the most "legible policy"
for i=1:length(params.thetas)
    plot(params.thetas{i}(1), params.thetas{i}(2), ...
                'o', 'Color', goalColors{i}, ...
                'markeredgecolor', goalColors{i}, ...
                'markerfacecolor', goalColors{i}, ...
                'markersize', scattersz);
    gTxt = strcat('g',num2str(i));
    t1 = text(params.thetas{i}(1)+0.3, params.thetas{i}(2), gTxt, ...
        'Interpreter', 'latex', 'fontsize', fontsz);
    t1.FontSize = fontsz;
    t1.Color = goalColors{i};
end

xlim([params.gmin(1), params.gmax(1)]) %3]);
ylim([params.gmin(2), params.gmax(2)]) %3]);
set(gca,'xtick',[])
set(gca,'ytick',[])
ylabel('y', 'Interpreter', 'latex', 'fontsize', 20)
xlabel('x', 'Interpreter', 'latex', 'fontsize', 20)
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,700,700])
box on
grid off
