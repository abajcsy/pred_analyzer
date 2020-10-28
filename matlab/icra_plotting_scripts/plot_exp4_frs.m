clear all
close all

%% Plots the gradient results.

% Load the results
%load('exp4_gradient_1.7436     -1.9487         0.9_theta_0.1_beta0.5.mat');
%load('exp4_gradient_1.7436     -1.9487         0.5_theta_0.1_beta0.5.mat');
%load('exp4_gradient_1.7436     -1.9487        0.25_theta_0.1_beta0.5.mat');
%load('exp4_gradient_1.7436     -1.9487           0_theta_0.1_beta0.5.mat');
% 
% z = {-0.444, 1,185, 0.05};
% for i =1:length(params.dyn_sys.controls)
%     ui = params.dyn_sys.controls{i};
%     validAction = params.dyn_sys.checkValidAction(z,ui);
%     fprintf('is it valid for u=[%f, %f]? %d\n', ui(1), ui(2), validAction);
% end

% load('exp4_gradient_all_ttes_LARGE.mat')
% 
% %% Visualize time it takes to learn all thetas.
% figure(1)
% hold on
% all_thetas = [];
% for i=1:numel(all_initial_states)
%     state = all_initial_states{i};
%     thinit = state{3};
%     all_thetas(end+1) = thinit;
%     tte = all_ttes{i};
%     if isinf(tte)
%         plot([0, 10], [thinit, thinit], 'r-o');
%     else
%         plot([0, tte], [thinit, thinit], 'r-o');
%     end
% end
% xlim([0,5]);
% ylim([0,0.98]);
% %yticks(all_thetas)
% set(gcf, 'color', 'w');
% set(gcf, 'position', [0,0,800,200])
% xlabel('time to learn (TTL) (s)', 'interpreter', 'latex', 'fontsize', 16)
% ylabel('$\theta^0$', 'interpreter', 'latex', 'fontsize', 16)
% 
% bla = 1;

%load('exp4_gradient_value_funs.mat');
%load('exp4_gradient_value_funs_cornerCase.mat');
load('exp4_gradient_value_funs_interestingInits.mat');

initial_state = all_initial_states{1};
value_funs = all_value_funs{1};

%% Plot the environment
% plot obstacle.
figure(1)
hold on
bandw_cmap = [0,0,0;1,1,1]; 
colormap(bandw_cmap)
ph = pcolor(params.reward_info.obstacles.g{1}, ...
            params.reward_info.obstacles.g{2}, ...
            params.raw_occupancy_map);
set(ph, 'EdgeColor', 'none');

%% Plot the zero level set of the value functions foward in time.
start_c = [0,0,1];
end_c = [1,0,0];
rs = linspace(start_c(1), end_c(1), numel(value_funs));
gs = linspace(start_c(2), end_c(2), numel(value_funs));
bs = linspace(start_c(3), end_c(3), numel(value_funs));
colors = [rs', gs', bs'];
step = -5;
% for i=numel(value_funs):step:1
%     figure(1)
%     h1 = visSetIm(params.g, value_funs{i}, colors(numel(value_funs)-i+1, :));
%     h1.FaceAlpha = (1-i/numel(value_funs))*0.01 + (i/numel(value_funs))*1.0;
%     xlim([params.gmin(1), params.gmax(1)]);
%     ylim([params.gmin(2), params.gmax(2)]);
%     zlim([params.gmin(3), params.gmax(3)]);
%     box on
% end
set(gca,'xtick',[])
set(gca,'ytick',[])
set(gcf, 'color', 'w');
% plot the goal
figure(1)
% pos = [params.reward_info.goal(1) - params.reward_info.goalRad, ...
%             params.reward_info.goal(2) - params.reward_info.goalRad, ...
%             params.reward_info.goalRad*2, params.reward_info.goalRad*2];
% r = rectangle('Position', pos, 'Curvature', [1,1]);
% r.FaceColor = [255, 166, 166]/255.;
% r.EdgeColor = 'r';
scatter(params.reward_info.goal(1), params.reward_info.goal(2), 50, 'r', 'filled');
gt = text(params.reward_info.goal(1), params.reward_info.goal(2), 0.1, 'goal');
gt.Color = 'r';
gt.FontSize = 12;
gt.BackgroundColor = 'w';
zlabel('$\theta$', 'interpreter', 'latex', 'fontsize', 16);
xlabel('$x$', 'interpreter', 'latex', 'fontsize', 16);
ylabel('$y$', 'interpreter', 'latex', 'fontsize', 16);
t = title(strcat('Gradient Learning: Forward Reachable Set for t=', num2str((numel(value_funs)-1)*params.dt), ' s'), 'Interpreter', 'Latex');
t.FontSize = 16;
% initial state plotting
scatter(initial_state{1}, initial_state{2}, 50, 'b', 'filled');
it = text(initial_state{1}-2, initial_state{2}-2, 0.1, strcat('[',num2str(initial_state{1}), ',', num2str(initial_state{2}), ',', num2str(initial_state{3}), ']'));
it.Color = 'b';
it.FontSize = 12;
it.BackgroundColor = 'w';
% plot lines.
plot3([initial_state{1}, initial_state{1}], ...
        [initial_state{2}, initial_state{2}], ...
        [0, initial_state{3}], '--b');
set(gcf, 'position', [0,0,700,500]);
view(40,38)

%% Plot the set of reachable thetas over time. 
times = 0:params.dt:params.dt*length(value_funs);
for i=length(value_funs):-1:1
    figure(2)
    hold on
    [gOut, dataOut] = proj(params.g, value_funs{i}, [1,1,0], 'min');
    reachable_idxs = find(dataOut <= 0);
    reachable_thetas = gOut.xs{1}(reachable_idxs);
    s1 = scatter(times(length(value_funs)-i+1)*ones(1,length(reachable_thetas')), reachable_thetas', ...
        'markerfacecolor', colors(numel(value_funs)-i+1, :), ...
        'markeredgecolor', colors(numel(value_funs)-i+1, :));
    s1.MarkerFaceAlpha = 1;
    
end
set(gcf, 'color', 'w');
xticks(0:params.dt*2:params.dt*length(value_funs))
yticks([0, 0.25, 0.5, 0.75, 0.98]);
ylim([0,1]);
ylabel('$\theta^0$', 'interpreter', 'latex', 'fontsize', 16);
xlabel('$time (s)$', 'interpreter', 'latex', 'fontsize', 16);
box on;
set(gcf, 'position', [0,0,700,200])


% %% Visualize Projection of Value Funs
% % Visualize the projection onto the theta-dimension.
% row_len = 5;
% figure;
% for i=1:numel(value_funs)
%     h = subplot(floor(numel(value_funs)/row_len) + 1,row_len,i);
%     axis square
%     [gOut, dataOut] = proj(params.g, value_funs{numel(value_funs)-numel(value_funs)+i}, [1,1,0], 'min');
%     visSetIm(gOut, dataOut);
%     xlim([0,1]);
%     ylim([-0.4,0.4]);
%     %t = title(['t=',num2str((i-1)*params.dt),' s'], 'Interpreter', 'Latex');
%     %t.FontSize = 18;
%     grid on;
% end
% 
% % Visualize the projection of theta onto the xy-dimension.
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
%     t = title(['t=',num2str((i-1)*params.dt),' s'], 'Interpreter', 'Latex');
%     t.FontSize = 18;
%     grid on;
%     hold off;
% end