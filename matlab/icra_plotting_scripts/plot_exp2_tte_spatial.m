clear all
clf
close all

%% load data.
%load('cell_bounds.mat');
%load('overnight_rand.mat');
load('overnight_uniform.mat');

%% Load params. 
params = exp2_conf_reachability();

%% Create fig. 
figure
hold on
bandw_cmap = [1,1,1;0,0,0]; %[0,0,0;1,1,1]; %[1,1,1;0,0,0];
colormap(bandw_cmap)
ph = pcolor(params.gimg.xs{1}, params.gimg.xs{2}, params.obs_map_full);
%ph = pcolor(params.reward_info.obstacles.g{1}, params.reward_info.obstacles.g{2}, params.reward_info.obstacles.data);
set(ph, 'EdgeColor', 'none');

max_tte = max(cell2mat(all_ttes));
min_tte = min(cell2mat(all_ttes));
max_color = [228, 18, 94]/255.;
min_color = [113, 145, 209]/255.; %[129, 134, 147]/255.;
for i=1:length(all_ttes)
    tte = all_ttes{i};
    traj = all_trajs{i};
    alpha = (tte - min_tte)/(max_tte - min_tte);
    color = alpha * max_color + (1-alpha) * min_color;
    bounds = relevant_cell_bounds{i};
    rect_pos = [bounds(1), bounds(2), bounds(3)-bounds(1), bounds(4)-bounds(2)];
    r = rectangle('position', rect_pos, ...
            'curvature', [0.0,0.0], ...
            'FaceColor', [color, 0.5], ...
            'EdgeColor', 'none');
    s = scatter(traj(1),traj(2),20,color,'filled');
end
% plot modelled goal. 
scatter(params.theta(1), params.theta(2), 80,'r', 'linewidth', 3);

xlim([params.gmin(1),params.gmax(1)])
ylim([params.gmin(2),params.gmax(2)])
set(gca,'xtick',[])
set(gca,'ytick',[])
ylabel('y', 'Interpreter', 'latex', 'fontsize', 20)
xlabel('x', 'Interpreter', 'latex', 'fontsize', 20)
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,700,700])
box on
grid off

%% TODO: add in a cute colorbar. 
r = linspace(min_color(1), max_color(1), 4);
g = linspace(min_color(2), max_color(2), 4);
b = linspace(min_color(3), max_color(3), 4);
ax = axes;
colormap(ax, [r', g', b'])
colorbar
%lcolorbar([min_tte, 1.36, 1.8182, max_tte], 'YLabelString', 'minTTE')
ax.Visible = 'off';