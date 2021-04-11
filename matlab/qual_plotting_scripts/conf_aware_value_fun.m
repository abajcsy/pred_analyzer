clear all
clf
%close all

%% load data.
load('conf_aware_value_fun.mat');

%% Video recording?
write_video = true;

if write_video
    v = VideoWriter(strcat('conf_aware_value_fun_video.mp4'),'MPEG-4');
    v.Quality = 100;
    open(v)
    % consider all value functions
    end_idx = 1;
else
    % consider just the first value function.
    end_idx = numel(value_funs); 
end

%% Plotting 
% Environment
bandw_cmap = [0,0,0;1,1,1]; 
colormap(bandw_cmap)
%ph = pcolor(params.gimg.xs{1}, params.gimg.xs{2}, params.obs_map_full);
ph = pcolor(params.reward_info.obstacles.g{1}, params.reward_info.obstacles.g{2}, params.reward_info.obstacles.data);
set(ph, 'EdgeColor', 'none');
xlim([params.gmin(1),params.gmax(1)])
ylim([params.gmin(2),params.gmax(2)])
hold on;
% plot the goal
rad = 0.2;
pos = [params.theta(1)-rad, params.theta(2)-rad, rad*2, rad*2];
rg = rectangle('Position', pos, 'Curvature', [1,1]); %80, 'r', 'linewidth', 3);
rg.EdgeColor = 'r';
rg.LineWidth = 3;

% Title and stuff
% title('Minimum TTL as a function of initial state', 'FontSize', 20);
xlim([params.gmin(1),params.gmax(1)])
ylim([params.gmin(2),params.gmax(2)])
zlim([params.gmin(3),params.gmax(3)])

% set(gca,'xtick',[])
% set(gca,'ytick',[])
ylabel('$h_y$', 'Interpreter', 'latex', 'fontsize', 16)
xlabel('$h_x$', 'Interpreter', 'latex', 'fontsize', 16)
zlabel('$b(\beta=0)$', 'Interpreter', 'latex', 'fontsize', 16)

box on
set(gcf, 'color', 'k')
set(gca, 'color', 'k')

ax = gca; % Get handle to current axes.
ax.XColor = 'w';
ax.YColor = 'w'; 
ax.ZColor = 'w'; 
ax.GridAlpha = 1;  % Make grid lines less transparent.
ax.GridColor = 'w'; 

set(gcf, 'position', [0,0,700,700])

%% Plot the value function

hold on
red_orange = [241,65,36]/255.;
z0_loc = scatter3(-4,-2,0.5);
z0_loc.MarkerFaceColor = red_orange;
z0_loc.MarkerEdgeColor = red_orange;
z0_loc.SizeData = 50;

color = 'c';
level = 0.0;
extraArgs.alpha = 0.5;
tstep = -1;
for i=numel(value_funs):tstep:end_idx
    h = visSetIm(params.g, value_funs{i}, color, level, extraArgs);
    t = title(strcat('$$V^{t=',num2str((i-1)*params.dt),'}(z) \leq 0$$'), 'FontSize', 20, 'interpreter','latex');
    t.Color = 'w';
    
    if write_video
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
    if i ~= numel(value_funs) && i ~= end_idx
        delete(h)
    end
end

if write_video 
    close(v);
else
    Image = getframe(gcf);
    imwrite(Image.cdata, 'conf_aware_lx.png');
end