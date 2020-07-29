clear all
close all

% load('brt_max_uthresh013.mat'); 
load('brt_min_uthresh013.mat');

% Setup plotting & traj info.
zero_tol = -0.00001;
extraPltArgs.compType = 'conf';
extraPltArgs.uThresh = uThresh;
extraPltArgs.uMode = uMode;
extraPltArgs.saveFigs = false;

% Create relevant grid structures. 
g = createGrid(gmin, gmax, gnums);
compute_grid = Grid(gmin, gmax, gnums);

% Store candidate initial conditions to test at specific times. 
test_init_conds = containers.Map;
test_init_conds(num2str(num_timesteps-1)) = {{-1.474, 1.895, 0.8947}, ...
                                            {0.2105, 3.579, 0.7895}, ...
                                            {1.474, 0.2105, 0.7895}};
test_init_conds(num2str(num_timesteps-2)) = {{0.2105, 1.053, 0.5789}};
test_init_conds(num2str(num_timesteps-6)) = {{-1.474, 0.6316, 0.3684}};
test_init_conds(num2str(num_timesteps-13)) = {{-2.316, -1.474, 0.3684}, ...
                                            {3.579, -1.895, 0.5263}};
                                            % {-3.158,0.2105, 0.4211}
fidx = 1;
for k = keys(test_init_conds)
    init_conds = test_init_conds(k{1});
    hold on
    figure(fidx);
    for state = init_conds
        % Get optimal trajectory for this state. 
        [traj, traj_tau, ctrl_seq, reached, time] = ...
            find_opt_control(state{1}, value_funs, compute_grid, dyn_sys, ...
                dyn_sys.controls, uMode, str2num(k{1}), zero_tol);

        % Plot optimal trajectory.
        plotTraj(traj, traj_tau, thetas, trueThetaIdx, ...
            gmin, gmax, gnums, 0, extraPltArgs);
    end
    
    % Plot the likely controls. 
    plot_likely_ctrls(dyn_sys, gmin, gmax, gnums, thetas, trueThetaIdx, uThresh);
    
    % Finish up plotting setup. 
    t = strcat("Opt Traj for uMode=",uMode," at t=-", num2str(num_timesteps-str2num(k{1})));
    title(t);
    set(gcf,'color','w'); 
    set(gcf,'Position',[100 100 900 900]);
    hold off
    fidx = fidx + 1;
    
    % Save images!
    if extraPltArgs.saveFigs
        repo = what('pred_analyzer');
        [p, ~] = repo.path;
        filename = strcat('t',num2str(num_timesteps-str2num(k{1})),'_',uMode,'.png');
        saveas(gcf, strcat(p, '/matlab/imgs/', filename));
    end
end
                                 
%% Plots the likely ctrls given a threshold. 
function plot_likely_ctrls(dyn_sys, gmin, gmax, gnums, thetas, trueThetaIdx, uThresh)
xs = linspace(gmin(1),gmax(1),gnums(1));
ys = linspace(gmin(2),gmax(2),gnums(2));
pb = 0.1;
true_theta = thetas{trueThetaIdx};

sz = 20;

for i=1:length(xs)
    x = xs(i);
    for j=1:length(ys)
        y = ys(j);
        state = {x,y,pb};
        for k=1:numel(dyn_sys.controls)
            u = dyn_sys.controls{k};
            pu_true = dyn_sys.pugivenxtheta(u,state,true_theta);
            if pu_true >= uThresh
                next_state = dyn_sys.dynamics(state,u);
                dx = next_state{1} - x;
                dy = next_state{2} - y;
                q = quiver(x,y,dx,dy,0.4);
                q.ShowArrowHead = 'on';
                q.MaxHeadSize = 4;
                q.Color = [0.6,0.6,0.6];
                
                % if the control is "STOP", then plot a point.
                if u(1) == 0 && u(2) == 0
                    s = scatter(x,y);
                    s.CData = [0.6,0.6,0.6];
                end
                hold on
            end
        end
    end
end
end

%% Finds the optimal control & trajectory given an initial state.
function [traj, traj_tau, ctrl_seq, reached, time] = ...
    find_opt_control(initial_state, value_funs, grid, dyn_sys, ...
                     controls, uMode, start_idx, zero_tol)
    [~, num_timesteps] = size(value_funs);
    n = numel(initial_state);
    
    % Find latest time BRS includes initial state.
    initial_coord = grid.RealToCoords(initial_state);
    traj(1:3,1) = cell2mat(initial_coord);
    traj_tau(1) = 0.0;
    initial_idx = grid.RealToIdx(initial_state);
    initial_idx = initial_idx{1};
    start_t = start_idx; %0;
    i = num_timesteps;
    
    likelyMasks = dyn_sys.getLikelyMasks(grid.get_grid());
    
%     while i >= start_idx
%         fprintf('Value of initial state at t=-%f: %f ...\n', i, ...
%                 value_funs{i}(initial_idx));
%         if value_funs{i}(initial_idx) <= zero_tol
%             fprintf('Value of initial state at earliest appearance in BRS: %f\n', ...
%                 value_funs{i}(initial_idx));
%             start_t = i;
%             break
%         end
%         i = i - 1;
%     end
    if start_t == num_timesteps
        reached = true;
        time = 0;
        ctrl_seq = [];
    elseif start_t == 0
        reached = false;
        time = inf;
        ctrl_seq = [];
    else
        reached = true;
        ctrl_seq = cell(1,num_timesteps-start_t);
        state = initial_coord;
        j = 1;
        time = 0;
        
        for t=start_t:num_timesteps-1
%             state
            vals = [];
            value_fun_next = value_funs{t+1};
            idx_curr = grid.RealToIdx(state);
            for i=1:numel(controls)
                u_i = controls{i};
                idx = grid.RealToIdx(dyn_sys.dynamics(state,u_i));
                likelyMask = likelyMasks(num2str(u_i));
                val = value_fun_next(idx{1}) * likelyMask(idx_curr{1});
                vals = [vals, val];
            end
            if strcmp(uMode, "min")
                [~, ctrl_ind] = min(vals, [], 2);
            else
                [~, ctrl_ind] = max(vals, [], 2);
            end
            ctrl = controls{ctrl_ind};
            ctrl_seq{j} = ctrl;
            state = grid.RealToCoords(dyn_sys.dynamics(state,ctrl));
            time = time + 1;
            
            traj(1:3, j+1) = cell2mat(state);
            traj_tau(j+1) = time;

            j = j + 1;
        end
    end
end

%% Plots the trajectory. 
function plotTraj(traj, traj_tau, goals, trueGoalIdx, ...
    grid_min, grid_max, grid_nums, goalSetRad, extraPltArgs)
    hold on
    
    % Goal colors.
    g1Color = 'r';
    g2Color = [38., 138., 240.]/255.; %'b';

    % Setup colors.
    %startColor = [79., 0., 128.]/255.;
    %endColor = [255., 143., 255.]/255.;
    startColor = [97., 76., 76.]/255.;
    endColorRed = [255., 0., 0.]/255.;
    endColorBlue = [38., 138., 240.]/255.;
    
    if trueGoalIdx == 1
        endColor = endColorRed;
        finaloffset = -0.3;
        initoffset = 0.3;
    else
        endColor = endColorBlue;
        finaloffset = 0.3;
        initoffset = -0.3;
    end
    
    r = linspace(startColor(1), endColor(1), length(traj_tau));
    g = linspace(startColor(2), endColor(2), length(traj_tau));
    b = linspace(startColor(3), endColor(3), length(traj_tau));

    % Record state.
    % Plot first point.
    color = [r(1), g(1), b(1)];
    xcurr = traj(1:3, 1);
    plot3(xcurr(1), xcurr(2), xcurr(3), '-o', 'color', color, ...
        'markeredgecolor', color, 'markerfacecolor', color);
    
    % Add first timestamp.
    txt = strcat('t=', num2str(traj_tau(1)), ', p=', num2str(xcurr(3)));
    tp = text(xcurr(1)+0.05, xcurr(2)+initoffset, xcurr(3)+0.05, txt);
    tp.Color = color;
    for t=2:length(traj_tau)
        xprev = traj(1:3, t-1);
        xcurr = traj(1:3, t);
        % Plot point and connection between pts.
        color = [r(t), g(t), b(t)];
        p = plot3([xprev(1), xcurr(1)], [xprev(2), xcurr(2)], [xprev(3), xcurr(3)], '-o', ...
                'Color', color, ...
                'markeredgecolor', color, ...
                'markerfacecolor', color);
        p.LineWidth = 2;
    end
    
    xcurr = traj(1:3, end);
    % add final info.
    txt = strcat('t=', num2str(traj_tau(end)), ', p=', num2str(xcurr(3)));
    tp = text(xcurr(1), xcurr(2)+finaloffset, xcurr(3)+0.1, txt);
    tp.Color = color;

    % Plot GOAL 1.
    if strcmp(extraPltArgs.compType, 'conf_and_goal') || ...
            strcmp(extraPltArgs.compType, 'goal')
        
        rectangle('Position',[goals{1}(1)-goalSetRad ...
                              goals{1}(2)-goalSetRad ...
                              goalSetRad*2 ...
                              goalSetRad*2],...
                              'Curvature',1, ...
                              'FaceColor',[1, 0.67, 0.67],...
                              'EdgeColor',[1, 0.67, 0.67],...
                              'LineWidth',1);
    end
    plot3(goals{1}(1), goals{1}(2), 0.5, '-o', ...
                'Color', g1Color, ...
                'markeredgecolor', g1Color, ...
                'markerfacecolor', g1Color);
    g1Txt = 'g1';
    t1 = text(goals{1}(1)+0.3, goals{1}(2), 0.55, g1Txt);
    t1.FontSize = 12;
    t1.Color = g1Color;

    % Plot GOAL 2
    if strcmp(extraPltArgs.compType, 'conf_and_goal') || ...
            strcmp(extraPltArgs.compType, 'goal')
        rectangle('Position',[goals{2}(1)-goalSetRad ...
                  goals{2}(2)-goalSetRad ...
                  goalSetRad*2 ...
                  goalSetRad*2],...
                  'Curvature',1, ...
                  'FaceColor',[0.7098, 0.8980, 1.0000],...
                  'EdgeColor',[0.7098, 0.8980, 1.0000],...
                  'LineWidth',1);
    end
    plot3(goals{2}(1), goals{2}(2), 0.5, '-o', ...
                'Color', g2Color, ...
                'markeredgecolor', g2Color, ...
                'markerfacecolor', g2Color);
            
    g2Txt = 'g2';
    t2 = text(goals{2}(1)+0.3, goals{2}(2), 0.55, g2Txt);
    t2.FontSize = 12;
    t2.Color = g2Color;
    
    grid on;
    xticks([grid_min(1), -3, -2, -1, 0, 1, 2, 3, grid_max(1)]);
    xlim([grid_min(1), grid_max(1)]);
    ylim([grid_min(2), grid_max(2)]);
    zlim([grid_min(3), grid_max(3)]);
    
    % Setup labels and position of figure.
    xlabel('x');
    ylabel('y');
    zlabel('P(g = g1)');
    set(gcf,'Position',[100 100 800 800]);
    
    set(gca,'xtick',linspace(grid_min(1),grid_max(1),grid_nums(1)));
    set(gca,'ytick',linspace(grid_min(2),grid_max(2),grid_nums(2)));
    set(gca,'ztick',linspace(grid_min(3),grid_max(3),grid_nums(3)));
    
    ax = gca;
    ax.XAxis.TickLabelFormat = '%.1f';
    ax.YAxis.TickLabelFormat = '%.1f';

    % Saving functionality.
    if extraPltArgs.saveFigs
        repo = what('hallucinate');
        filename = strcat('g', num2str(trueGoalIdx), '_uthr' , ...
            num2str(extraPltArgs.uThresh), '_', extraPltArgs.compType, ...
            '_', extraPltArgs.uMode, '.png');
        saveas(gcf, strcat(repo.path, '/boltzmann_legibility_imgs/', filename));

        view(17, 14);
        filename = strcat('g', num2str(trueGoalIdx), '_uthr' , ...
            num2str(extraPltArgs.uThresh), '_', extraPltArgs.compType, ...
            '_', extraPltArgs.uMode, '_view2.png');
        saveas(gcf, strcat(repo.path, '/boltzmann_legibility_imgs/', filename));
        fprintf("Saved figures!");
    end
end

%% Visualizes the BRT in 3D. 
function plot_brt_3D(value_funs, gmin, gmax, gnums, num_timesteps)
    figure(1);
    g = createGrid(gmin, gmax, gnums);
    viewAxis = [gmin(1) gmax(1) ...
               gmin(2) gmax(2) ...
               -0.1 1.1];
    for i=num_timesteps:-1:1
        hold on
        % Plot value function backwards in time.
        axis(viewAxis)
        axis square
        level = 0;
        extraArgs.sliceDim = 0;
        h = visSetIm(g, value_funs{i}, 'r', level, extraArgs);
        t = title(['t=-',num2str((num_timesteps-i)),' s'], 'Interpreter', 'Latex');
        t.FontSize = 18;
        view(-43, 13);

        % Visualize computation grid through the 'grid' feature in
        % plotting.
        num2str(tix,'%.1f')
        set(gca,'xtick',linspace(gmin(1),gmax(1),gnums(1)));
        set(gca,'ytick',linspace(gmin(2),gmax(2),gnums(2)));
        set(gca,'ztick',linspace(gmin(3),gmax(3),gnums(3)));
       
        grid on;

        pause(0.5);
        if i ~= 1
            delete(h);
        end
    end
end