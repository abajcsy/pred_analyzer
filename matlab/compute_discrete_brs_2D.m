clear
%% File to compute reachable sets and optimal control via discrete HJ

% Grid setup
gmin = [-4,-4,0];
gmax = [4,4,1];
gnums = [20,20,20];

% Hamilton-Jacobi Problem Setup
uMode = "min"; % min or max
num_timesteps = 5;
plot = true;
tol = 0.05;
compType = 'conf';
%compType = 'goal';
%compType = 'conf_and_goal';

% Joint Dynamics Setup
dt = 0.5;
thetas = {[-1, 2], [1, 2]};
trueTheta = 1;
centerPgoal1 = 0.95;
num_ctrls = 20;
controls = linspace(0,2*pi,num_ctrls);
v = 1;
uThresh = 0.05;

% ---- Plotting info --- %
extraPltArgs.compType = compType;
extraPltArgs.uThresh = uThresh;
extraPltArgs.uMode = uMode;
extraPltArgs.saveFigs = false;
% ---- Plotting info --- %

initial_state = cell(1,3);
initial_state{1} = 0;
initial_state{2} = 0;
initial_state{3} = 0.5;
dyn_sys = HumanBelief2D(dt, thetas, num_ctrls, controls, initial_state, v, uThresh);

% Target Set Setup
xyoffset = 0.1;
poffset = 0.01;
center = [0; 0; centerPgoal1];
widths = [(gmax(1) - gmin(1)) - xyoffset; ...
          (gmax(2) - gmin(2)) - xyoffset; 
          tol - poffset];
initial_value_fun = construct_value_fun(center, widths, gmin, gmax, gnums);
goalSetRad = 0.1; % not relevant for this high conf goal set.

tic

% Store initial value function
value_funs = cell(1, num_timesteps);
value_funs{num_timesteps} = initial_value_fun;

% Compute BRS backwards in time
tidx = num_timesteps - 1;

while tidx > 0
    % Create grid and set data to value function at t+1
    grid = Grid(gmin, gmax, gnums);
    grid.SetData(value_funs{tidx + 1});
    
    % Compute possible next state after one timestep and find possible
    % value functions
    current_state = grid.get_grid();
    possible_value_funs = zeros([gnums, numel(controls)]);
    likelyMasks = dyn_sys.getLikelyMasks(current_state);
    for i=1:numel(controls)
        u_i = controls(i);
        next_state = dyn_sys.dynamics(current_state, u_i);
        likelyMask = likelyMasks(num2str(u_i));
        possible_value_funs(:,:,:,i) = grid.GetDataAtReal(next_state) .* likelyMask;
    end
    
    % Minimize/Maximize over possible future value functions.
    if strcmp(uMode, "min")
        value_fun = min(possible_value_funs, [], 4);
    else
        value_fun = max(possible_value_funs, [], 4);
    end
    
    value_funs{tidx} = value_fun;
    
    tidx = tidx - 1;
end

% TODO: How to plot in 3d
% Plot valued functions
if plot
%     g = grid.get_grid();
    figure;
    g = createGrid(gmin, gmax, gnums);
    row_len = 5;
    for i=1:num_timesteps
        % Plot signed distance function, l(x). 
        h = subplot(floor(num_timesteps/row_len) + 1,row_len,i);
        visSetIm(g,value_funs{i});
        title(['t=',num2str(i)])
    %     figure;
%         v = min((value_funs{i} > 0), [], 3);
%         x = min(g{1}, [], 3);
%         y = min(g{2}, [], 3);
%         surf(x, y, double(v));
%         zlabel('$l(z)$', 'Interpreter', 'Latex');
%         xlabel('$x$', 'Interpreter', 'Latex');
%         ylabel('$b(\theta)$', 'Interpreter', 'Latex');
    end
end

toc

[traj, traj_tau, ctrl_seq, reached, time] = find_opt_control(initial_state,value_funs, grid, dyn_sys, controls, uMode);
if reached & plot
    figure;
    plotTraj(traj, traj_tau, thetas, trueTheta, ...
    gmin, gmax, goalSetRad, extraPltArgs);
end

reached
ctrl_seq

%% Helper functions
% TODO: Make x_ind, b_ind depend on real values not indices
function value_fun = construct_value_fun_fmm(x_ind, y_ind, b_ind, gmin, gmax, gnums)
    grid = createGrid(gmin, gmax, gnums);
    target_set = ones(size(grid.xs{1}));
    target_set(x_ind(1):x_ind(2),y_ind(1):y_ind(2),b_ind(1):b_ind(2)) = -1;
    value_fun = compute_fmm_map(grid, target_set);
end

function value_fun = construct_value_fun(center, widths, gmin, gmax, gnums)
    grid = createGrid(gmin, gmax, gnums);
    value_fun = shapeRectangleByCenter(grid, center, widths);
end

function [traj, traj_tau, ctrl_seq, reached, time] = find_opt_control(initial_state,value_funs, grid, dyn_sys, controls, uMode)
    [~, num_timesteps] = size(value_funs);
    n = numel(initial_state);
    
    % Find latest time BRS includes initial state.
    initial_coord = grid.RealToCoords(initial_state);
    traj(1:3,1) = cell2mat(initial_coord);
    traj_tau(1) = 0.0;
    initial_idx = grid.RealToIdx(initial_state);
    initial_idx = initial_idx{1};
    start_t = 0;
    i = num_timesteps;
    while i > 0
        if value_funs{i}(initial_idx) <= 0
            start_t = i;
            break
        end
        i = i - 1;
    end
    
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
            for i=1:numel(controls)
                u_i = controls(i);
                idx = grid.RealToIdx(dyn_sys.dynamics(state,u_i));
                val = value_fun_next(idx{1});
                vals = [vals, val];
            end
            if strcmp(uMode, "min")
                [~, ctrl_ind] = min(vals, [], 2);
            else
                [~, ctrl_ind] = max(vals, [], 2);
            end
            ctrl = controls(ctrl_ind);
            ctrl_seq{j} = ctrl;
            state = grid.RealToCoords(dyn_sys.dynamics(state,ctrl));
            time = time + dyn_sys.dt;
            
            traj(1:3, j+1) = cell2mat(state);
            traj_tau(j+1) = time;

            j = j + 1;
        end
    end
end

function plotTraj(traj, traj_tau, goals, trueGoalIdx, ...
    grid_min, grid_max, goalSetRad, extraPltArgs)
    figure(2);
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
    set(gcf,'Position',[100 100 500 500]);
    
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