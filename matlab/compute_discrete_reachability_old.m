clear all
clf
close all

%% File to compute reachable sets and optimal control via discrete HJ

% Grid setup
gmin = [-4, -4, 0];
gmax = [4, 4, 1];
gnums = [20, 20, 20];

% Hamilton-Jacobi Problem Setup
uMode = "max"; % min or max
num_timesteps = 20;
tol = 0.1;
compType = 'conf';
zero_tol = -0.00001;%-0.03;
minWith = "zero"; % none or zero

% Joint Dynamics Setup.
thetas = {[-2, 2], [2, 2]};
trueThetaIdx = 1;
centerPgoal1 = 0.9;
controls_type = "mdp"; % 'mdp' or 'non-mdp'
if strcmp(controls_type, 'mdp')
    gdisc = (gmax - gmin) ./ (gnums-1);
    controls = generate_controls_mdp(gdisc);
else
    v = 1.0;
    n = 20;
    controls = generate_controls_non_mdp(n,v);
end

num_ctrls = numel(controls);
uThresh = 0.14; % 0.16;

% ---- Plotting info --- %
extraPltArgs.compType = compType;
extraPltArgs.uThresh = uThresh;
extraPltArgs.uMode = uMode;
extraPltArgs.saveFigs = false;
row_len = 2;        % Number of subplots on each row of overall plot
plot = true;        % Visualize the BRS?
plotVideo = true;   % Plot the BRS growing as a video? If false, plots as subplots.
% ---- Plotting info --- %

% Initial state and dynamical system setup
initial_state = {-2,0,0.1};
dyn_sys = MDPHumanBelief2D(thetas, num_ctrls, controls, ...
            initial_state, uThresh, trueThetaIdx);

% Target Set Setup
xyoffset = 0.1;
center = [0; 0; centerPgoal1];
widths = [(gmax(1) - gmin(1)) - xyoffset; ...
          (gmax(2) - gmin(2)) - xyoffset; 
          tol];
initial_value_fun = construct_value_fun(center, widths, gmin, gmax, gnums);
goalSetRad = 0.1; % not relevant for this high conf goal set.

% (Benchmarking) Timer for the overall computation.
overallStart = tic;

% Store initial value function.
value_funs = cell(1, num_timesteps);
value_funs{num_timesteps} = initial_value_fun;

% Compute BRS backwards in time.
tidx = num_timesteps - 1;
start_idx = 0;
    
while tidx > 0
    fprintf("Computing value function for iteration t=%f...\n", tidx);
    %(Benchmarking) Timer for single computation.
    singleCompStart = tic;
    
    % Create grid and set data to value function at t+1
    compute_grid = Grid(gmin, gmax, gnums);
    compute_grid.SetData(value_funs{tidx + 1});
    
    % Compute possible next state after one timestep and find possible
    % value functions for each control
    current_state = compute_grid.get_grid();
    possible_value_funs = zeros([gnums, numel(controls)]);
    likelyMasks = dyn_sys.getLikelyMasks(current_state);
    for i=1:numel(controls)
        u_i = controls{i};
        next_state = dyn_sys.dynamics(current_state, u_i);
        likelyMask = likelyMasks(num2str(u_i));
        data_next = compute_grid.GetDataAtReal(next_state);
        possible_value_funs(:,:,:,i) = data_next .* likelyMask;
    end
    
    % Minimize/Maximize over possible future value functions.
    if strcmp(uMode, "min")
        if strcmp(minWith, "none")
            value_fun = min(possible_value_funs, [], 4);
        else
            value_fun = min(min(possible_value_funs, [], 4), value_funs{num_timesteps});
        end
    else
        if strcmp(minWith, "none")
            value_fun = max(possible_value_funs, [], 4);
        else
            value_fun = min(max(possible_value_funs, [], 4), value_funs{num_timesteps});
        end
    end
    
    value_funs{tidx} = value_fun;
    start_idx = tidx;
    tidx = tidx - 1;
    
    singleCompEnd = toc(singleCompStart); 
    fprintf("    [One backup computation time: %f s]\n", singleCompEnd);
    
    % Index of initial state.
    initial_idx = compute_grid.RealToIdx(initial_state);
    initial_idx = initial_idx{1};
    if value_fun(initial_idx) <= zero_tol
        fprintf("    Found earliest BRS containing z0!\n");
        break;
    end
end

% Save data
% save('brt_min_uthresh013.mat', 'value_funs', 'gmin', 'gmax', 'gnums', ...
%     'thetas', 'dyn_sys', 'uThresh', 'trueThetaIdx', 'num_timesteps', 'uMode');

% End timing the overall computation;
toc(overallStart);

% Plot value functions
if plot && plotVideo
    figure(1);
    g = createGrid(gmin, gmax, gnums);
    viewAxis = [gmin(1) gmax(1) ...
               gmin(2) gmax(2) ...
               -0.1 1.1];
    for i=num_timesteps:-1:start_idx
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
        set(gca,'xtick',linspace(gmin(1),gmax(1),gnums(1)));
        set(gca,'ytick',linspace(gmin(2),gmax(2),gnums(2)));
        set(gca,'ztick',linspace(gmin(3),gmax(3),gnums(3)));

        grid on;
        
        % Plot initial state.
        s = scatter3(initial_state{1}, initial_state{2}, initial_state{3});
        s.SizeData = 40;
        s.MarkerFaceColor = 'k';
        s.MarkerEdgeColor = 'k';
        pause(0.5);
        if i ~= 1
            delete(h);
        end
    end
elseif plot && ~plotVideo
    figure(1);
    g = createGrid(gmin, gmax, gnums);
    viewAxis = [gmin(1) gmax(1) ...
               gmin(2) gmax(2) ...
               -0.1 1.1];
    for i=start_idx:num_timesteps
        % Plot value function at time i.
        dt=1;
        h = subplot(floor(num_timesteps/row_len) + 1,row_len,i);
        axis(viewAxis)
        axis square
        visSetIm(g,value_funs{i});
        t = title(['t=',num2str((i-1)*dt),' s'], 'Interpreter', 'Latex');
        t.FontSize = 18;
        view(-43, 13);
        grid on;
        
        % Plot initial state.
        hold on
        s = scatter3(initial_state{1}, initial_state{2}, initial_state{3});
        s.SizeData = 40;
        s.MarkerFaceColor = 'k';
        s.MarkerEdgeColor = 'k';
    end
end

% Find and plot optimal control sequence (if reachable by computed BRS)
[traj, traj_tau, ctrl_seq, reached, time] = ...
    find_opt_control(initial_state,value_funs, compute_grid, ...
        dyn_sys, controls, uMode, likelyMasks, start_idx, zero_tol);
    
if reached && plot
    figure(2)
    plotTraj(traj, traj_tau, thetas, trueThetaIdx, ...
        gmin, gmax, gnums, goalSetRad, extraPltArgs);
end

if ~reached
    fprintf("BRS never included initial state! Cannot find optimal control.");
end

ctrl_seq

%% Helper functions

function controls = generate_controls_mdp(gdisc)
    controls = cell(1,9);
    xs = {0, -1*gdisc(1), gdisc(1)};
    ys = {0, -1*gdisc(2), gdisc(2)};
    
    ind = 1;
    for i=1:numel(xs)
        for j=1:numel(ys)
            controls{ind} = [xs{i}, ys{j}];
            ind = ind + 1;
        end
    end
end

function controls = generate_controls_non_mdp(n, v)
    us = linspace(0,2*pi - 1e-2,n);
    controls = cell(1,n);
    
    for i=1:n
        u_i = us(i);
        controls{i} = [v*cos(u_i), v*sin(u_i)];
    end
end

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

function [traj, traj_tau, ctrl_seq, reached, time] = ...
    find_opt_control(initial_state, value_funs, grid, dyn_sys, controls, ...
    uMode, likelyMasks, start_idx, zero_tol)

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
    
    while i >= start_idx
        fprintf('Value of initial state at t=-%f: %f ...\n', i, ...
                value_funs{i}(initial_idx));
        if value_funs{i}(initial_idx) <= zero_tol
            fprintf('Value of initial state at earliest appearance in BRS: %f\n', ...
                value_funs{i}(initial_idx));
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
    set(gcf,'Position',[100 100 500 500]);
    
    set(gca,'xtick',linspace(grid_min(1),grid_max(1),grid_nums(1)));
    set(gca,'ytick',linspace(grid_min(2),grid_max(2),grid_nums(2)));
    set(gca,'ztick',linspace(grid_min(3),grid_max(3),grid_nums(3)));
    
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