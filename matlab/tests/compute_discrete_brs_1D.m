clear
%% File to compute reachable sets and optimal control via discrete HJ

% Grid setup
gmin = [-4, 0];
gmax = [4, 1];
gnums = [80, 80];

% Hamilton-Jacobi Problem Setup
uMode = "max"; % min or max
num_timesteps = 10;
compType = 'conf';
uThresh = 0.;
tol = 0.1;

% Joint Dynamics Setup
dt = 0.1;
thetas = [-3, 3];
trueThetaIdx = 1; %2; 
centerPgoal1 = 0.9; %0.1;
num_ctrls = 2;
controls = [-1, 1];

% Plotting 
plot = true;
goalSetRad = 0.;
extraPltArgs.compType = compType;
extraPltArgs.uThresh = uThresh;
extraPltArgs.uMode = uMode;
extraPltArgs.saveFigs = false;

z0 = [0, 0.5];
dyn_sys = HumanBelief2D(dt, thetas, num_ctrls, controls, z0);

% Target Set Setup
% x_target_ind = [2,gnums(1)-1];
% b_target_ind = [17,gnums(2)];
% initial_value_fun = construct_value_fun(x_target_ind, b_target_ind, gmin, gmax, gnums);

% Target Set Setup
xoffset = 0.1;
center = [0; centerPgoal1];
widths = [(gmax(1) - gmin(1)) - xoffset; 
          tol];
initial_value_fun = construct_value_fun(center, widths, gmin, gmax, gnums);

targets = initial_value_fun;

tic
% Store initial value function
value_funs = cell(1, num_timesteps);
value_funs{end} = initial_value_fun;

% Compute BRS backwards in time
tidx = num_timesteps - 1;

initial_state = cell(1,2);
initial_state{1} = z0(1);
initial_state{2} = z0(2);

while tidx > 0
    % Create grid and set data to value function at t+1
    compute_grid = Grid(gmin, gmax, gnums);
    compute_grid.SetData(value_funs{tidx + 1});
    
    % Compute possible next state after one timestep and find possible
    % value functions
    current_state = compute_grid.get_grid();
    possible_value_funs = zeros([gnums, numel(controls)]);
    for i=1:numel(controls)
        u_i = controls(i);
        next_state = dyn_sys.dynamics(current_state, u_i);
        possible_value_funs(:,:,i) = compute_grid.GetDataAtReal(next_state);
    end
    
    % Minimize/Maximize over possible future value functions.
    if strcmp(uMode, "min")
%         value_fun = min(possible_value_funs, [], 3);
        
        value_fun = min(min(possible_value_funs, [], 3), targets);
    else
%         value_fun = max(possible_value_funs, [], 3);
        
        value_fun = min(max(possible_value_funs, [], 3), targets);
    end
    
    value_funs{tidx} = value_fun;
    
    tidx = tidx - 1;
end

toc

% Plot value functions
if plot
    videoFilename = strcat('brs_2D_',uMode,'.mp4'); 
    vout = VideoWriter(videoFilename,'MPEG-4'); 
    %VideoWriter(videoFilename,'Motion JPEG AVI');
    vout.Quality = 100;
    vout.FrameRate = 5;
    vout.open;

    figure(1);
    g = compute_grid.get_grid();
    for i=num_timesteps:-1:1
        % Plot signed distance function, l(x). 
        %h = subplot(1,num_timesteps,i);
        %h = visSetIm(g, data, color, level, extraArgs);
%         v = (value_funs{i} > 0);
%         s = surf(g{1}, g{2}, double(v));

        % Plot V(z, t);
        s = surf(g{1}, g{2}, value_funs{i});
        shading interp
        zlabel('$l(z)$', 'Interpreter', 'Latex');
        xt = xlabel('$x$', 'Interpreter', 'Latex');
        yt = ylabel('$b(\theta = -3)$', 'Interpreter', 'Latex');
%         t = title('l(z)', 'Interpreter', 'Latex');
        t = title(strcat('V(z,t=', num2str((i-1)*dt), ')'), 'Interpreter', 'Latex');
        xt.FontSize = 15;
        yt.FontSize = 15;
        t.FontSize = 15;
        colorbar
        view(0, 90);
        
        % Plot zero level set.
        hold on
        c = contour(g{1}, g{2}, value_funs{i}, [0,0], '-r', 'LineWidth', 3);
        
        % Plot goals.
        g1 = scatter3(thetas(1), 0, max(value_funs{i}(:)));
        g1.SizeData = 100;
        g1.MarkerFaceColor = 'r';
        g1.MarkerEdgeColor = 'r';
        g2 = scatter3(thetas(2), 0, max(value_funs{i}(:)));
        g2.SizeData = 100;
        g2.MarkerFaceColor = 'b';
        g2.MarkerEdgeColor = 'b';
        
        % Plot initial state.
        is = scatter3(initial_state{1}, initial_state{2}, max(value_funs{i}(:)));
        is.SizeData = 40;
        is.MarkerFaceColor = 'k';
        is.MarkerEdgeColor = 'k';
        
        set(gcf,'color','w'); 
%         colormap(flipud(pink))

        a = [255, 125, 147]/255.;
        b = [255, 219, 38]/255.;
        cmap = [linspace(1,1,256)', ...
                linspace(a(2),b(2),256)', ...
                linspace(a(3),b(3),256)'];

        colormap(cmap);
        hold off
        pause(0.5);
        
        % write video!
        current_frame = getframe(gcf); %gca does just the plot
        writeVideo(vout,current_frame);
        
        if i ~= 1
            delete(s);
        end
    end
    vout.close;
end

% [ctrl_seq, reached, time] = ...
%     find_opt_control(initial_state,value_funs, grid, dyn_sys, controls, uMode)

% Find and plot optimal control sequence (if reachable by computed BRS)
[traj, traj_tau, ctrl_seq, reached, time] = ...
    find_opt_control(initial_state,value_funs, compute_grid, ...
        dyn_sys, controls, uMode, dt);
    
figure(2);
plotTraj(traj, traj_tau, thetas, trueThetaIdx, ...
        gmin, gmax, gnums, goalSetRad, extraPltArgs);

%% Helper functions
% TODO: Make x_ind, b_ind depend on real values not indices
function value_fun = construct_value_fun_fmm(x_ind, b_ind, gmin, gmax, gnums)
    grid = createGrid(gmin, gmax, gnums);
    target_set = ones(size(grid.xs{1}));
    target_set(x_ind(1):x_ind(2),b_ind(1):b_ind(2)) = -1;
    value_fun = compute_fmm_map(grid, target_set);
end

function value_fun = construct_value_fun(center, widths, gmin, gmax, gnums)
    grid = createGrid(gmin, gmax, gnums);
    value_fun = shapeRectangleByCenter(grid, center, widths);
end

function [traj, traj_tau, ctrl_seq, reached, time] = ...
    find_opt_control(initial_state,value_funs, grid, dyn_sys, controls, uMode, dt)
    
    [~, num_timesteps] = size(value_funs);
    n = numel(initial_state);
    
    % Find latest time BRS includes initial state.
    initial_coord = grid.RealToCoords(initial_state);
    traj(1:2,1) = cell2mat(initial_coord);
    traj_tau(1) = 0.0;
    initial_idx = grid.RealToIdx(initial_state);
    initial_idx = initial_idx{1};
    start_t = 0;
    i = num_timesteps;
    %i = 1
    
    while i > 0
        fprintf('Value of initial state at t=-%f: %f ...\n', i*dt, ...
                value_funs{i}(initial_idx));
        if value_funs{i}(initial_idx) <= 0
            start_t = i;
            break
        end
        i = i - 1;
        %i = i + 1;
    end
    
    if start_t == num_timesteps
        reached = true;
        time = 0;
    elseif start_t == 0
        reached = false;
        time = inf;
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
            
            traj(1:2, j+1) = cell2mat(state);
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
    g2Color = [38., 138., 240.]/255.; 

    % Setup colors.
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
    xcurr = traj(1:2, 1);
    plot(xcurr(1), xcurr(2), '-o', 'color', color, ...
        'markeredgecolor', color, 'markerfacecolor', color);
    
    % Add first timestamp.
    txt = strcat('t=', num2str(traj_tau(1)), ', p=', num2str(xcurr(2)));
    tp = text(xcurr(1)+0.05, xcurr(2)+initoffset, txt);
    tp.Color = color;
    for t=2:length(traj_tau)
        xprev = traj(1:2, t-1);
        xcurr = traj(1:2, t);
        % Plot point and connection between pts.
        color = [r(t), g(t), b(t)];
        p = plot([xprev(1), xcurr(1)], [xprev(2), xcurr(2)], '-o', ...
                'Color', color, ...
                'markeredgecolor', color, ...
                'markerfacecolor', color);
        p.LineWidth = 2;
        
        % plot projection onto xaxis.
        p2 = plot([xprev(1), xcurr(1)], [0, 0], '-o', ...
        'Color', color, ...
        'markeredgecolor', color, ...
        'markerfacecolor', color);
        p2.LineWidth = 2;
    end
    
    xcurr = traj(1:2, end);
    % add final info.
    txt = strcat('t=', num2str(traj_tau(end)), ', p=', num2str(xcurr(2)));
    tp = text(xcurr(1), xcurr(2)+0.1, txt);
    tp.Color = color;

    % Plot GOAL 1.
    if strcmp(extraPltArgs.compType, 'conf_and_goal') || ...
            strcmp(extraPltArgs.compType, 'goal')
        
        rectangle('Position',[goals(1)-goalSetRad ...
                              0-goalSetRad ...
                              goalSetRad*2 ...
                              goalSetRad*2],...
                              'Curvature',1, ...
                              'FaceColor',[1, 0.67, 0.67],...
                              'EdgeColor',[1, 0.67, 0.67],...
                              'LineWidth',1);
    end
    scatter(goals(1), 0, ...
                'markeredgecolor', g1Color, ...
                'markerfacecolor', g1Color);
    g1Txt = 'g1';
    t1 = text(goals(1)+0.3, 0, 0.55, g1Txt);
    t1.FontSize = 12;
    t1.Color = g1Color;

    % Plot GOAL 2
    if strcmp(extraPltArgs.compType, 'conf_and_goal') || ...
            strcmp(extraPltArgs.compType, 'goal')
        rectangle('Position',[goals(2)-goalSetRad ...
                  0-goalSetRad ...
                  goalSetRad*2 ...
                  goalSetRad*2],...
                  'Curvature',1, ...
                  'FaceColor',[0.7098, 0.8980, 1.0000],...
                  'EdgeColor',[0.7098, 0.8980, 1.0000],...
                  'LineWidth',1);
    end
    scatter(goals(2), 0, ...
                'markeredgecolor', g2Color, ...
                'markerfacecolor', g2Color);
            
    g2Txt = 'g2';
    t2 = text(goals(2)+0.3, 0, 0.55, g2Txt);
    t2.FontSize = 12;
    t2.Color = g2Color;
    
    grid on;
    xlim([grid_min(1), grid_max(1)]);
    ylim([grid_min(2), grid_max(2)]);
    
    % Setup labels and position of figure.
    xlabel('x');
    zlabel('P(g = g1)');
    set(gcf,'Position',[100 100 500 500]);
    
    set(gca,'xtick',linspace(grid_min(1),grid_max(1),4));
    set(gca,'ytick',linspace(grid_min(2),grid_max(2),4));
    set(gcf,'color','w'); 
end