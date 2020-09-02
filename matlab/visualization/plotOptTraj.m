function plotOptTraj(traj, traj_tau, goals, trueGoalIdx, ...
    grid_min, grid_max, grid_nums, goalSetRad, extraPltArgs)
    
    % Extract which dimension(s) are the belief dimensions.
    bdims = extraPltArgs.bdims;
    dims = length(grid_nums); % total number of dimensions of this system

    figure
    hold on
    
    % Goal colors.
    g1Color = 'r';
    g2Color = [38., 138., 240.]/255.; %'b';

    % Setup colors.
    startColor = [97., 76., 76.]/255.;
    endColorRed = [255., 0., 0.]/255.;
    endColorBlue = [38., 138., 240.]/255.;
    
    if trueGoalIdx == 1
        endColor = endColorRed;
        finaloffset = -0.3;
        initoffset = 0.1;
    else
        endColor = endColorBlue;
        finaloffset = 0.3;
        initoffset = -0.1;
    end
    
    r = linspace(startColor(1), endColor(1), length(traj_tau));
    g = linspace(startColor(2), endColor(2), length(traj_tau));
    b = linspace(startColor(3), endColor(3), length(traj_tau));

    % Record state.
    % Plot first point.
    color = [r(1), g(1), b(1)];
    xcurr = traj(1:dims, 1);
    
    if dims == 4
        gdisc = (grid_max - grid_min) ./ (grid_nums - 1);
        scale = gdisc(1);
        % plot the current state.
        quiver(xcurr(1), ...
                xcurr(2), ...
                cos(xcurr(3)), ...
                sin(xcurr(3)), scale, ...
                'Color', color, ...
                'linewidth', 2, ...
                'marker', 'o', ...
                'markeredgecolor', color, ...
                'markerfacecolor', color);
    else
        plot3(xcurr(1), xcurr(2), xcurr(bdims), '-o', 'color', color, ...
            'markeredgecolor', color, 'markerfacecolor', color);
    end
    
    % Add first timestamp.
    txt = strcat('t=', num2str(traj_tau(1)), ', b=', num2str(xcurr(bdims)));
    tp = text(xcurr(1)+0.05, xcurr(2)+initoffset, xcurr(bdims)+0.05, txt);
    tp.Color = color;
    for t=2:length(traj_tau)
        xprev = traj(1:dims, t-1);
        xcurr = traj(1:dims, t);
        % Plot point and connection between pts.
        color = [r(t), g(t), b(t)];
        if dims == 4
            % plot the current state.
            quiver(xcurr(1), ...
                    xcurr(2), ...
                    cos(xcurr(3)), ...
                    sin(xcurr(3)), scale, ...
                    'Color', color, ...
                    'linewidth', 2, ...
                    'marker', 'o', ...
                    'markeredgecolor', color, ...
                    'markerfacecolor', color);
        else
             p = plot3([xprev(1), xcurr(1)], [xprev(2), xcurr(2)], [xprev(bdims), xcurr(bdims)], '-o', ...
                'Color', color, ...
                'markeredgecolor', color, ...
                'markerfacecolor', color);
            p.LineWidth = 2;
        end

        if t == length(traj_tau)
            txt = strcat('t=', num2str(traj_tau(t)), ', b=', num2str(xcurr(bdims)));
            tp = text(xcurr(1)+0.05, xcurr(2)+initoffset+0.1, xcurr(bdims)+0.05, txt);
            tp.Color = color;
        end
    
    end

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
    zlim([grid_min(bdims), grid_max(bdims)]);
    
    % Setup labels and position of figure.
    xlabel('x');
    ylabel('y');
    zlabel('P(g = g1)');
    set(gcf,'Position',[100 100 500 500]);
    set(gcf,'color','w');
    
%     set(gca,'xtick',linspace(grid_min(1),grid_max(1),grid_nums(1)));
%     set(gca,'ytick',linspace(grid_min(2),grid_max(2),grid_nums(2)));
%     set(gca,'ztick',linspace(grid_min(3),grid_max(3),grid_nums(3)));
    
    % Saving functionality.
    if extraPltArgs.saveFigs
        repo = what('pred_analyzer');
        filename = strcat('g', num2str(trueGoalIdx), '_uthr' , ...
            num2str(extraPltArgs.uThresh), '_', extraPltArgs.compType, ...
            '_', extraPltArgs.uMode, '.png');
        saveas(gcf, strcat(repo.path, '/imgs/', filename));

        view(17, 14);
        filename = strcat('g', num2str(trueGoalIdx), '_uthr' , ...
            num2str(extraPltArgs.uThresh), '_', extraPltArgs.compType, ...
            '_', extraPltArgs.uMode, '_view2.png');
        saveas(gcf, strcat(repo.path, '/imgs/', filename));
        fprintf("Saved figures!");
    end
    
    % Plot obstacles.
    if isfield(extraPltArgs, 'obstacles')

        for oi = 1:length(extraPltArgs.obstacles)
            obs_info = extraPltArgs.obstacles{oi};
            obs_min = obs_info(1:2);

            x_min = obs_min(1);
            y_min = obs_min(2);
            p_min = 0;
            l = [obs_info(3), ...
                obs_info(4), ...
                1];
            plotcube(l,[x_min y_min p_min], .5, [0.3 0.3 0.3]);
        end
    end
    
    hold off
end