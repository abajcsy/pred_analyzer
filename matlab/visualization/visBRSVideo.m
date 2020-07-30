function visBRSVideo(g, value_funs, initial_state, tau)
    figure;
    viewAxis = [g.min(1) g.max(1) ...
               g.min(2) g.max(2) ...
               -0.1 1.1];
    num_timesteps = tau(end);       
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
        set(gca,'xtick',linspace(g.min(1),g.max(1),g.N(1)));
        set(gca,'ytick',linspace(g.min(2),g.max(2),g.N(2)));
        set(gca,'ztick',linspace(g.min(3),g.max(3),g.N(3)));

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