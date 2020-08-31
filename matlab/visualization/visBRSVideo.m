function visBRSVideo(g, value_funs, initial_state, tau, stopTau, dt)
    videoFilename = strcat('brs_video.mp4'); 
    vout = VideoWriter(videoFilename,'MPEG-4'); 
    %VideoWriter(videoFilename,'Motion JPEG AVI');
    vout.Quality = 100;
    vout.FrameRate = 5;
    vout.open;
    
    figure;
    viewAxis = [g.min(1) g.max(1) ...
               g.min(2) g.max(2) ...
               -0.1 1.1];
    num_timesteps = tau(end);       
    for i=num_timesteps:-1:stopTau
        hold on
        % Plot value function backwards in time.
        axis(viewAxis)
        axis square
        level = 0;
        extraArgs.sliceDim = 0;
        h = visSetIm(g, value_funs{i}, 'r', level, extraArgs);
        t = title(['$V(z, t=',num2str((i-1)*dt),') \leq 0$'], 'Interpreter', 'Latex');
        t.FontSize = 18;
        view(-43, 13);
        
        % Visualize computation grid through the 'grid' feature in
        % plotting.
        set(gca,'xtick', [-4,-3,-2,-1,0,1,2,3,4]); %linspace(g.min(1),g.max(1),g.N(1)));
        set(gca,'ytick', [-4,-3,-2,-1,0,1,2,3,4]); %linspace(g.min(2),g.max(2),g.N(2)));
        set(gca,'ztick', [0,0.5,1]); %linspace(g.min(3),g.max(3),g.N(3)));
        set(gcf, 'color', 'w')
        grid off;
        box on;
        
        % Plot initial state.
        s = scatter3(initial_state{1}, initial_state{2}, initial_state{3});
        s.SizeData = 40;
        s.MarkerFaceColor = 'k';
        s.MarkerEdgeColor = 'k';
        pause(0.5);
        
        % write video!
        current_frame = getframe(gcf); %gca does just the plot
        writeVideo(vout,current_frame);
        
        if i ~= 1
            delete(h);
        end
    end
    hold off
    vout.close;