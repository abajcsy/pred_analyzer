function visBRSSubplots(g, value_funs, initial_state, tau)
    figure;
    viewAxis = [g.min(1) g.max(1) ...
               g.min(2) g.max(2) ...
               -0.1 1.1];
           
    num_timesteps = tau(end);
    row_len = 2;        % Number of subplots on each row of overall plot
    for i=1:num_timesteps
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