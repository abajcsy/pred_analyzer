clear all
close all

%% Load up all the info for robot.
robot_params = exp2_contingency_planner();

%% Load up all the info for the human predictors.
% Setup what kind of predictors we have
opt_human_params = exp2_opt_pred();
frs_human_params = exp2_frs_pred();
conf_human_params = exp2_conf_pred();

%% Video creation!
save_video = false;
if save_video
    curr_date = datestr(now,'mm_dd_yyyy_HH_MM');
    filename = strcat('exp2_contingency_',curr_date,'.mp4');
    vout = VideoWriter(filename,'MPEG-4');
    vout.Quality = 100;
    vout.FrameRate = 5;
    vout.open;
end

%% PLOT!
figure(1)
hold on
% Plot environment.
bandw_cmap = [0,0,0;1,1,1]; %[1,1,1;0,0,0];
colormap(bandw_cmap)
%ph = pcolor(params.g2d.xs{1}, params.g2d.xs{2}, params.sd_obs);
ph = pcolor(robot_params.gimg.xs{1}, robot_params.gimg.xs{2}, robot_params.obs_map_full);
set(ph, 'EdgeColor', 'none');
% plot robot goal.
qrg = quiver(robot_params.goal(1), robot_params.goal(2), ...
    cos(robot_params.goal(3)), sin(robot_params.goal(3)), 'k');
qrg.Marker = 'o';
qrg.MarkerFaceColor = 'k';
% plot human goal.
scatter(opt_human_params.goal(1), opt_human_params.goal(2), 'r');
xlim([opt_human_params.gmin(1), opt_human_params.gmax(1)])
ylim([opt_human_params.gmin(2), opt_human_params.gmax(2)])
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,600,600])

%% Setup robot start state. 
r_start = [2, 2, -pi/4, 0.01]; %[1, 4, -pi/4, 0.01];
% plot robot state
rsh = scatter(r_start(1), r_start(2), 'k', 'filled');

%% Setup human start state. 
h_start = [-1, 0];
% plot human state
hsh = scatter(h_start(1), h_start(2), 'r', 'filled');

%% Branch times!
%{-1, 0, 0.5}           --> TTE: 1.379310 s
%{-0.3103, 0, 0.6009}   --> TTE: 1.379310 s
%{0.3793, 0, 0.6954}    --> TTE: 0.6897 s
%{1.069, 0, 0.7762}     --> TTE: 0.6897 s
%{1.7586, 0, 0.841}     --> TTE: 0 (this could also be close enough to conf)
%{2.4483, 0, 0.8898}    --> TTE: 0 (already confident enough)
branch_times = [1.379310, 1.379310, 0.6897, 0.6897, 0, 0];

%% TODO: WHAT TO DO IF TTE IS = 0! --> this means we should just use the conservative model?

%% Prior
pbeta = robot_params.belief;

%% Setup simulation horizon. 
simT = 10;
h_xcurr = h_start;
r_xcurr = r_start;

gray_c = [0.8,0.8,0.8];

rps_h = [];
rpopt_h = [];
rpfrs_h = [];
tb1 = [];
tb2 = [];

planned = false;

for t=1:simT
    % Predict human!
    fprintf('Predicting...\n');
    opt_preds = ...
        opt_human_params.predictor.predict(h_xcurr, opt_human_params.T);

    frs_preds = ...
        frs_human_params.predictor.predict(h_xcurr, frs_human_params.T);
    
    
    % Plan for the robot!
    if ~planned 
        branch_t = branch_times(t);
        fprintf('Planning...\n');
        robot_plan = ...
            robot_params.planner.contingency_plan(r_xcurr, robot_params.goal, ...
                                                   opt_preds, frs_preds, ...
                                                   opt_human_params.real_times, ...
                                                   opt_human_params.pred_g, ...
                                                   pbeta, ...
                                                   branch_t); 
        planned = true;                                       
    end
    
    % extract parts of contingency plan.
    shared_plan = robot_plan{1};
    opt_plan = robot_plan{2};
    frs_plan = robot_plan{3};

    % Update human state.
    if t > 2
        h_ctrl = 0;
    else
        h_ctrl = 0; %pi;
    end
    h_xnext = [h_xcurr(1) + opt_human_params.dt * cos(h_ctrl), ...
               h_xcurr(2) + opt_human_params.dt * sin(h_ctrl)];
    
    % Update robot state.
    if t < length(shared_plan{1})
        r_xnext = [shared_plan{1}(t+1), shared_plan{2}(t+1), shared_plan{5}(t+1), shared_plan{3}(t+1)];
    else
        tidx = t - length(shared_plan{1}) + 1;
        r_xnext = [frs_plan{1}(tidx), frs_plan{2}(tidx), frs_plan{5}(tidx), frs_plan{3}(tidx)];
    end
    
    %% Plot robot plan
    hsh.CData = gray_c;
    rsh.CData = gray_c;
%     hsh.Color = gray_c;
%     hsh.MarkerFaceColor = gray_c;
%     rsh.Color = gray_c;
%     rsh.MarkerFaceColor = gray_c;
    if ~isempty(rps_h)
        rps_h.CData = gray_c;
        rps_h.MarkerEdgeColor = gray_c;
        rpopt_h.CData = gray_c;
        rpopt_h.MarkerEdgeColor = gray_c;
        rpfrs_h.CData = gray_c;
        rpfrs_h.MarkerEdgeColor = gray_c;
    end
    % plot human preds
    %g1_th = scatter(human_preds_g1{1}, human_preds_g1{2}, 'markerfacecolor', 'none', 'markeredgecolor', orange_c);
    %g2_th = scatter(human_preds_g2{1}, human_preds_g2{2}, 'markerfacecolor', 'none', 'markeredgecolor', 'b');
    
    % Plot robot shared plan, and contingency plans.
    rps_h = scatter(shared_plan{1}, shared_plan{2}, 'markeredgecolor', 'k');
    rpopt_h = scatter(opt_plan{1}, opt_plan{2}, 'markeredgecolor', 'r');
    rpfrs_h = scatter(frs_plan{1}, frs_plan{2}, 'markeredgecolor', 'b');
    
    hsh = scatter(h_xcurr(1), h_xcurr(2), 'r', 'filled');
    rsh = scatter(r_xcurr(1), r_xcurr(2), 'k', 'filled');
    
    % Plot the belief.
    if ~isempty(tb1)
        delete(tb1)
        delete(tb2)
    end
    tb1 = text(-4.7,-1,strcat('b_{', num2str(t-1),'}(low conf) = ', num2str(pbeta(1))));
    tb1.Color = 'r';
    tb2 = text(-4.7,-1.5,strcat('b_{', num2str(t-1),'}(high conf) = ', num2str(pbeta(2))));
    tb2.Color = 'r';

    if save_video
        current_frame = getframe(gcf); %gca does just the plot
        writeVideo(vout,current_frame);
    end
    pause(0.1);
    
    %% Update belief over betas.
    pbeta = conf_human_params.predictor.belief_update(h_xnext, h_xcurr, pbeta);
    
    %% Finally update states.
    h_xcurr = h_xnext; 
    r_xcurr = r_xnext;
end

if save_video
    vout.close()
end