clear all
close all

%% Load up all the info for robot.
robot_params = exp2_planner_baselines();

%% Load up all the info for the human.
% Setup what kind of collision checking we will do
%coll_check = 'conf'; 
coll_check = 'opt'; 
%coll_check = 'frs'; 

if strcmp(coll_check, 'conf')
    human_params = exp2_conf_pred();
elseif strcmp(coll_check, 'opt')
    human_params = exp2_opt_pred();
elseif strcmp(coll_check, 'frs')
    human_params = exp2_frs_pred();
else
    error('other predictors not implemented here yet!')
end

%% Video creation!
vout = VideoWriter(strcat(coll_check,'_baseline.mp4'),'MPEG-4');
vout.Quality = 100;
vout.FrameRate = 5;
vout.open;

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
scatter(human_params.goal(1), human_params.goal(2), 'r');
% plot human state
hsh = scatter(human_params.x0(1), human_params.x0(2), 'r', 'filled');
xlim([human_params.gmin(1), human_params.gmax(1)])
ylim([human_params.gmin(2), human_params.gmax(2)])
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,600,600])

%% Setup robot start state. 
r_start = [1, 4, -pi/4, 0.01];
rsh = scatter(r_start(1), r_start(2), 'k', 'filled');

%% Setup simulation horizon. 
simT = 10;
h_xcurr = human_params.x0;
r_xcurr = r_start;

if strcmp(coll_check, 'conf')
    pbeta = human_params.beta_prior; 
end

gray_c = [0.8,0.8,0.8];

rph = [];
tb1 = [];
tb2 = [];

for t=1:simT
    % Predict human!
    fprintf('Predicting...\n');
    if strcmp(coll_check, 'conf')
        human_preds = ...
            human_params.predictor.predict(h_xcurr, human_params.T, pbeta);
    elseif strcmp(coll_check, 'opt') || strcmp(coll_check, 'frs')
        human_preds = ...
            human_params.predictor.predict(h_xcurr, human_params.T);
    end
    
    % Plan for the robot!
    fprintf('Planning...\n');
    robot_plan = ...
        robot_params.planner.plan(r_xcurr, robot_params.goal, ...
            human_preds, human_params.real_times, human_params.pred_g, coll_check);
    
    % Update human state.
    if t > 2
        h_ctrl = 0;
    else
        h_ctrl = pi;
    end
    h_xnext = [h_xcurr(1) + human_params.dt * cos(h_ctrl), ...
               h_xcurr(2) + human_params.dt * sin(h_ctrl)];
    
    % Update robot state.
    r_xnext = [robot_plan{1}(2), robot_plan{2}(2), robot_plan{5}(2), robot_plan{3}(2)];
    
    %% Plot robot plan
    hsh.CData = gray_c;
    rsh.CData = gray_c;
    if ~isempty(rph)
        rph.CData = gray_c;
    end
    rph = scatter(robot_plan{1}, robot_plan{2}, 'k');
    hsh = scatter(h_xcurr(1), h_xcurr(2), 'r', 'filled');
    rsh = scatter(r_xcurr(1), r_xcurr(2), 'k', 'filled');
    
    % Plot the belief.
    if strcmp(coll_check, 'conf')
        if ~isempty(tb1)
            delete(tb1)
            delete(tb2)
        end
        tb1 = text(-4.7,-1,strcat('b_', num2str(t-1),'(low conf) = ', num2str(pbeta(1))));
        tb1.Color = 'r';
        tb2 = text(-4.7,-1.5,strcat('b_', num2str(t-1),'(high conf) = ', num2str(pbeta(2))));
        tb2.Color = 'r';
    end
    current_frame = getframe(gcf); %gca does just the plot
    writeVideo(vout,current_frame);
    pause(0.1)
    
    %% Update belief over betas.
    if strcmp(coll_check, 'conf')
        pbeta = human_params.predictor.belief_update(h_xnext, h_xcurr, pbeta);
    end
    
    %% Finally update states.
    h_xcurr = h_xnext; 
    r_xcurr = r_xnext;
end

vout.close()