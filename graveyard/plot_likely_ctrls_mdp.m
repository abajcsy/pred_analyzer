clear all
clf
close all

%% File to plot likely controls
% Note that plot is of controls, not of next state (which would be
% gridded).

% Grid setup
gmin = [-4, -4, 0];
gmax = [4, 4, 1];
gnums = [20,20,20];

% Joint Dynamics Setup.
thetas = {[-2, 2], [2, 2]};
trueThetaIdx = 1;
gdisc = (gmax - gmin) ./ (gnums-1);
controls = generate_controls(gdisc);
num_ctrls = numel(controls);
uThresh = 0.13;

% Initial state and dynamical system setup
initial_state = cell(1,3);
initial_state{1} = 0;
initial_state{2} = 0;
initial_state{3} = 0.1;
dyn_sys = MDPHumanBelief2D(thetas, num_ctrls, controls, ...
            initial_state, uThresh, trueThetaIdx);

xs = linspace(gmin(1),gmax(1),gnums(1));
ys = linspace(gmin(2),gmax(2),gnums(2));
pb = 0.1;
true_theta = thetas{trueThetaIdx};

sz = 20;

figure
for i=1:length(xs)
    x = xs(i);
    for j=1:length(ys)
        y = ys(j);
        state = {x,y,pb};
        for k=1:numel(controls)
            u = controls{k};
            pu_true = dyn_sys.pugivenxtheta(u,state,true_theta);
            if pu_true >= uThresh
                next_state = dyn_sys.dynamics(state,u);
                dx = next_state{1} - x;
                dy = next_state{2} - y;
                quiver(x,y,dx,dy,0.4);
                
                % if the control is "STOP", then plot a point.
                if u(1) == 0 && u(2) == 0
                    scatter(x,y,'k');
                end
                hold on
            end
        end
    end
end
for i=1:length(thetas)
    theta = thetas{i};
    plot(theta(1),theta(2),'r.','MarkerSize',sz)
    hold on
end

title_str = strcat("Likely Controls under theta=(", ...
    num2str(true_theta(1)), ", ", num2str(true_theta(2)), ...
    ") and uThresh=", num2str(uThresh));
title(title_str);
xlabel("x")
ylabel("y")

function controls = generate_controls(gdisc)
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


function controls = generate_controls_ext(gdisc)
    controls = cell(1,9);
    xs = {0, -1*gdisc(1), 1*gdisc(1), -2*gdisc(1), 2*gdisc(1)};
    ys = {0, -1*gdisc(2), 1*gdisc(2), -2*gdisc(2), 2*gdisc(2)};
    
    ind = 1;
    for i=1:numel(xs)
        for j=1:numel(ys)
            controls{ind} = [xs{i}, ys{j}];
            ind = ind + 1;
        end
    end
end