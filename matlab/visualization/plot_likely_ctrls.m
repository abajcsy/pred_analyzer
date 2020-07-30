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
dt = 0.5;
thetas = {[-2, 2], [2, 2]};
trueThetaIdx = 1;
num_ctrls = 20;
controls = linspace(0,2*pi,num_ctrls);
v = 1.0;
uThresh = 0.075;

% Initial state and dynamical system setup
initial_state = cell(1,3);
initial_state{1} = 1;
initial_state{2} = 0;
initial_state{3} = 0.5;
dyn_sys = HumanBelief2D(dt, thetas, num_ctrls, controls, ...
            initial_state, v, uThresh, trueThetaIdx);

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
        for k=1:length(controls)
            u = controls(k);
            pu_true = dyn_sys.pugivenxtheta(u,state,true_theta);
            if pu_true >= uThresh
                next_state = dyn_sys.dynamics(state,u);
                dx = cos(u);
                dy = sin(u);
                quiver(x,y,dx,dy,0.4)
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

xlabel("x")
ylabel("y")

% figure
% for i=
% quiver(0,0,1,0,0.5)
% hold on
% quiver(1,0,1,0,0.5)
% hold on
% quiver(0,1,1,0,0.5)