%% This is a test file for the HumanBelief1D Class

%% Grid setup
gmin = [-2, -1, -3];
gmax = [3, 1, 5];
gnums = [6, 3, 9];
grid = Grid(gmin, gmax, gnums);

%% Human belief setup
dt = 0.8;
thetas = [-2, 2];
num_ctrls = 2;
controls = [-1, 1];

z0 = cell(1,2);
z0{1} = -2.3158;
z0{2} = 1;
human0 = HumanBelief1D(dt, thetas, num_ctrls, controls, z0);

%% Test dynamics
z1_1 = human0.dynamics(z0, -1)
z1_2 = human0.dynamics(z0, 1)
