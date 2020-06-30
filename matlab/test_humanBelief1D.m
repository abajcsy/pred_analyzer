%% This is a test file for the HumanBelief1D Class

%% Grid setup
gmin = [-2, -1, -3];
gmax = [3, 1, 5];
gnums = [6, 3, 9];
grid = Grid(gmin, gmax, gnums);

%% Human belief setup
dt = 1.0;
thetas = [-2, 2];
num_ctrls = 2;
controls = [-1, 1];

z0 = [0, 0.5];
human0 = HumanBelief1D(dt, thetas, num_ctrls, controls, z0);

%% Test dynamics
z1 = human0.dynamics(z0, -1);
all([z1(1) == -1])
