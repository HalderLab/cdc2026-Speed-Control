%% Simulate
T = 15;

% Initial positions ([x_1, y_1, theta_1, x_2, y_2, theta_2, ..., x_n, y_n,
% theta_n])
q0 = [0, 0, 0,... 
    0, -.5, 0,...
    0, -1, 0,...
    0, -1.5, 0,...
    0, -2, 0]; 

% Leader controls ([v, u])
u1 = [1, .5];      
 
% Constants ([k1, k2, k_rho, rho_0])
k = [1, 1, 2, .5];   

simout = sim("Nparticles.slx");
%% Animate

% Extract simulation results and plot
t = simout.tout;
positions = simout.positions;
shapes = simout.shapes;
controls = simout.controls;
AnimateN(t, positions.Data, shapes.Data, controls.Data, false)