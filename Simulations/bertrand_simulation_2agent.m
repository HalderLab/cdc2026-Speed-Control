
% Setup Parameters
T = 12;
tspan = [0 T]; 

% Initial Conditions
% Good set I found: q0 = [0; 0; pi/4; 1; 1; pi/2]; 
% p.v1 = .5;
% p.u1 = .5;       
% p.rho0 = .5;    
% p.k1 = 1;       
% p.k_rho = 2.0;    
% p.k2 = 2.0;    

% State vector: q = [x1; y1; theta1; x2; y2; theta2]
q0 = [0; 0; -pi/4; -1; 0; 0]; 

% Constants
p.v1 = 2.5;
p.u1 = @(t)-.5 + sin(pi*t);       
p.rho0 = 1;    

% Gains
p.k1 = 1;       
p.k_rho = 2;    
p.k2 = 2;       

% Follower left (p.left = 1) or right (p.left = -1)
p.left = -1; 

% Control w/ or w/o knowledge of u1 (1 = with, 0 = without)
p.know = 1;

% Run ODE Solver
options = odeset('RelTol', 1e-5, 'AbsTol', 1e-6);
[t, q] = ode45(@(t,q) dynamics(t, q, p), tspan, q0, options);

% Animate
r1 = q(:, 1:2);
r2 = q(:, 4:5);
Animate2p(t, p, q, 'test');


% Dynamics
function dqdt = dynamics(t, q, p)
    % Unpack Global States
    x1 = q(1); y1 = q(2); th1 = q(3);
    x2 = q(4); y2 = q(5); th2 = q(6);
    
    % Calculate Shape Variables
    dx = x2 - x1;
    dy = y2 - y1;
    rho = sqrt(dx^2 + dy^2);
    phi12 = atan2(dy, dx);
    phi21 = atan2(-dy, -dx);

    alpha1 = wrapToPi(phi12 - th1);
    alpha2 = wrapToPi(phi21 - th2);

    % Calculate side and alpha desired

    alpha1_d = pi/2 * p.left;
    alpha2_d = wrapToPi(alpha1_d + pi * p.left); 

    f_rho = p.k_rho * (rho^2-p.rho0^2)/rho^2;

    % Calculate Controls 
    % Lyapunov function V = 1 - cos(alpha1_err) + 1 - cos(alpha2_err)
    %                       + h(rho)

    v2 = p.left * (p.v1 * sin(alpha1) - rho * p.u1(t) * p.know + rho * p.k1 * sin(alpha1-alpha1_d)) + rho * p.v1 * f_rho;
    u2 = (p.v1 * sin(alpha1) + v2 * sin(alpha2)) / rho + p.k2 * sin(alpha2 - alpha2_d);
    
    % Dynamics
    dqdt = zeros(6,1);

    % Agent 1
    dqdt(1) = p.v1 * cos(th1);
    dqdt(2) = p.v1 * sin(th1);
    dqdt(3) = p.u1(t);
    
    % Agent 2
    dqdt(4) = v2 * cos(th2);
    dqdt(5) = v2 * sin(th2);
    dqdt(6) = u2;
end

