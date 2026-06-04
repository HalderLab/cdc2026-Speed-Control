clc; clear all; close all;
% Setup Parameters
T = 60;
tspan = [0 T];
p.N = 2;
dist = 1;

% Initial Conditions
% Good set I found: q0 = [0; 0; pi/4; 1; 1; pi/2]; 
% p.v1 = .5;
% p.u1 = .5;       
% p.rho0 = .5;    
% p.k1 = 1;       
% p.k_rho = 2.0;    
% p.k2 = 2.0;    
% State vector: q = [xi, yi, thetai, vi]
q0 = zeros(3*p.N,1);
j = 0;
for i = 1:3:3*p.N
    q0(i,1)   = 0;
    q0(i+1,1) = j;
    q0(i+2,1) = pi/2;
    j = j - dist;
end
% Constants
w1 = 5;
w2 = 1;
A = -1;
tc = 25;
sigma = 1;
p.v1 = .5;
%p.u1 = @(t) -0.15 * double(t >= 2 & t <= 14);
p.u1 = @(t) A*exp(-((t - tc).^2)/(2*sigma^2));       
p.rho0 = 0.5;    
p.v_d = 10;

% Parent
p.parent = zeros(1,p.N);
for i = 1:p.N
    p.parent(i) = i-1;
end

% Gains
p.k1 = 0.5;       
p.k_rho = 1.5;    
p.k2 = 0.5;       

% Follower left (p.left = 1) or right (p.left = -1)
p.left = 1; 

% Control w/ or w/o knowledge of u1 (1 = with, 0 = without)
p.know = 1;

% Run ODE Solver
options = odeset('RelTol', 1e-5, 'AbsTol', 1e-6);
[t, q] = ode45(@(t,q) dynamics(t, q, p), tspan, q0, options);

Vhist = zeros(length(t), p.N);
Uhist = zeros(length(t), p.N);
Khist = zeros(length(t), p.N);

for k = 1:length(t)
    [vk, uk] = controls(t(k), q(k,:)', p);

    Vhist(k,:) = vk.';
    Uhist(k,:) = uk.';
    Ihist(k,:) = 0.5*(w1*(uk.').^2+w2*(vk.'-0.5).^2);

    for i = 1:p.N
        if abs(vk(i)) > 1e-8
            Khist(k,i) = uk(i) / vk(i);
        else
            Khist(k,i) = NaN;
        end
    end
end

% Animate
%r1 = q(:, 1:2);
%r2 = q(:, 4:5);
%[u2_hist, v2_hist, rho_hist] = Animate2p(t, p, q, 'test');

% Plotting
figure; hold on; axis equal; grid on

for i = 1:p.N
    idx = 3*(i-1)+1;
    plot(q(:,idx), q(:,idx+1), 'LineWidth', 1.5)
end

t_mark = 0:3:T;

for i = 1:p.N
    idx = 3*(i-1)+1;

    for k = 1:length(t_mark)
        [~, ind] = min(abs(t - t_mark(k)));
        plot(q(ind,idx), q(ind,idx+1), 'ko', 'MarkerSize', 6);
    end
end

legend(arrayfun(@(i) sprintf('agent %d',i), 1:p.N, 'UniformOutput', false))

figure(2); clf; hold on; grid on
for i = 1:p.N
    plot(t, Vhist(:,i), 'LineWidth', 1.5)
end
xlabel('Time (s)')
ylabel('Speed v')
title('Agent Speeds')
legend(arrayfun(@(i) sprintf('agent %d', i), 1:p.N, 'UniformOutput', false))

figure(3); clf; hold on; grid on
for i = 1:p.N
    plot(t, Khist(:,i), 'LineWidth', 1.5)
end
xlabel('Time (s)')
ylabel('Curvature \kappa')
title('Agent Curvatures')
legend(arrayfun(@(i) sprintf('agent %d', i), 1:p.N, 'UniformOutput', false))

figure(4); clf; hold on; axis equal; grid on

t_mark = 0:1:T;
nmark = length(t_mark);

% initial frame index
[~, ind0] = min(abs(t - t_mark(1)));

% leader initial orientation
theta0 = q(ind0, 3);

for k = 1:nmark
    [~, ind] = min(abs(t - t_mark(k)));

    X = zeros(p.N,1);
    Y = zeros(p.N,1);

    for i = 1:p.N
        idx = 3*(i-1)+1;
        X(i) = q(ind, idx);
        Y(i) = q(ind, idx+1);
    end

    % translate so leader is at origin
    X = X - X(1);
    Y = Y - Y(1);

    % interpolate smooth curve
    s  = 1:p.N;
    ss = linspace(1, p.N, 300);
    Xs = interp1(s, X, ss, 'pchip');
    Ys = interp1(s, Y, ss, 'pchip');

    % current leader orientation
    theta_leader = q(ind, 3);

    % rotate current frame back to initial leader orientation
    phi = theta0 - theta_leader + pi/2;

    R = [cos(phi) -sin(phi);
         sin(phi)  cos(phi)];

    P  = R * [X.';  Y.'];
    Ps = R * [Xs; Ys];

    X_rot  = P(1,:).';
    Y_rot  = P(2,:).';
    Xs_rot = Ps(1,:).';
    Ys_rot = Ps(2,:).';

    % plot
    %plot(X_rot, Y_rot, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k')
    plot(Xs_rot, Ys_rot, 'LineWidth', 1.5)
end

plot(0, 0, 'ro', 'MarkerFaceColor', 'r')
xlabel('x')
ylabel('y')

figure(5); clf; hold on; grid on
for i = 1:p.N
    plot(t, Ihist(:,i), 'LineWidth', 1.5)
end
xlabel('Time (s)')
ylabel('Information')
title('Agent Information')
legend(arrayfun(@(i) sprintf('agent %d', i), 1:p.N, 'UniformOutput', false))

figure(6); clf; hold on; grid on
subplot(2,1,1); hold on;
for i = 1:p.N
    plot(t, Uhist(:,i), 'LineWidth', 1.5)
end
xlabel('Time (s)')
ylabel('Steering u')
title('Agent Speeds')
legend(arrayfun(@(i) sprintf('agent %d', i), 1:p.N, 'UniformOutput', false))

subplot(2,1,2); hold on;
for i = 1:p.N
    plot(t, Vhist(:,i), 'LineWidth', 1.5)
end
xlabel('Time (s)')
ylabel('Speed v')
title('Agent Speeds')
legend(arrayfun(@(i) sprintf('agent %d', i), 1:p.N, 'UniformOutput', false))
% Dynamics
function dqdt = dynamics(t, q, p)
    % Unpack Global States
    N = p.N;
    x = zeros(N,1);
    y = zeros(N,1);
    th = zeros(N,1);

    %x1 = q(1); y1 = q(2); th1 = q(3);
    %x2 = q(4); y2 = q(5); th2 = q(6);
   
    for i = 1:N
        idx = 3*(i-1)+1;
        x(i) = q(idx);
        y(i) = q(idx+1);
        th(i) = q(idx+2);
    end

    [v, u] = controls(t, q, p);
    dqdt = zeros(3*N,1);

    for i = 1:N
        idx = 3*(i-1)+1;
        dqdt(idx) = v(i) * cos(th(i));
        dqdt(idx+1) = v(i) * sin(th(i));
        dqdt(idx+2) = u(i);
    end
end

function [v, u] = controls(t, q, p)
    N = p.N;
    x = zeros(N,1);
    y = zeros(N,1);
    th = zeros(N,1);

    for i = 1:N
        idx = 3*(i-1)+1;
        x(i) = q(idx);
        y(i) = q(idx+1);
        th(i) = q(idx+2);
    end

    v = zeros(N,1);
    u = zeros(N,1);

    v(1) = p.v1;
    u(1) = p.u1(t);

    for i = 2:N
        j = p.parent(i);

        dx = x(i) - x(j);
        dy = y(i) - y(j);
        rho = sqrt(dx^2 + dy^2);
        rho = max(rho, 1e-8);

        phi12 = atan2(dy, dx);
        phi21 = atan2(-dy, -dx);

        alpha1 = wrapToPi(phi12 - th(j));
        alpha2 = wrapToPi(phi21 - th(i));

        alpha1_d = pi/2 * p.left;
        alpha2_d = wrapToPi(alpha1_d + pi * p.left);

        vj = v(j);
        uj = u(j);

        f_rho = p.k_rho * (rho^2 - p.rho0^2)/rho^2;

        v(i) = p.left * (vj * sin(alpha1) - rho * uj * p.know ...
             + rho * p.k1 * sin(alpha1 - alpha1_d)) ...
             + rho * vj * f_rho;

        u(i) = (vj * sin(alpha1) + v(i) * sin(alpha2)) / rho ...
             + p.k2 * sin(alpha2 - alpha2_d);

        v(i) = max(min(v(i), p.v_d), -p.v_d);
    end
end