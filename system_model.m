%% Dynamical Model of System

clc
clear all
close all

% q(t) = [theta; phi];
syms m m_w I I_w l r g phi(t) theta(t)

x_m = r*phi - l*sin(theta); % x - l*sin(theta)
y_m = r + l*cos(theta);
dx_m = diff(x_m, t);
dy_m = diff(y_m, t);
v_m_sqr = dx_m^2 + dy_m^2;

K_m = 0.5*m*v_m_sqr + 0.5*I*(diff(theta,t)^2);
K_w = 0.5*m_w*(r^2)*(diff(phi,t)^2) + 0.5*I_w*(diff(phi,t)^2);

P = m*g*l*cos(theta);

L = K_m + K_w - P;
% tau is the generalized joint torques
% its two components will be equal and oposite
tau(1,1) = diff(diffFunction(L, diff(theta, t)), t) - diffFunction(L, theta);
tau(2,1) = diff(diffFunction(L, diff(phi,   t)), t) - diffFunction(L, phi);

M11 = simplify(tau(1) - subs(tau(1),diff(theta(t), t, t),0)) / diff(theta(t), t, t);
M12 = simplify(tau(1) - subs(tau(1),diff(phi(t), t, t), 0)) / diff(phi(t), t, t);
M21 = simplify(tau(2) - subs(tau(2),diff(theta(t), t, t),0)) / diff(theta(t), t, t);
M22 = simplify(tau(2) - subs(tau(2),diff(phi(t), t, t), 0)) / diff(phi(t), t, t);
M = [M11 M12; M21 M22];
G = simplify(subs(tau, [diff(theta,t) diff(theta,t,t) diff(phi,t) diff(phi,t,t)], [0 0 0 0]));
C = simplify(tau - M*[diff(theta,t,t); diff(phi,t,t)] - G);


% helper functions
function new_f=diffFunction(f,x)
    syms temp
    new_f = subs(f,x,temp);
    new_f = diff(new_f,temp);
    new_f = subs(new_f,temp,x);
end