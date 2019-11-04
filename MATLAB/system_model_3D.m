%% 3D Dynamical Model of System

clc
clear all
close all

syms m m_w I I_w h r g phi_1(t)  phi_2(t) theta(t)

v_1 = r*diff(phi_1(t),t);
v_2 = r*diff(phi_2(t),t);
v_0 = (v_1 + v_2)/2;
v_m_sqr = v_0^2 + 2*v_0*h*diff(theta,t)*cos(theta) + (h*diff(theta,t))^2;

K_m = 0.5*m*v_m_sqr + 0.5*I*diff(theta,t)^2;
K_w = 0.5*I_w*(diff(phi_1,t)^2 + diff(phi_2,t)^2) + 0.5*m_w*(v_1^2 + v_2^2);
P = m*g*h*cos(theta);

L = K_m + K_w - P;

tau(1,1) = diff(diffFunction(L, diff(theta(t),t)), t) - diffFunction(L, theta(t));
tau(2,1) = diff(diffFunction(L, diff(phi_1(t),t)), t) - diffFunction(L, phi_1(t));
tau(3,1) = diff(diffFunction(L, diff(phi_2(t),t)), t) - diffFunction(L, phi_2(t));

M11 = simplify(tau(1) - subs(tau(1),diff(theta(t), t, t),0)) / diff(theta(t), t, t);
M12 = simplify(tau(1) - subs(tau(1),diff(phi_1(t), t, t),0)) / diff(phi_1(t), t, t);
M13 = simplify(tau(1) - subs(tau(1),diff(phi_2(t), t, t),0)) / diff(phi_2(t), t, t);
M21 = simplify(tau(2) - subs(tau(2),diff(theta(t), t, t),0)) / diff(theta(t), t, t);
M22 = simplify(tau(2) - subs(tau(2),diff(phi_1(t), t, t),0)) / diff(phi_1(t), t, t);
M23 = simplify(tau(2) - subs(tau(2),diff(phi_2(t), t, t),0)) / diff(phi_2(t), t, t);
M31 = simplify(tau(3) - subs(tau(3),diff(theta(t), t, t),0)) / diff(theta(t), t, t);
M32 = simplify(tau(3) - subs(tau(3),diff(phi_1(t), t, t),0)) / diff(phi_1(t), t, t);
M33 = simplify(tau(3) - subs(tau(3),diff(phi_2(t), t, t),0)) / diff(phi_2(t), t, t);
M = [M11 M12 M13; M21 M22 M23; M31 M32 M33];

G = subs(tau, {diff(theta(t), t, t), diff(phi_1(t), t, t), diff(phi_2(t), t, t), diff(theta(t), t), diff(phi_1(t), t), diff(phi_2(t), t)}, {0,0,0,0,0,0});

C = simplify(tau - M*{diff(theta,t,t); diff(phi_1,t,t); diff(phi_2,t,t)} - G);

% syms x_1 x_2 x_3 x_4 x_5 x_6 tau_1 tau_2 tau_3
% Tau = [tau_1; tau_2; tau_3];
% dx = [x_4; x_5; x_6; inv(M)*(Tau-C-G)];
% dx = simplify(dx);
% dx = subs(dx, {diff(theta,t) diff(phi_1,t) diff(phi_2,t)}, {x_4 x_5 x_6});
% dx = subs(dx, {theta phi_1 phi_2}, {x_1 x_2 x_3});

function new_f=diffFunction(f,x)
    syms temp
    new_f = subs(f,x,temp);
    new_f = diff(new_f,temp);
    new_f = subs(new_f,temp,x);
end