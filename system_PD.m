clc
clear all
close all

%syms m m_w I I_w l r g theta(t) phi(t)
global m m_w I I_w l r g q_d dq_d K_p K_d torque
torque = [];
g = 9.8;
m = 1; m_w = 0.1; l = 0.25; r = 0.04;
I = (1/3)*m*(2*l)^2; I_w = 2*(1/2)*m_w*r^2; 
K_p = [10 0; 0 10];
K_d = [10 0; 0 10];
q_d = [0;0];
dq_d = [0;0];
q_0 = [pi/3;0];
dq_0 = [0;0];
X0 = [q_0; dq_0];
[T,X] = ode45(@(t,x)ODE(t,x), [0 10], X0);
plot(T,X);

function dx = ODE(t,x)
    global torque
    [M,C,G] = MCG(x);
    dx(1:2,1) = x(3:4);
    tau = PD_control(x,G);
    torque = [torque tau];
    dx(3:4,1) = inv(M)*(tau - C - G);
end

% helper functions
function [M,C,G] = MCG(x)
    global m m_w I I_w l r g
    theta  = x(1);
    phi    = x(2);
    dtheta = x(3);
    dphi   = x(4);
    M = [            m*l^2 + I,  -l*m*r*cos(theta);
          -l*m*r*cos(theta), I_w + m*r^2 + m_w*r^2];
    C = [l*m*r*sin(theta)*dphi*dtheta;
            l*m*r*sin(theta)*dtheta^2];
    G = [-g*l*m*sin(theta);
                         0];
end

function tau = PD_control(x, G)
    global K_p K_d q_d dq_d
    tau = K_p*(q_d - x(1:2)) + K_d*(dq_d - x(3:4)) + G;
end