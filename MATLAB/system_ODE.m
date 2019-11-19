clc
clear all
close all

global m m_w I I_w h r g q_d dq_d K_p K_d
g = 9.8;
m = 1.51518; m_w = 0.053337; h = 0.057794; r = 0.04445;
I = 0.02054; I_w = 0.00004896;

% PD values
K_p = 100*eye(3);
K_d = 10*eye(3);

% desired state
q_d = [0;0;0];
dq_d = [0;0;0];

% call this function with the state and it will
% give back the derivative of state (which includes wheel speeds)
function dx = ODE(x)
    [M,C,G] = MCG(x);
    dx(1:3,1) = x(4:6);
    tau = PD_control(x,G);
    dx(4:6,1) = M\(tau - C - G);
end

% helper functions
function [M,C,G] = MCG(x)
    global m m_w I I_w h r g
    theta  = x(1);
    %phi_1  = x(2); % wheel angles aren't used by the model
    %phi_2  = x(3);
    dtheta = x(4);
    dphi_1 = x(5);
    dphi_2 = x(6);
    
    M = [           m*h^2 + I,      (h*m*r*cos(theta))/2,       (h*m*r*cos(theta))/2;
         (h*m*r*cos(theta))/2, I_w + (m*r^2)/4 + m_w*r^2,                  (m*r^2)/4;
         (h*m*r*cos(theta))/2,                 (m*r^2)/4, I_w + (m*r^2)/4 + m_w*r^2];
    C = [-(h*m*r*sin(theta)*dtheta*(dphi_1 + dphi_2))/2;
                         -(h*m*r*sin(theta)*dtheta^2)/2;
                         -(h*m*r*sin(theta)*dtheta^2)/2];
    G = [-m*g*h*sin(theta); 0; 0];
end

function tau = PD_control(x, G)
    global K_p K_d q_d dq_d
    tau = K_p*(q_d - x(1:3)) + K_d*(dq_d - x(4:6)) + G;
end