clc
clear all
close all

global m m_w I I_w h r g q_d dq_d K_p K_d torque
torque = [];
g = 9.8;
m = 1.51518; m_w = 0.053337; h = 0.057794; r = 0.04445;
I = 0.02054; I_w = 0.00004896;
kp = 15;
kd = 5;
K_p = [ kp 0;
       -kp 0];
K_d = [ kd 0;
       -kd 0];
q_d = [0;0];
dq_d = [0;0];
q_0 = [pi/6;0];
dq_0 = [0;0];
X0 = [q_0; dq_0];
tf = 4;
[T,X] = ode45(@(t,x)ODE(t,x), [0 tf], X0);

subplot(2,1,1);
plot(T,X(:,1),T,X(:,2));
title('Tilt angle (\theta) and wheel angle (\phi) vs time');
legend({'\theta','\phi'});
ylabel('\theta, \phi [rad]');

subplot(2,1,2);
plot(torque(1,:), torque(2,:), 'r-');
hold on
plot(torque(1,:), torque(3,:), 'b-');
title('Torques vs time');
legend({'\tau_{\theta}', '\tau_{\phi}'});
ylabel('\tau_{\theta,\phi} [Nm]');
xlabel('time [s]');

function dx = ODE(t,x)
    global torque
    [M,C,G] = MCG(x);
    dx(1:2,1) = x(3:4);
    tau = PD_control(x,G);
    torque = [torque [t; tau]];
    dx(3:4,1) = M\(tau - C - G);
end

% helper functions
function [M,C,G] = MCG(x)
    global m m_w I I_w h r g
    theta  = x(1);
    % phi    = x(2);
    dtheta = x(3);
    dphi   = x(4);
    M = [          m*h^2 + I,       h*m*r*cos(theta);
         h*m*r*cos(theta), 2*I_w + m*r^2 + 2*m_w*r^2];
    C = [-h*m*r*sin(theta)*dphi*dtheta;
            -h*m*r*sin(theta)*dtheta^2];
    G = [-g*h*m*sin(theta);
                         0];
end

function tau = PD_control(x, G)
    global K_p K_d q_d dq_d
    tau = K_p*(q_d - x(1:2)) + K_d*(dq_d - x(3:4)) + G;
end