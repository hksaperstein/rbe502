clc
clear all
close all

global m m_w I I_w h r g q_d dq_d i_term
g = 9.8;
m = 1.51518; m_w = 0.053337; h = 0.057794; r = 0.04445;
I = 0.02054; I_w = 0.00004896;

% desired state
q_d = [0;0];
dq_d = [0;0];

i_term = 0.0; % accumulator for I term of PID

% ROS Setup
rosinit;
vels = rospublisher('/cmd_vel','geometry_msgs/Twist');
IMUSub = rossubscriber('/imu');
VelSub = rossubscriber('/joint_states');
% while(IMUdata == 0)
%     pause(0.1);
% end
% while(Veldata == 0)
%     pause(0.1);
% end
IMUdata = receive(IMUSub);
qy = IMUdata.Orientation.Y;
qw = IMUdata.Orientation.W;
old_theta = 2*atan2(qy,qw);
tic;
old_t = toc;
old_phi_1 = 0;
old_phi_2 = 0;
i = 0;
while(1)
    t = toc;
    dt = t - old_t;
    old_t = t;
    % Get new state data from ROS
    Veldata = receive(VelSub);
    IMUdata = receive(IMUSub);
    qy = IMUdata.Orientation.Y;
    qw = IMUdata.Orientation.W;
    theta = 2*atan2(qy,qw);
    
    dphi_1 = (Veldata.Position(2) - old_phi_1)/dt;
    dphi_2 = (Veldata.Position(1) - old_phi_2)/dt;
    dphi = (dphi_1 + dphi_2)/2;
    old_phi_1 = Veldata.Position(2);
    old_phi_2 = Veldata.Position(1);
    
    old_theta = theta;
    
    %Build up state vector
    x = zeros(4,1);
    x(1) = theta;
    x(2) = (Veldata.Position(2) + Veldata.Position(1))/2;
    x(3) = (theta - old_theta)/dt;
    x(4) = dphi;
    
    % Call the ODE function
    % this applies the state-space model
    dx = ODE(x,dt);
    
    new_x = dx*dt + x;
    
    % use wheel speeds from dx to calculate linear x and angular z
    wheelVel = rosmessage('geometry_msgs/Twist');
    wheelVel.Linear.X = r*new_x(4);
    send(vels,wheelVel);
end

% call this function with the state and it will
% give back the derivative of state (which includes wheel speeds)
function dx = ODE(x,dt)
    %global torque
    [M,C,G] = MCG(x);
    dx(1:2,1) = x(3:4);
%     tau = PD_control(x,G);
    tau = PID_control(x,dt);
    %torque = [torque [t; tau]];
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
    global q_d dq_d
    kp = 0.05;
    kd = 0.06;
    K_p = [ kp 0;
           -kp 0];
    K_d = [ kd 0;
           -kd 0];
    tau = K_p*(q_d - x(1:2)) + K_d*(dq_d - x(3:4)) + G;
end

function tau = PID_control(x,dt)
    global q_d dq_d i_term
    kp = 0.03005;
    kd = 0.0009;
    ki = 0.02;
%     K_p = [kp 0;
%           -kp 0];
%     K_d = [kd 0;
%            0 kd];
%     K_i = [ki 0;
%           -ki 0];
     e = q_d - x(1:2)
%     de = dq_d - x(3:4);
    i_term = (i_term + e)*dt;
%     tau = K_p*e + K_d*de + K_i*i_term
    tau_0 = kp*(q_d(1)-x(1)) + kd*(dq_d(1)-x(3)) + ki*i_term(1)
    tau = [tau_0; -tau_0];
end

% function updateVel(~, Veldata_in, ~)
%     global Veldata
%     Veldata = Veldata_in;
% end
% 
% function updateIMU(~, IMUdata_in, ~)
%     global IMUdata
%     IMUdata = IMUdata_in;
% end