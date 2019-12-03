clc
clear all
close all

global m m_w I I_w h r g q_d dq_d K_p K_d
g = 9.8;
m = 1.51518; m_w = 0.053337; h = 0.057794; r = 0.04445;
I = 0.02054; I_w = 0.00004896;

kp = 2;
kd = 0;
K_p = [ kp 0;
       -kp 0];
K_d = [ kd 0;
       -kd 0];

% desired state
q_d = [0;0];
dq_d = [0;0];

global Veldata IMUdata
Veldata = 0;
IMUdata = 0;
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
old_theta = 2*atan2(qy,qw)*180/pi;
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
    Veldata = receive(VelSub); % 10 is the timeout in s, yes this is blocking...
    IMUdata = receive(IMUSub);
    qy = IMUdata.Orientation.Y;
    qw = IMUdata.Orientation.W;
    theta = 2*atan2(qy,qw)*180/pi;
    
    dphi_1 = (Veldata.Position(2) - old_phi_1)/dt;
    dphi_2 = (Veldata.Position(1) - old_phi_2)/dt;
    dphi = (dphi_1 + dphi_2)/2;
    old_phi_1 = Veldata.Position(2);
    old_phi_2 = Veldata.Position(1);
    
    %Build up state vector
    x = zeros(4,1);
    x(1) = theta;
    x(3) = (theta - old_theta)/dt;
    x(4) = dphi;
    
    % Call the ODE function
    % this applies the state-space model
    dx = ODE(x);
    
    new_x = dx*dt + x;
    
    % use wheel speeds from dx to calculate linear x and angular z
    wheelVel = rosmessage('geometry_msgs/Twist');
    wheelVel.Linear.X = r*new_x(4);
    send(vels,wheelVel);
end

% call this function with the state and it will
% give back the derivative of state (which includes wheel speeds)
function dx = ODE(x)
    %global torque
    [M,C,G] = MCG(x);
    dx(1:2,1) = x(3:4);
    tau = PD_control(x,G);
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
    global K_p K_d q_d dq_d
    tau = K_p*(q_d - x(1:2)) + K_d*(dq_d - x(3:4)) + G;
end

function updateVel(~, Veldata_in, ~)
    global Veldata
    Veldata = Veldata_in;
end

function updateIMU(~, IMUdata_in, ~)
    global IMUdata
    IMUdata = IMUdata_in;
end