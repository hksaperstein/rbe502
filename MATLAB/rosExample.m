% 
% %Connect to roscore
% rosinit;
% 
% %% Velocity Publisher
% % Wheel Vels publisher setup
% vels = rospublisher('/cmd_vel','geometry_msgs/Twist');
% %Message setup
% wheelVel = rosmessage('geometry_msgs/Twist');
% 
% %Message population
% WheelVel.Linear.X = 0.1; %this is where you set wheel velocity
% 
% %Send the message
% send(vels,WheelVel);
% 
% %%IMU SUbscriber
% % setup IMU subscriber
% IMUSub = rossubscriber('/imu');
% 
% %data recieved by the subscriber
% IMUdata = receive(IMUSub,10); % 10 is the timeout in s
% 
% 
% %%Vels SUbscriber
% % setup Vel subscriber
% VelSub = rossubscriber('/cmd_vel');
% 
% %data recieved by the subscriber
% Veldata = receive(VelSub,10); % 10 is the timeout in s
% 
% 
% 
% 
% 
% 
% %Disconnect from roscore
% rosshutdown;



rosinit;
vels = rospublisher('/cmd_vel','geometry_msgs/Twist');
wheelVel = rosmessage('geometry_msgs/Twist');
IMUSub = rossubscriber('/imu');
VelSub = rossubscriber('/body_vel'); %this is currently broken till harry adds the publisher

Veldata = receive(VelSub); % 10 is the timeout in s, yes this is blocking...

IMUdata = receive(IMUSub);

%  process things here
WheelVel.Linear.X = 0.1; % replace with the velocity for the wheels from maths
send(vels,WheelVel);


rosshutdown