rosinit;
IMUSub = rossubscriber('/imu');
VelSub = rossubscriber('/joint_states');

data = [];
time = [];
tic
t = toc;
while(t < 15)
    IMUdata = receive(IMUSub);
    %qx = IMUdata.Orientation.X;
    qy = IMUdata.Orientation.Y;
    %qz = IMUdata.Orientation.Z;
    qw = IMUdata.Orientation.W;
    theta = 2*atan2(qy,qw)*180/pi;
    
    veldata = receive(VelSub);
    data = [data veldata];
    
    t = toc;
end
rosshutdown;

% x = []; y = []; z = [];
% theta = []; theta2 = [];
% qx = []; qy = []; qz = []; qw = [];
% for i = 1:length(data)
%     x(i) = data(i).LinearAcceleration.X;
%     y(i) = data(i).LinearAcceleration.Y;
%     z(i) = data(i).LinearAcceleration.Z;
%     theta(i) = atan2(x(i),norm([x(i), z(i)]));
%     qx(i) = data(i).Orientation.X;
%     qy(i) = data(i).Orientation.Y;
%     qz(i) = data(i).Orientation.Z;
%     qw(i) = data(i).Orientation.W;
%     theta2(i) = 2*atan2(qy(i),qw(i))*180/pi;
% end
% subplot(4,1,1)
% plot(time,x,'r-')
% subplot(4,1,2)
% plot(time,y,'g-')
% subplot(4,1,3)
% plot(time,z,'b-')
% subplot(4,1,4)
% plot(time,theta,'r-')
% 
% figure;
% subplot(5,1,1)
% plot(time,qx,'r-')
% subplot(5,1,2)
% plot(time,qy,'g-')
% subplot(5,1,3)
% plot(time,qz,'b-')
% subplot(5,1,4)
% plot(time,qw,'r-')
% subplot(5,1,5)
% plot(time,theta2)

% state: [theta dtheta]
% A = [1 dt;
%      0  1];
% Q = [0.05; 0.05]; % process noise
% 
% x_pred = A*x_old; % State prediction, no input
% P_pred = A*P_old*A.' + Q; % Covariance prediction
% y_tilde = 