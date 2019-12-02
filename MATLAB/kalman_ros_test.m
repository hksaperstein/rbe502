% state: [theta dtheta]
A = [1 dt;
     0  1];
Q = [0.05; 0.05]; % process noise

x_pred = A*x_old; % State prediction, no input
P_pred = A*P_old*A.' + Q; % Covariance prediction
y_tilde = 