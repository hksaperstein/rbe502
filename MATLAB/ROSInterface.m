rightWheelVel = rospublisher(cmd_vel_r,twist);
leftWheelVel = rospublisher(cmd_vel_l,twist);


sub = rossubscriber(topicname, msgtype,callback)