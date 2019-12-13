# rbe502
RBE 502 Controls Project
Instructions to run the simulation:
1. Clone or Download repository
2. cd into rbe502/rbe502_ws
3. catkin_make
4. source ./devel/setup.bash 
      - can also add this to bash.rc
5. cd ../MATLAB
6. Run matlab in this dir
7. in terminal RUN: roslaunch urdf_sim_tutorial gazebo.launch model:='$(find rbe502_sim)/urdf/robot.urdf.xacro'
8. in matlab run system_ODE.m

If restart of system_ODE.m control is needed
1. run rosshutdown in matlab terminal
2. kill gazebo with ctrl+c in terminal
3. launch again
