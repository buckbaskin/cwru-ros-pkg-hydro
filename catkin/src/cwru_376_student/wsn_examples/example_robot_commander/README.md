# example_robot_commander

Here are example nodes that publish values on the "cmd_vel" topic.
The nodes are specialized to send to topic robot0/cmd_vel, which works with the STDR simulator.
Change this topic to jinx/cmd_vel to drive the robot "Jinx" in the lab.


The version "vel_scheduler" is reactive.  It ramps velocity up and down and will recover from halts.  To do so, it uses odometry info, published by STDR on topic /robot0/odom.
To run, e.g., on Jinx, change this topic to listen to Jinx's odom messages.

The version "baskin_vel_scheduler" uses proportional velocity control instead of a direct calculation of a trapezoidal profile. If it is far away, it will request maximum velocity. As it get closer, it requests speed as (currently) .25*distance. Once it gets below a minimum distance, it requests a minimum velocity. Actual velocity is either the requested velocity or the current velocity + acceleration if that is not within the right bounds. This enforces acceleration limits.

Key parameters:
v_max 		 | m/s
v_min 		 | m/s
a_max 		 | m/s/s
omega_max 	 | rad/s
alpha_max	 | rad/s
linear_gain  | (1/s)
angular_gain | (1/s)
DT (run hz)	 | 1/s

## Example usage
To run the STDR simulator:  
'roslaunch cwru_376_launchers stdr_glennan_2.launch'
Then run a velocity commander, e.g.:
'rosrun example_robot_commander vel_scheduler'
Can also observe the speed commands by plotting using:
rqt_plot /robot0/cmd_vel/linear/x


    