/* William Baskin */
// try this, e.g. with roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch
// or: roslaunch cwru_376_launchers stdr_glennan_2.launch
// watch resulting velocity commands with: rqt_plot /robot0/cmd_vel/linear/x (or jinx/cmd_vel...)
//intent of this program: modulate the velocity command to comply with a speed limit, v_max,
// acceleration limits, +/-a_max, and come to a halt gracefully at the end of
// an intended line segment
// notes on quaternions:
/*
From:
http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
qx = ax * sin(angle/2)
qy = ay * sin(angle/2)
qz = az * sin(angle/2)
qw = cos(angle/2)
so, quaternion in 2-D plane (x,y,theta):
ax=0, ay=0, az = 1.0
qx = 0;
qy = 0;
qz = sin(angle/2)
qw = cos(angle/2)
therefore, theta = 2*atan2(qz,qw)
*/

//to reset robot in stdr: rosrun stdr_robot robot_handler replace /robot0 27 21 3.9


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <vector>
/* 
	DYNAMIC LIMITS 
 */
const double v_max = 1.0; //1m/sec is a slow walk
const double v_min = 0.1; // if command velocity too low, robot won't move
const double a_max = 0.1; //1m/sec^2 is 0.1 g's

const double omega_max = 1.0; //1 rad/sec-> about 6 seconds to rotate 1 full rev
const double alpha_max = 0.5; //0.5 rad/sec^2-> takes 2 sec to get from rest to full omega

//proportional control adjustment parameters
double linear_gain = 0.25;
double angular_gain = 0.5;

const double DT = 0.050; // update rate of 20Hz; faster with actual hardware
std::vector< double > path;
int segment_index = 0;

double odom_vel_ = 0.0; //robot speed
double odom_omega_ = 0.0; // robot spin
double odom_x_ = 0.0;
double odom_y_ = 0.0;
double odom_phi_ = 0.0;
double dt_odom_ = 0.0;
ros::Time t_last_callback_;
double dt_callback_=0.0;

void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
	// compute the delta-time between successive callbacks:
	dt_callback_ = (ros::Time::now() - t_last_callback_).toSec();
	t_last_callback_ = ros::Time::now();

	if (dt_callback_ > 0.15) { // avoid eccessively large dt
		dt_callback_ = 0.1; 
		ROS_WARN("large dt; dt = %lf", dt_callback_);
	}
	// move current state to global vars for main
	odom_vel_ = odom_rcvd.twist.twist.linear.x;
	odom_omega_ = odom_rcvd.twist.twist.angular.z;
	odom_x_ = odom_rcvd.pose.pose.position.x;
	odom_y_ = odom_rcvd.pose.pose.position.y;
	//Convert quaternion to heading in 2D
	double quat_z = odom_rcvd.pose.pose.orientation.z;
	double quat_w = odom_rcvd.pose.pose.orientation.w;
	odom_phi_ = 2.0*atan2(quat_z, quat_w);
}


/*
	MAIN
*/

int main(int argc, char **argv) {
	ros::init(argc, argv, "vel_scheduler_wcb38");
	ros::NodeHandle nh;
	ros::Publisher vel_cmd_pub = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1);
	ros::Subscriber odom_sub = nh.subscribe("/robot0/odom", 1, odomCallback);
	ros::Rate rtimer(1 / DT);
	path.resize(6,0.0); 
	/*
		index % 2 == 0 --> straight_dist in meters. index%2 == 1 --> turn_angle in radians.
	 */
	path[0] =  4.666;
	path[1] = -1.54;
	path[2] =  12.25;
	path[3] = -1.571;
	path[4] =  8.0;
	path[5] =  0.0; //done

	/*
		Initialize processing variables
	*/
	//segment information
	double segment_length = 0.0;
	double segment_length_done = 0.0;
	double start_x = 0.0;
	double start_y = 0.0;
	double start_phi = 0.0;

	//desired output (**note W or V)
	double scheduled_vel = 0.0;
	double scheduled_w = 0.0;

	//adjusted output for acceleration limits
	double new_cmd_vel = 0.0;
	double new_cmd_omega = 0.0;

	// message to output
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0.0;
	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;

	while(segment_index < path.size() && ros::ok()) {
		// Take segment from defined path
		double segment_length = path[segment_index];

		//wait for new odom data
		odom_omega_ = 1000000;
		ROS_INFO("waiting for valid odom callback...");
		t_last_callback_ = ros::Time::now();
		while (odom_omega_ > 1000 && ros::ok()) {
			rtimer.sleep();
			ros::spinOnce();
		}
		ROS_INFO("received odom message; proceeding");
		start_x = odom_x_;
		start_y = odom_y_;
		start_phi = odom_phi_;
		ROS_INFO("start pose: x %f, y= %f, phi = %f", start_x, start_y, start_phi);
		/*
			START CONTROL LOGIC
		*/
		if (segment_index % 2 == 0) { //linear drive forward
			double max_vel = v_max;
			double max_accel = a_max;
			ROS_INFO("forward segment #%d", (segment_index+1));
			while (ros::ok())
			{
				ros::spinOnce();
				double delta_x = odom_x_ - start_x;
				double delta_y = odom_y_ - start_y;
				//calculate distance traveled by distance from start
				segment_length_done = sqrt(delta_x * delta_x + delta_y * delta_y);
				double dist_to_go = segment_length - segment_length_done;
				ROS_INFO("dist_to_go: %f", dist_to_go);

				//planning speed to go
				if (dist_to_go<= 0.0) { // if goal state
					scheduled_vel=0.0;
				}
				else { // else request dist_proportional velocity w/in maxes
					scheduled_vel = std::max(-1.0*max_vel , std::min(max_vel, dist_to_go*linear_gain));
					if ( std::abs(scheduled_vel) < v_min) { // if speed is too slow, make it min speed
						if (scheduled_vel > 0.0) { scheduled_vel = v_min ; }
						else { scheduled_vel = -v_min ; }
					}
				}

				// check against accel limits
				if (odom_vel_ < scheduled_vel - max_accel*dt_callback_) {
					double v_test = odom_vel_ + max_accel*dt_callback_;
					new_cmd_vel = v_test;
				}
				else if (odom_vel_ > scheduled_vel + a_max*dt_callback_) {
					double v_test = odom_vel_ - a_max*dt_callback_;
					new_cmd_vel = v_test;
				}
				else {
					new_cmd_vel = scheduled_vel;
				}

				//publish message
				cmd_vel.linear.x = new_cmd_vel;
				cmd_vel.angular.z = 0.0;
				if (dist_to_go <= 0.0) {
					cmd_vel.linear.x = 0.0;
					cmd_vel.angular.z = 0.0;
				}
				vel_cmd_pub.publish(cmd_vel);
				if (dist_to_go <= 0.0) break; // finish if arrived
				rtimer.sleep();

			}
			ROS_INFO("completed move distance for segment %d", (segment_index+1));
			segment_index = segment_index+1;
		}
		else { //turning to heading
			double max_alpha = alpha_max;
			double max_w = omega_max;
			ROS_INFO("turn segment #%d", (segment_index+1));
			while (ros::ok())
			{
				ros::spinOnce();
				// calculate distance twisted by distance from start
				segment_length_done = odom_phi_- start_phi;
				double dist_to_go = segment_length - segment_length_done;
				ROS_INFO("dist to go: %f", dist_to_go);
				
				//planning speed to go
				if (dist_to_go<= 0.0 && segment_length >= 0.0) { // if goal state 1
					scheduled_w=0.0;
				}
				else if (dist_to_go > 0.0 && segment_length < 0.0 ) { // if goal state 2
					scheduled_w=0.0;
				}
				else { // else request twist_proportional angular vel w/in maxes
					scheduled_w = std::max(-1.0*max_w , std::min(max_w, dist_to_go*angular_gain));
					//if (segment_length < 0.0 ) { scheduled_w = scheduled_w * -1.0; }
					if ( std::abs(scheduled_w) < .05) { //if speed is too slow, make it min speed
						if (scheduled_w > 0.0) { scheduled_w = 0.05 ; }
						else { scheduled_w = -0.05 ; }
					}
				}

				//check against accel limits
				if (odom_omega_ < scheduled_w - max_alpha*dt_callback_) {
					double w_test = odom_omega_ + max_alpha*dt_callback_;
					new_cmd_omega = w_test;
				}
				else if (odom_vel_ > scheduled_w + max_alpha*dt_callback_) {
					double w_test = odom_omega_ - max_alpha*dt_callback_;
					new_cmd_omega = w_test;
				}
				else {
					new_cmd_omega = scheduled_w;
				}
				
				//publish message
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = new_cmd_omega;
				if (dist_to_go<= 0.0 && segment_length >= 0.0) {
					cmd_vel.linear.x = 0.0;
					cmd_vel.angular.z = 0.0;
				}
				else if (dist_to_go > 0.0 && segment_length < 0.0 ) {
					cmd_vel.linear.x = 0.0;
					cmd_vel.angular.z = 0.0;
				}
				vel_cmd_pub.publish(cmd_vel);
				
				// finish if arrived
				if (dist_to_go <= 0.0 && segment_length >= 0.0) break;
				if (dist_to_go >= 0.0 && segment_length <= 0.0) break;
				rtimer.sleep();
			}
			ROS_INFO("completed turn distance for segment %d", (segment_index+1));
			segment_index = segment_index+1;
		} //end if statement for chosing turn or forward
		/*
			END CONTROL LOGIC
		*/
	} //end while loop to run through segments
	cmd_vel.linear.x = 0.0;
	cmd_vel.angular.z = 0.0;
	vel_cmd_pub.publish(cmd_vel);
	return 0;
	// end node
}
