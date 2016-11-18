#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

float base_speed_est = 0.0;
float base_omega_est = 0.0;

 double x = 0.0;
  double y = 0.0;
  double th = 0.0; 

float odom_dx = 0.0;

void baseSpeedCallback(const std_msgs::Float32::ConstPtr& msg)
{
	base_speed_est = msg->data;

	//ROS_INFO("base_speed_est: %f",base_speed_est);
}

void baseOmegaCallback(const std_msgs::Float32::ConstPtr& msg)
{
	base_omega_est = msg->data;

	//ROS_INFO("base_omega_est: %f", base_omega_est);
}

void odomXCallback(const std_msgs::Float32::ConstPtr& msg)
{
	odom_dx = msg->data;

        x += odom_dx * cos(th);
        y += odom_dx * sin(th);

	//ROS_INFO("odom_dx: %f", odom_dx);
}


void odomYawCallback(const std_msgs::Float32::ConstPtr& msg)
{
	th = msg->data;

	//ROS_INFO("th: %f", th);
}



int main(int argc, char** argv){
  ros::init(argc, argv, "ros_car_odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  ros::Subscriber base_speed_sub = n.subscribe("base_speed_est", 1000, baseSpeedCallback);
  ros::Subscriber base_omega_sub = n.subscribe("base_omega_est", 1000, baseOmegaCallback);

ros::Subscriber odom_dx_sub = n.subscribe("odom_x", 1000, odomXCallback);
ros::Subscriber odom_dth_sub = n.subscribe("odom_yaw", 1000, odomYawCallback);

  tf::TransformBroadcaster odom_broadcaster;

  //double x = 0.0;
  //double y = 0.0;
  //double th = 0.0;

  //double vx, vy, vth;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100);

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec(); 


    //vx = odom_x / dt; 
    //vy = 0.0;
    //vth = odom_th / dt;

     //vx = base_speed_est;
     //vy = 0.0;
     //vth = base_omega_est;

     //th += odom_th;
     //x += vx * cos(th);
     //y += vx * sin(th);

    //compute odometry in a typical way given the velocities of the robot
    
    //double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    //double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    //double delta_th = vth * dt;

    //ROS_INFO("dt:%f,odom_x:%f,odom_th:%f",dt,odom_x,odom_th);

    //x += delta_x;
    //y += delta_y;
    //th += delta_th;


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = base_speed_est;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = base_omega_est;

    // ROS_INFO("base_speed_est: %f",odom.twist.twist.linear.x);
    // ROS_INFO("base_omega_est: %f",odom.twist.twist.angular.z);

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
