#include <ros/ros.h>
#include <termios.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <yocs_msgs/NavigationControl.h>
#include <yocs_msgs/NavigationControlStatus.h>

class RosCarTeleop
{

public:
  RosCarTeleop();
  char getch(void);
  void startNav(void);
  void pauseNav(void);

private:

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_, start_button_, pause_button_, start_key_, pause_key_;
  double l_scale_, a_scale_;
  std::string trajectory_name_;

  ros::Publisher cmd_vel_pub_, nav_control_pub_;
  ros::Subscriber joy_sub_;

  yocs_msgs::NavigationControl nav_ctrl;

};


RosCarTeleop::RosCarTeleop():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, 1);
  nh_.param("axis_angular", angular_, 0);
  nh_.param("scale_angular", a_scale_, 1.0);
  nh_.param("scale_linear", l_scale_, 1.0);
  nh_.param("start_button", start_button_, 0);
  nh_.param("pause_button", pause_button_, 1);
  nh_.param("start_key", start_key_, 1);
  nh_.param("pause_key", pause_key_, 1);
  nh_.param("trajectory_name", trajectory_name_, trajectory_name_);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  nav_control_pub_ = nh_.advertise<yocs_msgs::NavigationControl>("nav_ctrl", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &RosCarTeleop::joyCallback, this);

  nav_ctrl.goal_name = trajectory_name_;

}

void RosCarTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	  if (joy->buttons[pause_button_]){
      this->startNav();
	  }
	  else if (joy->buttons[start_button_]){
      this->pauseNav();
	  }

	  geometry_msgs::Twist cmd_vel;
	  cmd_vel.linear.x = l_scale_*joy->axes[linear_];
	  cmd_vel.angular.z = a_scale_*joy->axes[angular_];
	  cmd_vel_pub_.publish(cmd_vel);

}

void RosCarTeleop::startNav(void) {
      nav_ctrl.control = yocs_msgs::NavigationControl::START;
      nav_control_pub_.publish(nav_ctrl);
}

void RosCarTeleop::pauseNav(void) {
      nav_ctrl.control = yocs_msgs::NavigationControl::PAUSE;
      nav_control_pub_.publish(nav_ctrl);
}

char RosCarTeleop::getch() 
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if(rv == -1) {
		ROS_ERROR("select");
	} else if(rv == 0) {
		//ROS_INFO("no_key_pressed");
	} else {
		read(filedesc, &buff, len );
  }

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "ros_car_teleop");
	RosCarTeleop ros_car_teleop;

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		int c = 0;
		c=ros_car_teleop.getch();
		//ROS_INFO("%c", c);
    if (c == 'q'){ // ENTER Key
      ros_car_teleop.startNav();
    }
    else if (c == 'w'){ // SPACE Key
      ros_car_teleop.pauseNav();
    }

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
  
}
