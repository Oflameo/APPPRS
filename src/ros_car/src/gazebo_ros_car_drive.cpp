
/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */

//#define USE_STEER_PID
#define SET_WORLD_SPEED

#include <algorithm>
#include <assert.h>

#include <gazebo_ros_car_drive.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo {

enum {

	LEFT_FRONT_WHEEL,
	RIGHT_FRONT_WHEEL,
	LEFT_REAR_WHEEL,
	RIGHT_REAR_WHEEL,
	LEFT_FRONT_STEER,
	RIGHT_FRONT_STEER

};

enum {
	CENTER,
	LEFT,
	RIGHT
};

double fsign(double x) {

	if (x >= 0)
		return 1;
	else
		return -1;

}

GazeboRosCarDrive::GazeboRosCarDrive() {}

// Destructor
GazeboRosCarDrive::~GazeboRosCarDrive() {}

// Load the controller
void GazeboRosCarDrive::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf ) {

	if (!ros::isInitialized()) {
	  int argc = 0;
	  char** argv = NULL;
	  ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler |
									  ros::init_options::AnonymousName);
	}

    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "RosCarDrive" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_link" );

    gazebo_ros_->getParameterBoolean ( publishJointTF_, "publishJointTF", true );
    gazebo_ros_->getParameterBoolean ( publishJointState_, "publishJointStates", true );

    gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.15 );
    gazebo_ros_->getParameter<double> ( wheel_track_width_, "wheelTrackWidth", 0.5 );
    gazebo_ros_->getParameter<double> ( wheel_base_length_, "wheelBaseLength", 1.0 );

    gazebo_ros_->getParameter<double> ( max_wheel_accel, "maxWheelAcceleration", 0.0 );
    gazebo_ros_->getParameter<double> ( max_wheel_torque, "maxWheelTorque", 10.0 );

    gazebo_ros_->getParameter<double> ( min_steer_angle_, "minSteerAngle", -0.5236 );
    gazebo_ros_->getParameter<double> ( max_steer_angle_, "maxSteerAngle",  0.5236 );
    gazebo_ros_->getParameter<double> ( max_steer_velocity, "maxSteerVelocity", 1.0 );

    gazebo_ros_->getParameter<double> ( min_turn_radius_, "minTurnRadius", wheel_track_width_/2 );
    min_turn_radius_ = fmax(fabs(min_turn_radius_),wheel_track_width_/2);

    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 1000.0 );

    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );

    joints_.resize ( 6 );

    joints_[LEFT_FRONT_WHEEL] = gazebo_ros_->getJoint ( parent, "leftFrontWheelJoint", "front_left_wheel_joint" );
    joints_[RIGHT_FRONT_WHEEL] = gazebo_ros_->getJoint ( parent, "rightFrontWheelJoint", "front_right_wheel_joint" );
    joints_[LEFT_REAR_WHEEL] = gazebo_ros_->getJoint ( parent, "leftRearWheelJoint", "rear_left_wheel_joint" );
    joints_[RIGHT_REAR_WHEEL] = gazebo_ros_->getJoint ( parent, "rightRearWheelJoint", "rear_right_wheel_joint" );
    joints_[LEFT_FRONT_STEER] = gazebo_ros_->getJoint ( parent, "leftSteerJoint", "front_left_steer_joint" );
    joints_[RIGHT_FRONT_STEER] = gazebo_ros_->getJoint ( parent, "rightSteerJoint", "front_right_steer_joint" );

	#ifdef USE_STEER_PID

		common::PID steer_pid = common::PID(100.0, 10.0, 1.0);

		this->parent->GetJointController()->SetPositionPID(
				joints_[LEFT_FRONT_STEER]->GetScopedName(), steer_pid);

		this->parent->GetJointController()->SetPositionPID(
				joints_[RIGHT_FRONT_STEER]->GetScopedName(), steer_pid);

	#endif

    this->publish_tf_ = true;
    if (!_sdf->HasElement("publishTf")) {
      ROS_WARN("GazeboRosCarDrive Plugin (ns = %s) missing <publishTf>, defaults to %d",
          this->robot_namespace_.c_str(), this->publish_tf_);
    } else {
      this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;

    last_update_time_ = parent->GetWorld()->GetSimTime();

    // Initialize velocity stuff
	for ( int wheel_i = 0; wheel_i < 4; wheel_i++ ) {
		wheel_speed_cmd_[wheel_i] = 0;
		wheel_speed_applied_[wheel_i] = 0;
	}
	for ( int steer_i = 0; steer_i < 2; steer_i++ ) {
		steer_angle_cmd_[steer_i] = 0;
		steer_angle_applied_[steer_i] = 0;
	}

    base_speed_cmd_ = 0;
    base_omega_cmd_ = 0;
    alive_ = true;

    if (this->publishJointState_)
    {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
        ROS_INFO("%s: Advertise joint_states!", gazebo_ros_->info());
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                boost::bind(&GazeboRosCarDrive::cmdVelCallback, this, _1),
                ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO("%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());

    if (this->publish_tf_)
    {
      odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
      ROS_INFO("%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_topic_.c_str());
    }

    // start custom queue for drive control plugin
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &GazeboRosCarDrive::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosCarDrive::UpdateChild, this ) );

}

void GazeboRosCarDrive::Reset() {

	last_update_time_ = parent->GetWorld()->GetSimTime();

	pose_encoder_.x = 0.0;
	pose_encoder_.y = 0.0;
	pose_encoder_.theta = 0.0;

	base_speed_cmd_ = 0.0;
	base_omega_cmd_ = 0.0;

	steer_angle_cmd_[RIGHT] = 0.0; steer_angle_cmd_[CENTER] = 0.0; steer_angle_cmd_[LEFT] = 0.0;

	joints_[LEFT_FRONT_STEER]->SetPosition(0, steer_angle_cmd_[LEFT]);
	joints_[RIGHT_FRONT_STEER]->SetPosition (0, steer_angle_cmd_[RIGHT]);

    joints_[LEFT_FRONT_WHEEL]->SetVelocity (0, 0.0);
    joints_[RIGHT_FRONT_WHEEL]->SetVelocity (0, 0.0);
    joints_[LEFT_REAR_WHEEL]->SetVelocity (0, 0.0);
    joints_[RIGHT_REAR_WHEEL]->SetVelocity (0, 0.0);

    linear_vel_cmd_.x = 0.0; linear_vel_cmd_.y = 0.0; linear_vel_cmd_.z = 0.0;
	this->parent->SetLinearVel(linear_vel_cmd_);

	angular_vel_cmd_.x = 0.0; angular_vel_cmd_.y = 0.0; angular_vel_cmd_.z = 0.0;
	parent->SetAngularVel(angular_vel_cmd_);

}

void GazeboRosCarDrive::publishJointStates() {

    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints_.size() );
    joint_state_.position.resize ( joints_.size() );

    for ( int joint_i = 0; joint_i < joints_.size(); joint_i++ ) {

    	physics::JointPtr joint = joints_[joint_i];

    	joint_state_.name[joint_i] = joint->GetName();

        gazebo::math::Angle angle = joint->GetAngle (0);

        joint_state_.position[joint_i] = angle.Radian () ;

    }

    joint_state_publisher_.publish ( joint_state_ );

}

void GazeboRosCarDrive::publishJointTF() {

    ros::Time current_time = ros::Time::now();
    for ( int joint_i = 0; joint_i < joints_.size(); joint_i++ ) {

        std::string joint_frame = gazebo_ros_->resolveTF(joints_[joint_i]->GetChild()->GetName ());
        std::string joint_parent_frame = gazebo_ros_->resolveTF(joints_[joint_i]->GetParent()->GetName ());

        math::Pose poseJoint = joints_[joint_i]->GetChild()->GetRelativePose();

        tf::Quaternion qt ( poseJoint.rot.x, poseJoint.rot.y, poseJoint.rot.z, poseJoint.rot.w );
        tf::Vector3 vt ( poseJoint.pos.x, poseJoint.pos.y, poseJoint.pos.z );

        tf::Transform tfJoint ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( tfJoint, current_time, joint_parent_frame, joint_frame ) );

    }

}

// Update the controller
void GazeboRosCarDrive::UpdateChild()
{

    common::Time current_time = parent->GetWorld()->GetSimTime();
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ ) {

        if (this->publish_tf_) publishOdometry ( seconds_since_last_update );
        if ( publishJointTF_ ) publishJointTF();
        if ( publishJointState_ ) publishJointStates();

        // Update robot in case new velocities have been requested
        getJointVelocityCmd();

        //////////////////////////////////////////////////////////////////////////////////////////////

		#ifdef USE_STEER_PID
			this->parent->GetJointController()->SetPositionTarget(joints_[LEFT_FRONT_STEER]->GetScopedName(), steer_angle_cmd_[LEFT]);
			this->parent->GetJointController()->SetPositionTarget(joints_[RIGHT_FRONT_STEER]->GetScopedName(), steer_angle_cmd_[RIGHT]);
			this->parent->GetJointController()->Update();
		#else
			joints_[LEFT_FRONT_STEER]->SetPosition( 0, steer_angle_cmd_[LEFT]);
			joints_[RIGHT_FRONT_STEER]->SetPosition ( 0, steer_angle_cmd_[RIGHT]);
		#endif

		//////////////////////////////////////////////////////////////////////////////////////////////
		// wheel_speed_cmd_ in rad/s

		/*
		joints_[LEFT_FRONT_WHEEL]->SetVelocity ( 0, wheel_speed_cmd_[LEFT_FRONT_WHEEL]);
		joints_[RIGHT_FRONT_WHEEL]->SetVelocity ( 0, wheel_speed_cmd_[RIGHT_FRONT_WHEEL]);

                joints_[LEFT_REAR_WHEEL]->SetVelocity ( 0, wheel_speed_cmd_[LEFT_REAR_WHEEL]);
		joints_[RIGHT_REAR_WHEEL]->SetVelocity ( 0, wheel_speed_cmd_[RIGHT_REAR_WHEEL]);
                */

		//////////////////////////////////////////////////////////////////////////////////////////////

		#ifdef SET_WORLD_SPEED

			world_pose_ = this->parent->GetWorldPose();

			linear_vel_cmd_.x = cosf(world_pose_.rot.GetYaw())*base_speed_cmd_;
			linear_vel_cmd_.y = sinf(world_pose_.rot.GetYaw())*base_speed_cmd_;

			angular_vel_cmd_.z = base_omega_cmd_;

			this->parent->SetLinearVel(linear_vel_cmd_);
			this->parent->SetAngularVel(angular_vel_cmd_);

		#endif

        last_update_time_+= common::Time ( update_period_ );

        //////////////////////////////////////////////////////////////////////////////////////////////
        // Debug output
		//steer_angle = joints_[LEFT_FRONT_STEER]->GetAngle ( 0 );
		ROS_INFO("yaw = %f", world_pose_.rot.GetYaw());
		ROS_INFO("base speed = %f", base_speed_cmd_);
		ROS_INFO("x = %f", linear_vel_cmd_.x);
		ROS_INFO("y = %f", linear_vel_cmd_.y);

		//steer_angle = joints_[RIGHT_FRONT_STEER]->GetAngle ( 0 );
		//ROS_INFO("steer_angle_cmd = %f, steer_angle = %f",steer_angle_cmd_[RIGHT], steer_angle.Radian());
        //ROS_INFO("wheel_speed_cmd = %f, wheel_speed = %f",wheel_speed_cmd_[LEFT_REAR_WHEEL], joints_[LEFT_REAR_WHEEL]->GetVelocity ( 0 ));
		//ROS_INFO("wheel_speed_cmd [RL,RR] = [%f,%f], wheel_speed [RL,RR] = [%f,%f]",
		//	wheel_speed_cmd_[LEFT_REAR_WHEEL]*( wheel_diameter_ / 2.0 ), wheel_speed_cmd_[RIGHT_REAR_WHEEL]*( wheel_diameter_ / 2.0 ),
		//	joints_[LEFT_REAR_WHEEL]->GetVelocity ( 0 )*( wheel_diameter_ / 2.0 ),joints_[RIGHT_REAR_WHEEL]->GetVelocity ( 0 )*( wheel_diameter_ / 2.0 ));

    }


}

// Finalize the controller
void GazeboRosCarDrive::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosCarDrive::getJointVelocityCmd()
{
    boost::mutex::scoped_lock scoped_lock ( lock );

    // commanded
    double base_vx = base_speed_cmd_;
    double base_omega = base_omega_cmd_;

    turn_radius_[CENTER] = base_vx/base_omega;

    if (fabs(base_omega) > 0 && fabs(turn_radius_[CENTER]) >= min_turn_radius_) {

		turn_radius_[LEFT] = turn_radius_[CENTER] - wheel_track_width_/2;
		turn_radius_[RIGHT] = turn_radius_[CENTER] + wheel_track_width_/2;

		steer_angle_cmd_[CENTER] = atan(wheel_base_length_/turn_radius_[CENTER]);

		wheel_speed_cmd_[LEFT_REAR_WHEEL] = turn_radius_[LEFT] * base_omega / ( wheel_diameter_ / 2.0 );
		wheel_speed_cmd_[RIGHT_REAR_WHEEL] = turn_radius_[RIGHT] * base_omega / ( wheel_diameter_ / 2.0 );

	    steer_angle_cmd_[LEFT] = atan(wheel_base_length_/turn_radius_[LEFT]);
	    steer_angle_cmd_[RIGHT] = atan(wheel_base_length_/turn_radius_[RIGHT]);

	    wheel_speed_cmd_[LEFT_FRONT_WHEEL] = turn_radius_[LEFT]/cos(steer_angle_cmd_[LEFT]) * base_omega / ( wheel_diameter_ / 2.0 );
	    wheel_speed_cmd_[RIGHT_FRONT_WHEEL] = turn_radius_[RIGHT]/cos(steer_angle_cmd_[RIGHT]) * base_omega / ( wheel_diameter_ / 2.0 );

    } else {

    	steer_angle_cmd_[CENTER] = 0;
    	steer_angle_cmd_[LEFT] = 0;
    	steer_angle_cmd_[RIGHT] = 0;

        wheel_speed_cmd_[LEFT_REAR_WHEEL] = base_vx / ( wheel_diameter_ / 2.0 );
        wheel_speed_cmd_[RIGHT_REAR_WHEEL] = base_vx / ( wheel_diameter_ / 2.0 );

        wheel_speed_cmd_[LEFT_FRONT_WHEEL] = base_vx / ( wheel_diameter_ / 2.0 );
        wheel_speed_cmd_[RIGHT_FRONT_WHEEL] = base_vx / ( wheel_diameter_ / 2.0 );

    }

}

void GazeboRosCarDrive::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg ) {

    boost::mutex::scoped_lock scoped_lock ( lock );
    base_speed_cmd_ = cmd_msg->linear.x;
    base_omega_cmd_ = cmd_msg->angular.z;

}

void GazeboRosCarDrive::QueueThread() {

    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }

}

void GazeboRosCarDrive::UpdateOdometryEncoder() {


}

void GazeboRosCarDrive::publishOdometry ( double step_time ) {

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
    std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

    // getting data form gazebo world
    math::Pose pose_ = this->parent->GetWorldPose();
    tf::Quaternion qt = tf::Quaternion ( pose_.rot.x, pose_.rot.y, pose_.rot.z, pose_.rot.w );
    tf::Vector3 vt = tf::Vector3 ( pose_.pos.x, pose_.pos.y, pose_.pos.z );

	odom_.pose.pose.position.x = vt.x();
	odom_.pose.pose.position.y = vt.y();
	odom_.pose.pose.position.z = vt.z();

	odom_.pose.pose.orientation.x = qt.x();
	odom_.pose.pose.orientation.y = qt.y();
	odom_.pose.pose.orientation.z = qt.z();
	odom_.pose.pose.orientation.w = qt.w();

	math::Vector3 linear_vel_ = this->parent->GetWorldLinearVel();
	math::Vector3 angular_vel_ = this->parent->GetWorldAngularVel();

	odom_.twist.twist.angular.z = angular_vel_.z;

    float yaw_ = pose_.rot.GetYaw();
    odom_.twist.twist.linear.x = cosf ( yaw_ ) * linear_vel_.x + sinf ( yaw_ ) * linear_vel_.y;
	odom_.twist.twist.linear.y = cosf ( yaw_ ) * linear_vel_.y - sinf ( yaw_ ) * linear_vel_.x;

    tf::Transform base_footprint_to_odom ( qt, vt );
    transform_broadcaster_->sendTransform (
        tf::StampedTransform ( base_footprint_to_odom, current_time,
                               odom_frame, base_footprint_frame ) );

    // set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;

    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish ( odom_ );

}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosCarDrive )
}



