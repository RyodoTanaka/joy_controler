#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class JoyTwist
{
public:
  JoyTwist(ros::NodeHandle &nh);

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  // publisher (publishes cmd_vel)
  ros::Publisher vel_pub;
  // subscriber (subscribes joy)
  ros::Subscriber joy_sub;

  // params for robot 
  double linear_x_vel_turbo;
  double linear_y_vel_turbo;
  double linear_x_vel_nomal;
  double linear_y_vel_nomal;
  double angular_z_vel_turbo;
  double angular_z_vel_nomal;
  bool holonomic;

  // params joy button infomation
  int axis_linear_x;
  int axis_linear_y;
  int axis_angular_z;
  int enable_button;
  int enable_turbo_button;
};

JoyTwist::JoyTwist(ros::NodeHandle &nh) :
  linear_x_vel_turbo(1.1),
  linear_y_vel_turbo(1.1),
  linear_x_vel_nomal(0.5),
  linear_y_vel_nomal(0.5),
  angular_z_vel_turbo(M_PI),
  angular_z_vel_nomal(M_PI/3.0),
  axis_linear_x(1),
  axis_linear_y(2),
  axis_angular_z(0),
  enable_button(8),
  enable_turbo_button(10),
  holonomic(true)
{

  ros::NodeHandle n("~");
  // using parameter server
  // get joy button info
  n.param("joy_conf/axis_linear_x", axis_linear_x, axis_linear_x);
  n.param("joy_conf/axis_linear_y", axis_linear_y, axis_linear_y);
  n.param("joy_conf/axis_angular_z", axis_angular_z, axis_angular_z);
  n.param("joy_conf/enable_button", enable_button, enable_button);
  n.param("joy_conf/enable_turbo_button", enable_turbo_button, enable_turbo_button);
  // get robot info
  n.param("robot_conf/holonomic", holonomic, holonomic);
  n.param("robot_conf/linear_x_vel_turbo", linear_x_vel_turbo, linear_x_vel_turbo);
  n.param("robot_conf/linear_y_vel_turbo", linear_y_vel_turbo, linear_y_vel_turbo);
  n.param("robot_conf/linear_x_vel_nomal", linear_x_vel_nomal, linear_x_vel_nomal);
  n.param("robot_conf/linear_y_vel_nomal", linear_y_vel_nomal, linear_y_vel_nomal);
  n.param("robot_conf/angular_z_vel_turbo", angular_z_vel_turbo, angular_z_vel_turbo);
  n.param("robot_conf/angular_z_vel_nomal", angular_z_vel_nomal, angular_z_vel_nomal);

  vel_pub = nh.advertise<geometry_msgs::Twist>(n.param<std::string>("cmdvel_topic_name","cmd_vel"), 1);

  joy_sub = nh.subscribe<sensor_msgs::Joy>(n.param<std::string>("joy_topic_name","joy"), 10, &JoyTwist::joyCallback, this);

}

void JoyTwist::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist cmd_vel;
  
  if(joy->buttons[enable_button]){
    if(joy->buttons[enable_turbo_button]){
      cmd_vel.linear.x = linear_x_vel_turbo*joy->axes[axis_linear_x];
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = angular_z_vel_turbo*joy->axes[axis_angular_z];
      if(holonomic)
	cmd_vel.linear.y = linear_y_vel_turbo*joy->axes[axis_linear_y];
    } else {
      cmd_vel.linear.x = linear_x_vel_nomal*joy->axes[axis_linear_x];
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = angular_z_vel_nomal*joy->axes[axis_angular_z];
      if(holonomic)
	cmd_vel.linear.y = linear_y_vel_nomal*joy->axes[axis_linear_y];
    }
  } else{
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
  }
  vel_pub.publish(cmd_vel);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_twist");
  ros::NodeHandle n;
  JoyTwist joy_twist(n);
  ros::spin();
}

