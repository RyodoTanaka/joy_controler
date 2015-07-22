#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class JoyController
{
public:
  JoyController();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  // ros nodehandler
  ros::NodeHandle n;

  // publisher (publishes cmd_vel)
  ros::Publisher vel_pub;
  // subscriber (subscribes joy)
  ros::Subscriber joy_sub;

  // linear & angular limits
  double linear_vel_max;
  double angular_vel_max;
  double linear_acc_max;
  double angular_acc_max;

  // for joy control
  int axis_linear;
  int axis_angular;
  int enable_button;
  int enable_turbo_button;
  double linear_vel;
  double linear_turbo_vel;
  double angular_vel;
};

JoyController::JoyController() :
  linear_vel_max(1.1),
  angular_vel_max(M_PI),
  linear_acc_max(1.0),
  angular_acc_max(M_PI),
  axis_linear(1),
  axis_angular(0),
  enable_button(8),
  enable_turbo_button(10)
{

  // using parameter server
  n.param("joy_contoroller/linear_vel_max", linear_vel_max, linear_vel_max);
  n.param("joy_contoroller/angular_vel_max", angular_vel_max, angular_vel_max);
  n.param("joy_contoroller/linear_acc_max", linear_acc_max, linear_acc_max);
  n.param("joy_contoroller/angular_acc_max", angular_acc_max, angular_acc_max);
  n.param("joy_contoroller/axis_linear", axis_linear, axis_linear);
  n.param("joy_contoroller/axis_angular", axis_angular, axis_angular);
  n.param("joy_contoroller/enable_button", enable_button, enable_button);
  n.param("joy_contoroller/enable_turbo_button", enable_turbo_button, enable_turbo_button);

  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &JoyController::joyCallback, this);

}

void JoyController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist cmd_vel;

  if(joy->buttons[enable_button]){
    if(joy->buttons[enable_turbo_button])
      cmd_vel.linear.x = linear_vel_max*joy->axes[axis_linear];
    else
      cmd_vel.linear.x = linear_vel_max*0.7*joy->axes[axis_linear];
    cmd_vel.angular.z = angular_vel_max*joy->axes[axis_angular];
  } else {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }
  vel_pub.publish(cmd_vel);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_contoroller");
  JoyController NlabYpspurJoyCntler;
  ros::spin();
}

