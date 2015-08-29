#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class TeleopTwistJoy
{
public:
  TeleopTwistJoy();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  // ros nodehandler
  ros::NodeHandle n;

  // publisher (publishes cmd_vel)
  ros::Publisher vel_pub;
  // subscriber (subscribes joy)
  ros::Subscriber joy_sub;

  // params for robot 
  double linear_x_vel_max;
  double linear_y_vel_max;
  double linear_x_vel_min;
  double linear_y_vel_min;
  double angular_z_vel_max;
  double angular_z_vel_min;
  bool holonomic;

  // params joy button infomation
  int axis_linear_x;
  int axis_linear_y;
  int axis_angular_z;
  int enable_button;
  int enable_turbo_button;
};

TeleopTwistJoy::TeleopTwistJoy() :
  linear_x_vel_max(1.1),
  linear_y_vel_max(1.1),
  linear_x_vel_min(0.5),
  linear_y_vel_min(0.5),
  angular_z_vel_max(M_PI),
  angular_z_vel_min(M_PI/3.0),
  axis_linear_x(1),
  axis_linear_y(2),
  axis_angular_z(0),
  enable_button(8),
  enable_turbo_button(10),
  holonomic(true)
{

  // using parameter server
  // get joy button info
  n.param("teleop_twist_joy/axis_linear_x", axis_linear_x, axis_linear_x);
  n.param("teleop_twist_joy/axis_linear_y", axis_linear_y, axis_linear_y);
  n.param("teleop_twist_joy/axis_angular_z", axis_angular_z, axis_angular_z);
  n.param("teleop_twist_joy/enable_button", enable_button, enable_button);
  n.param("teleop_twist_joy/enable_turbo_button", enable_turbo_button, enable_turbo_button);
  // get robot info
  n.param("teleop_twist/holonomic", holonomic, holonomic);
  n.param("teleop_twist/linear_x_vel_max", linear_x_vel_max, linear_x_vel_max);
  n.param("teleop_twist/linear_y_vel_max", linear_y_vel_max, linear_y_vel_max);
  n.param("teleop_twist/linear_x_vel_min", linear_x_vel_min, linear_x_vel_min);
  n.param("teleop_twist/linear_y_vel_min", linear_y_vel_min, linear_y_vel_min);
  n.param("teleop_twist/angular_z_vel_max", angular_z_vel_max, angular_z_vel_max);
  n.param("teleop_twist/angular_z_vel_min", angular_z_vel_min, angular_z_vel_min);

  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTwistJoy::joyCallback, this);

}

void TeleopTwistJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist cmd_vel;

  if(joy->buttons[enable_button]){
    if(joy->buttons[enable_turbo_button]){
      cmd_vel.linear.x = linear_x_vel_max*joy->axes[axis_linear_x];
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = angular_z_vel_max*joy->axes[axis_angular_z];
      if(holonomic)
	cmd_vel.linear.y = linear_y_vel_max*joy->axes[axis_linear_y];
    } else{
      cmd_vel.linear.x = linear_x_vel_min*joy->axes[axis_linear_x];
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = angular_z_vel_min*joy->axes[axis_angular_z];
      if(holonomic)
	cmd_vel.linear.y = linear_y_vel_min*joy->axes[axis_linear_y];
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
  ros::init(argc, argv, "teleop_twist_joy");
  TeleopTwistJoy teleop_twist_joy;
  ros::spin();
}

