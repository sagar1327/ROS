#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot2_steering");

  ros::NodeHandle n;

  ros::Publisher robot2_steering =
    n.advertise<geometry_msgs::Twist>("robot2/wheel_controller/cmd_vel", 10);

  tf::TransformListener listener;

  ros::Rate r(100.0);
  while (n.ok()){
    tf::StampedTransform transform;
    try{
      listener.waitForTransform("robot2", "robot1", 
                               ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("robot2", "robot1",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    robot2_steering.publish(vel_msg);

    r.sleep();
  }
  return 0;
}