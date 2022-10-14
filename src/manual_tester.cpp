#include <ros/ros.h>
#include <mavros_msgs/ManualControl.h>

ros::Publisher control_pub;

void pub_ctrl(float st, float th)
{
    mavros_msgs::ManualControl manual_control_msg;

    manual_control_msg.header.stamp = ros::Time::now();
    manual_control_msg.x = 1000;
    manual_control_msg.y = st*1000; // steering;
    manual_control_msg.z = th*1000; // throttle;
    manual_control_msg.r = 1000;
    manual_control_msg.buttons = 0;

    control_pub.publish(manual_control_msg);
}

/*
idea: check for mode switch pwm -> if it is in the range corresponding mode 2, it is guided with speed control and stability control
for speed control: gain: relate to % error in speed -> function of battery voltage, motor kv, gearing ratio.
1% error in speed = 1% increase in throttle for the P part. Same for I part but with a time constant of 1 second -> cap both to be within 5% of total (max 10% increase/decrease in throttle)

*/

int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh("~");
  control_pub = nh.advertise<mavros_msgs::ManualControl>("/mavros/manual_control/send", 10);

  // subsribe topic
  ros::Rate r(50);
  ros::Time start = ros::Time::now();
  while(ros::ok())
  {
    float time = (ros::Time::now() - start).toSec();
    float steering = sin(time);
    float throttle = sin(time)*sin(time);
    pub_ctrl(steering, throttle);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}