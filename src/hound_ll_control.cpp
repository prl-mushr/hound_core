#include <ros/ros.h>
#include <mavros_msgs/ManualControl.h>
#include <opencv2/opencv.hpp>
// Include opencv2 -- why? because I need something to take care of the matrix and vector stuff.
// I'm really just using opencv as a replacement for numpy in C++. could have used eigen but why switch
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "mavros_msgs/RCIn.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/State.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "vesc_msgs/VescStateStamped.h"


/*
idea: check for mode switch pwm -> if it is in the range corresponding mode 2, it is guided with speed control and stability control
for speed control: gain: relate to % error in speed -> function of battery voltage, motor kv, gearing ratio.
1% error in speed = 1% increase in throttle for the P part. Same for I part but with a time constant of 1 second -> cap both to be within 5% of total (max 10% increase/decrease in throttle)

*/

class ll_controller
{
public:
  ros::Subscriber sub_vesc, sub_channel, sub_mode, sub_imu, sub_auto_control;
  ros::Publisher control_pub;

  float wheelspeed, steering_angle;
  int switch_pos;
  bool guided;
  cv::Point3f rotBF;
  float semi_steering, semi_wheelspeed;
  float auto_steering, auto_wheelspeed;
  float manual_steering, manual_wheelspeed;

  float erpm_gain, steering_max, wheelspeed_max, wheelbase, cg_height;
  bool channel_init, vesc_init, mode_init, imu_init, auto_init;

  float motor_kv, nominal_voltage, max_rated_speed;

  float speed_integral, speed_proportional, delta_t;
  float speed_control_kp, speed_control_ki;


  ll_controller(ros::NodeHandle &nh) // constructor
  {
    sub_vesc = nh.subscribe("/sensors/core", 1, &ll_controller::vesc_cb, this);
    sub_channel = nh.subscribe("/mavros/rc/in", 1, &ll_controller::channel_cb, this);
    sub_mode = nh.subscribe("/mavros/state", 1, &ll_controller::mode_cb, this);
    sub_imu = nh.subscribe("/mavros/imu/data_raw", 1, &ll_controller::imu_cb, this);
    sub_auto_control = nh.subscribe("hound/control", 1, &ll_controller::auto_control_cb, this);
    control_pub = nh.advertise<mavros_msgs::ManualControl>("/mavros/manual_control/send", 10);


    guided = false;  
    switch_pos = 0;  // 0 = manual, 1 = semi auto, 2 = auto
    mode_init = false;
    channel_init = false;
    vesc_init = false;
    imu_init = false;
    auto_init = false;
    delta_t = 0.02f;  // 50 Hz loop rate
    speed_integral = 0;

    if(not nh.getParam("hound/erpm_gain", erpm_gain))
    {
      erpm_gain = 3500.0f; 
    }
    if(not nh.getParam("hound/steering_max", steering_max))
    {
      steering_max = 0.488f;
    }
    if(not nh.getParam("hound/wheelbase", wheelbase))
    {
      wheelbase = 0.29f;
    }
    if(not nh.getParam("hound/cg_height", cg_height))
    {
      cg_height = 0.1f;
    }
    if(not nh.getParam("hound/max_wheelspeed", wheelspeed_max))
    {
      wheelspeed_max = 17.0f;
    }
    if(not nh.getParam("hound/nominal_voltage_default", nominal_voltage))
    {
      nominal_voltage = 14.8;
    }
    if(not nh.getParam("hound/motor_kv", motor_kv))
    {
      motor_kv = 3930;
    }
    if(not nh.getParam("hound/speed_control_kp", speed_control_kp))
    {
      speed_control_kp = 1.0f;
    }
    if(not nh.getParam("hound/speed_control_ki", speed_control_ki))
    {
      speed_control_ki = 1.0f;
    }

    // the 3930 kv rating is for "no-load". Under load the kv rating drops by 30%;
    max_rated_speed = 2 * 0.69 * motor_kv * nominal_voltage / erpm_gain;

  }


  void imu_cb(const sensor_msgs::Imu::ConstPtr imu)
  {
    if(not imu_init)
    {
      imu_init = true;
    }
    rotBF.x = imu->angular_velocity.x;
    rotBF.y = imu->angular_velocity.y;
    rotBF.z = imu->angular_velocity.z;

    // std::cout<<"rotations: "<<rotBF<<"\n";

    float throttle_duty = 0;
    if(!channel_init or !mode_init or !vesc_init or switch_pos == 0)
    {
      return;
    }
    if( (switch_pos == 2 and guided ) and !auto_init)
    {
      return;
    }

    // semi-auto mode
    if((switch_pos >= 1 and guided))
    {
      float wheelspeed_setpoint, steering_setpoint;
      
      if(switch_pos == 1)
      {
        wheelspeed_setpoint = semi_wheelspeed;
        steering_setpoint = semi_steering;
      }
      else
      {
        wheelspeed_setpoint = auto_wheelspeed;
        steering_setpoint = auto_steering;
      }
      float speed_error = (wheelspeed_setpoint - wheelspeed) / max_rated_speed;  // % error in speed in relation to the maximum achievable speed.

      float Kp_speed_error = speed_control_kp * speed_error;
      float Ki_speed_error_dt =  speed_control_ki * speed_error *  delta_t;

      speed_proportional = std::min(std::max(-0.05f, Kp_speed_error), 0.05f);
      speed_integral = std::min(std::max(-0.05f, Ki_speed_error_dt + speed_integral), 0.05f); // add to previous value and then constrain

      // speed control kp could be varied in proportion to the rate of change of input -> higher rate = more gain.
      throttle_duty = (semi_wheelspeed / max_rated_speed) + speed_error + speed_integral;

      throttle_duty = std::max(throttle_duty, 0.0f); // prevent negative values because we don't support reverse.

      // std::cout<<"speed_error"<<speed_error * max_rated_speed<<"\n";

      pub_ctrl(steering_setpoint / steering_max, throttle_duty);
    }

  }

  void channel_cb(const mavros_msgs::RCIn::ConstPtr rc)
  {

    if(not channel_init)
    {
      channel_init = true;
    }
    semi_steering = steering_max * ((rc->channels[0] - 1500) / 500.0f );
    semi_wheelspeed = wheelspeed_max * ( (rc->channels[2] - 1000) / 1000.0f );

    manual_steering = steering_max * ((rc->channels[0] - 1500) / 500.0f );
    manual_wheelspeed = wheelspeed_max * ( (rc->channels[2] - 1000) / 1000.0f );

    int mode_switch = rc->channels[4];
    if(mode_switch < 1200)
    {
      switch_pos = 0;
    }
    else if(mode_switch > 1200 and mode_switch < 1800)
    {
      switch_pos = 1;
    }
    else if(mode_switch > 1800)
    {
      switch_pos = 2;
    }
    else
    {
      switch_pos = 0;
    }
  }

  void mode_cb(const mavros_msgs::State::ConstPtr state)
  {
    if(not mode_init)
    {
      mode_init = true;
    }
    guided = (state->guided & state->armed);
  }

  void vesc_cb(const vesc_msgs::VescStateStamped::ConstPtr vesc)
  {
    if(not vesc_init)
    {
      vesc_init = true;
    }
    wheelspeed = vesc->state.speed / erpm_gain;
  }

  void auto_control_cb(const ackermann_msgs::AckermannDriveStamped::ConstPtr commands)
  {
    if(not auto_init)
    {
      auto_init = true;
    }
    auto_steering = commands->drive.steering_angle;
    auto_wheelspeed = commands->drive.speed;
    return;
  }

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

};

int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "hound_ll_control");
  ros::NodeHandle nh("~");
  ll_controller ll_ctrl(nh);
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}

// int main(int argc, char **argv)
// {
//   //initialize node
//   ros::init(argc, argv, "tester");
//   ros::NodeHandle nh("~");
//   control_pub = nh.advertise<mavros_msgs::ManualControl>("/mavros/manual_control/send", 10);

//   // subsribe topic
//   ros::Rate r(50);
//   ros::Time start = ros::Time::now();
//   while(ros::ok())
//   {
//     float time = (ros::Time::now() - start).toSec();
//     float steering = sin(time);
//     float throttle = sin(time)*sin(time);
//     pub_ctrl(steering, throttle);
//     ros::spinOnce();
//     r.sleep();
//   }

//   return 0;
// }