#include <ros/ros.h>
#include <mavros_msgs/ManualControl.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "mavros_msgs/RCIn.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/State.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "vesc_msgs/VescStateStamped.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include "unitree_legged_sdk/unitree_wireless.h"


class ll_controller
{
public:
  ros::Subscriber sub_vesc, sub_channel, sub_mode, sub_imu, sub_auto_control;
  ros::Publisher control_pub, diagnostic_pub, limits_pub;

  float wheelspeed, steering_angle;
  int switch_pos;
  bool guided;
  cv::Point3f rotBF, accBF, rpy;
  float semi_steering, semi_wheelspeed;
  float auto_steering, auto_wheelspeed;
  float auto_wheelspeed_limit;
  float manual_steering, manual_wheelspeed;

  float erpm_gain, steering_max, wheelspeed_max, wheelbase, cg_height, track_width;
  bool channel_init, vesc_init, mode_init, imu_init, auto_init;

  float motor_kv, nominal_voltage, max_rated_speed, voltage_input, K_drag;

  float speed_integral, speed_proportional, delta_t, last_throttle, throttle_delta;
  float speed_control_kp, speed_control_ki;

  bool safe_mode;

  float accel_gain, roll_gain, steer_slack, LPF_tau;
  bool liftoff_oversteer;


  // object pointer to WIRE_LESS_CONTROL class 
  static WIRE_LESS_CONTROL* wcon; 

  //for memcpying to wirelessremote field of cmd struct
  //that is to be sent to unitree

  // TRT | rc channel 4 | 1041, 1501, 1951}

  bool arm;  //L2 + B   rc channel | 1951
  bool init_mode; //L1 + start rc channel | 1041
 
  // rc channel 1501 | break
  //TRT

  bool change_mode;     //start

  ros::Rate *sleep_rate;

  ll_controller(ros::NodeHandle &nh) // constructor
  {
    sub_vesc = nh.subscribe("/sensors/core", 1, &ll_controller::vesc_cb, this);
    sub_channel = nh.subscribe("/mavros/rc/in", 1, &ll_controller::channel_cb, this);
    sub_mode = nh.subscribe("/mavros/state", 1, &ll_controller::mode_cb, this);
    sub_imu = nh.subscribe("/mavros/imu/data_raw", 1, &ll_controller::imu_cb, this);
    sub_auto_control = nh.subscribe("dawg/control", 1, &ll_controller::auto_control_cb, this);
    control_pub = nh.advertise<mavros_msgs::ManualControl>("/mavros/manual_control/send", 10);
    diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/low_level_diagnostics", 1);
    limits_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/control_limits", 1);

    guided         = false;  
    switch_pos     = 0;  // 0 = manual, 1 = semi auto, 2 = auto
    mode_init      = false;
    channel_init   = false;
    vesc_init      = false;
    imu_init       = false;
    auto_init      = false;
    delta_t        = 0.02f;  // 50 Hz loop rate
    speed_integral = 0;
    K_drag         = 0;
    last_throttle  = 0;
    
    wcon           = WIRE_LESS_CONTROL::GetUdpInstance(LOWLEVEL);
    wcon->_keyData = {0};
    arm            = false;
    init_mode      = false;
    change_mode    = false;


    if(not nh.getParam("/erpm_gain", erpm_gain))
    {
      erpm_gain = 3500.0f; 
    }
    if(not nh.getParam("/steering_max", steering_max))
    {
      steering_max = 0.488f;
    }
    if(not nh.getParam("/wheelbase", wheelbase))
    {
      wheelbase = 0.29f;
    }
    if(not nh.getParam("/cg_height", cg_height))
    {
      cg_height = 0.136f;
    }
    
    if(not nh.getParam("/wheelspeed_max", wheelspeed_max))
    {
      wheelspeed_max = 17.0f;
    }
    if(not nh.getParam("/nominal_voltage", nominal_voltage))
    {
      nominal_voltage = 14.8;
    }
    if(not nh.getParam("/motor_kv", motor_kv))
    {
      motor_kv = 3930;
    }
    if(not nh.getParam("/speed_control_kp", speed_control_kp))
    {
      speed_control_kp = 1.0f;
    }
    if(not nh.getParam("/speed_control_ki", speed_control_ki))
    {
      speed_control_ki = 1.0f;
    }
    if(not nh.getParam("/safe_mode", safe_mode))
    {
      safe_mode = true;
    }
    if(not nh.getParam("/track_width", track_width))
    {
      track_width = 0.25;
    }
    if(not nh.getParam("/accel_gain", accel_gain))
    {
      accel_gain = 1.0;
    }
    if(not nh.getParam("/roll_gain", roll_gain))
    {
      roll_gain = 0.33;
    }
    if(not nh.getParam("/steer_slack", steer_slack))
    {
      steer_slack = 0.4;
    }
    if(not nh.getParam("/LPF_tau", LPF_tau))
    {
      LPF_tau = 0.2;
    }
    if(not nh.getParam("/throttle_delta", throttle_delta))
    {
      throttle_delta = 0.02;
    }
    if(not nh.getParam("/liftoff_oversteer", liftoff_oversteer))
    {
      liftoff_oversteer = true;
    }

    // the 3930 kv rating is for "no-load". Under load the kv rating drops by 30%;
    max_rated_speed = 2 * 0.69 * motor_kv * nominal_voltage / erpm_gain;
    sleep_rate = new ros::Rate(100);
  }

  void LPF(cv::Point3f measurement, cv::Point3f &estimate)
  {
    estimate = LPF_tau*measurement + (1 - LPF_tau)*estimate;
  }

  void rpy_from_quat(geometry_msgs::Quaternion Q) 
  { 
    float q[4];
    q[0] = Q.x;
    q[1] = Q.y;
    q[2] = Q.z;
    q[3] = Q.w;
    rpy.x = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
    rpy.y = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
    rpy.z = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),1 - 2 * (q[1] * q[1] + q[2] * q[2]));
  }

  void imu_cb(const sensor_msgs::Imu::ConstPtr imu)
  {
    if(not imu_init)
    {
      imu_init = true;
    }
    cv::Point3f rotBFmeas, accBFmeas;
    rotBFmeas.x = imu->angular_velocity.x;
    rotBFmeas.y = imu->angular_velocity.y;
    rotBFmeas.z = imu->angular_velocity.z;

    accBFmeas.x = imu->linear_acceleration.x;
    accBFmeas.y = imu->linear_acceleration.y;
    accBFmeas.z = imu->linear_acceleration.z;

    LPF(accBFmeas, accBF);
    rotBF = rotBFmeas;

    rpy_from_quat(imu->orientation);
    // TODO: combined
    if(!channel_init or !mode_init or !vesc_init or switch_pos == 0 or ((switch_pos == 2 and guided ) and !auto_init))
    {
      sleep_rate->sleep();
      return;
    }
    // semi-auto or auto mode
    if((switch_pos >= 1 and guided))
    {
      float wheelspeed_setpoint, steering_setpoint;
      std::cout << "semi-auto or auto and guided" <<std::endl;
      if(switch_pos == 1)
      {
        std::cout<<"switch_pos = 1" <<std::endl;
        wheelspeed_setpoint = semi_wheelspeed;
        steering_setpoint = semi_steering;
      }
      else
      {
        wheelspeed_setpoint = std::min(auto_wheelspeed, auto_wheelspeed_limit);
        steering_setpoint = auto_steering;
      }

      // float wheelspeed_setpoint = speed_limiter(lidar scans); 
      bool intervention = false; // test code-block. Ref: https://answers.ros.org/question/262236/publishing-diagnostics-from-c/
      if(safe_mode)
      {
        steering_setpoint = steering_limiter(steering_setpoint, intervention);
      }
      float throttle_duty = speed_controller(wheelspeed_setpoint);
      pub_ctrl(steering_setpoint / steering_max, throttle_duty);
      
      diagnostic_msgs::DiagnosticArray dia_array;
      diagnostic_msgs::DiagnosticStatus robot_status;
      robot_status.name = "LL_control";
      robot_status.level = diagnostic_msgs::DiagnosticStatus::OK;
      robot_status.message = "intervention";
      diagnostic_msgs::KeyValue steering;
      steering.key = "steering";
      steering.value = std::to_string(intervention);

      diagnostic_msgs::KeyValue speed_error;
      speed_error.key = "speed_error";
      speed_error.value = std::to_string(wheelspeed_setpoint - wheelspeed);

      diagnostic_msgs::KeyValue steering_input;
      steering_input.key = "steering_input";
      steering_input.value = std::to_string(steering_setpoint);

      diagnostic_msgs::KeyValue wheelspeed_input;
      wheelspeed_input.key = "wheelspeed_input";
      wheelspeed_input.value = std::to_string(wheelspeed_setpoint);

      robot_status.values.push_back(steering);
      robot_status.values.push_back(speed_error);
      robot_status.values.push_back(steering_input);
      robot_status.values.push_back(wheelspeed_input);
      dia_array.status.push_back(robot_status);
      diagnostic_pub.publish(dia_array);
    }
    // sleep_rate->sleep();
  }

  float speed_controller(float wheelspeed_setpoint)
  {
    float throttle_duty = 0;
    float speed_error = (wheelspeed_setpoint - wheelspeed) / max_rated_speed;  // % error in speed in relation to the maximum achievable speed.

    float Kp_speed_error = speed_control_kp * speed_error;
    float Ki_speed_error_dt =  speed_control_ki * speed_error *  delta_t;

    speed_proportional = std::min(std::max(-0.05f, Kp_speed_error), 0.05f);
    speed_integral = std::min(std::max(-0.05f, Ki_speed_error_dt + speed_integral), 0.05f); // add to previous value and then constrain
    if(wheelspeed < 1)
    {
      speed_integral = 0;
      speed_proportional = 0;
    }
    // speed control kp could be varied in proportion to the rate of change of input -> higher rate = more gain.
    float voltage_gain = nominal_voltage/voltage_input;
    throttle_duty = voltage_gain*((1 + K_drag)*(wheelspeed_setpoint / max_rated_speed) + speed_error + speed_integral);
    throttle_duty = std::max(throttle_duty, 0.0f); // prevent negative values because we don't support reverse.

    throttle_duty = std::min(std::max(last_throttle - throttle_delta, throttle_duty), last_throttle + throttle_delta);
    last_throttle = throttle_duty;
    return throttle_duty;
  }

  float steering_limiter(float steering_setpoint, bool& intervention)
  {
    intervention = false;
    float whspd2 = std::max(1.0f, wheelspeed); // this is to protect against a finite/0 situation in the calculation below
    whspd2 *= whspd2;
    
    float Aylim = track_width * 0.5f * std::max(1.0f, fabs(accBF.z)) / cg_height; // taking fabs(Az) because god help you if your Az is already negative.
    if(liftoff_oversteer)
    {
      Aylim = sqrtf(std::max(Aylim*Aylim - accBF.x*accBF.x, 0.0f)); // slightly conservative estimate of Ay limit; this is done more or less specifically for the dawg
    }
    float steering_input_temp = steering_setpoint;
    float steering_limit_max =  atan2f(wheelbase * (Aylim - 9.81*sinf(rpy.x)), whspd2) + steer_slack*steering_max;
    float steering_limit_min = -atan2f(wheelbase * (Aylim + 9.81*sinf(rpy.x)), whspd2) - steer_slack*steering_max;
    // static limiter
    steering_setpoint = std::min(steering_limit_max, std::max(steering_limit_min, steering_setpoint));

    if(fabs(steering_setpoint - steering_input_temp) > 0.01)
    {
      intervention = true;
    }
    
    // this brings the car back from the roll-over.
    float Ay = accBF.y;
    float Ay_error = 0;
    float delta_steering;
    if(fabs(Ay) > Aylim)
    {
      intervention = true;
      if(Ay >= 0)
      {
        Ay_error = Aylim - Ay;
        delta_steering = (accel_gain * Ay_error - roll_gain * rotBF.x * fabs(accBF.z) )* cosf(steering_setpoint) * cosf(steering_setpoint) * wheelbase / whspd2;
        delta_steering = std::min(delta_steering, 0.0f);
      }
      else
      {
        Ay_error = -Aylim - Ay;
        delta_steering = (accel_gain * Ay_error - roll_gain * rotBF.x * fabs(accBF.z) )* cosf(steering_setpoint) * cosf(steering_setpoint) * wheelbase / whspd2;
        delta_steering = std::max(delta_steering, 0.0f);
      }
      steering_setpoint += delta_steering;
    }

    return steering_setpoint;
  }

  void channel_cb(const mavros_msgs::RCIn::ConstPtr rc)
  {
    if(rc->channels.empty())
    {
        return;
    }  
    semi_steering = steering_max * ((rc->channels[0] - 1500) / 500.0f );
    semi_wheelspeed = wheelspeed_max * ( (rc->channels[2] - 1000) / 1000.0f );

    manual_steering = steering_max * ((rc->channels[0] - 1500) / 500.0f );
    manual_wheelspeed = wheelspeed_max * ( (rc->channels[2] - 1000) / 1000.0f );

    // set auto mode speed limit using throttle too and publish as ackermann drive message that others can subscribe to
    auto_wheelspeed_limit = wheelspeed_max * ( (rc->channels[2] - 1000) / 1000.0f );
    ackermann_msgs::AckermannDriveStamped limits_msg;
    limits_msg.drive.speed = auto_wheelspeed_limit;
    limits_pub.publish(limits_msg);

    if(not channel_init)
    {
      channel_init = true;
    }

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


// For arming and initialzing the dawg
    if (rc->channels[4] > 1900)
      {
        if (init_mode)
            init_mode = false;

        if (!arm){
          arm = true;
          wcon->_keyData.btn.components.L2 = 1;
          wcon->_keyData.btn.components.B  = 1;
          memcpy(wcon->cmd.wirelessRemote, &(wcon->_keyData), 40);
        }
        else{
          if (!(wcon->_keyData.btn.value))
              wcon->_keyData.btn.components.L2 = 0;
              wcon->_keyData.btn.components.B  = 0;
        }
      }
    
    else if((rc->channels[4] < 1400) && (rc->channels[4] > 1000)){
      if (arm)
        arm = false;

      if (!init_mode){
        init_mode = true;
        wcon->_keyData.btn.components.L1    = 1;
        wcon->_keyData.btn.components.start = 1;
        memcpy(wcon->cmd.wirelessRemote, &(wcon->_keyData), 40);
      }
      else{
        if (!(wcon->_keyData.btn.value) )
          wcon->_keyData.btn.components.L1    = 0;
          wcon->_keyData.btn.components.start = 0;
      }
    }

    else{
      if (arm)
        arm = false;
      if (init_mode)
        init_mode = false;
       if (!(wcon->_keyData.btn.value))
          wcon->_keyData.btn.value = 0;
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
    voltage_input = vesc->state.voltage_input;
    float power_applied = fabs(vesc->state.duty_cycle * vesc->state.current_input);
    float Kd_meas = power_applied / std::max(1.0f, wheelspeed*wheelspeed);
    K_drag = std::min(1.0f, std::max(0.0f, 0.2f*Kd_meas + 0.8f*K_drag));
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
      manual_control_msg.y = -st*1000; // steering;
      manual_control_msg.z = th*1000; // throttle;
      manual_control_msg.r = 1000;
      manual_control_msg.buttons = 0;

      control_pub.publish(manual_control_msg);
  }

};

int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "dawg_ll_control");
  ros::NodeHandle nh("~");
  ll_controller ll_ctrl(nh);
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
