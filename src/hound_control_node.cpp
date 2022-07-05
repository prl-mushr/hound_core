#include <ros/ros.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
// Include opencv2 -- why? because I need something to take care of the matrix and vector stuff.
// I'm really just using opencv as a replacement for numpy in C++. could have used eigen but why switch
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/RCOut.h"
#include "sensor_msgs/Imu.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

using namespace message_filters;

class Bezier
{
public:
  cv::Point3f P[4]; // 4 points on a bezier curve
  cv::Point3f vec[2];
  cv::Point3f K[3];
  cv::Point3f TAN[2],NORM[2];

  float dist, arc_length, L[3], T[2], C[2];
  
  Bezier(cv::Point3f start, cv::Point3f start_vec, cv::Point3f end_vec, cv::Point3f end, bool Jerk_Opt)
  {
    P[0] = start;
    P[3] = end;
    vec[0] = start_vec;
    vec[1] = end_vec;

    dist = cv::norm(P[3] - P[0]);

    if(Jerk_Opt)
    {
      float k = jerk_optimal();
      P[1] = P[0] + k*dist*vec[0];
      P[2] = P[3] - k*dist*vec[1];
    }
    else
    {
      P[1] = P[0] + 0.33f*dist*vec[0];
      P[2] = P[3] - 0.33f*dist*vec[1];
    }

    for(int i=0; i<3; i++)
    {
      L[i] = cv::norm(P[i+1] - P[i]);
    }

    arc_length = 0.5*( L[0] + L[1] + L[2] + dist );
    T[0] = 0.5f*(L[0]/(L[0]+L[1]));
    T[1] = 1.0f - 0.5f*(L[2]/(L[2]+L[1]));

    K[0] = 9.0f*P[1] + 3.0f*P[3] - 3.0f*P[0] - 9.0f*P[2];
    K[1] = 6.0f*P[0] - 12.0f*P[1] + 6.0f*P[2];
    K[2] = 3.0f*(P[1] - P[0]);

    for(int i=0;i<2;i++)
    {
      get_CTN(T[i], C[i], TAN[i], NORM[i]);  // get the curvature tangent and normal (is this necessary?)
    }
  }

  float jerk_optimal(void)
  {
    cv::Point3f P_, A, B, A_B;
    float A_B_norm, k;

    P_ = P[3] - P[0];
    P_ /= cv::norm(P_);
    A = vec[0]/cv::norm(vec[0]);
    B = -vec[1]/cv::norm(vec[1]);

    A_B = A - B;
    A_B_norm = cv::norm(A_B);
    A_B_norm *= A_B_norm;

    if(A_B_norm != 0)
    {
      k = 0.6666f*P_.dot(A_B)/A_B_norm;
      k = std::min( k, 0.5f);
    }
    else
    {
      k = 0.33f;
    }
    return k;
  }

  float distance_to_goal(cv::Point3f goal)
  {
    return cv::norm(P[3] - goal);
  }

  void get_CTN(float t, float &C, cv::Point3f &Tan, cv::Point3f &Norm)
  {
    cv::Point3f B1, B2, BNormal;
    float k_num, k_den;
    B1 = t*t*K[0] + t*K[1] + K[2]; // first derivative (also tangent) . Up to this line is the get-tangent function
    B2 = 2.0f*t*K[0] + K[1]; // second derivative 
    BNormal = B1.cross(B2);  // binormal
    k_num = cv::norm(BNormal); // numerator for curvature 
    BNormal /= k_num; // please don't be zero?
    k_den = cv::norm(B1); // denominator for curvature
    k_den *= k_den*k_den;

    C = k_num/k_den; // curvature 
    Tan = B1/cv::norm(B1); // unit tangent vector
    Norm = BNormal.cross(Tan); // unit normal vector
  }

  cv::Point3f curve(float t)
  {
    float t_[4];
    float term_1 = (1.0f - t);
    t_[3] = t*t*t;
    t_[2] = 3.0f*t*t*term_1;
    t_[1] = 3.0f*t*term_1*term_1;
    t_[0] = term_1*term_1*term_1;
    return P[0]*t_[0] + P[1]*t_[1] + P[2]*t_[2] + P[3]*t_[3];
  }

  float get_T(float dist)
  {
    return dist/arc_length;
  }
};

class Juicebox
{
private:
  cv::Point3f P0,P3;

public:
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
  message_filters::Subscriber<mavros_msgs::RCOut> rc_sub;

  ros::Publisher control_publisher;

  typedef sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry, mavros_msgs::RCOut> MySyncPolicy;
  typedef Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;


  // model variables
  float D_const, B_const, C_const, wheelbase, trackwidth, cg_height, cg_ratio;
  float steering_limit, mass, wheelspeed_lim;
  float cruise_speed;
  // state variables. ENU for world frame, FLU for body frame:
  float rpy[3];
  geometry_msgs::Quaternion Q;
  cv::Point3f pos, vel, lastVel, velBF, accBF, accNED, rotBF, phi_dot, accelVelBFFrame;
  float Beta, speed;
  cv::Matx33f Tnb, Tbn;
  // control variables
  cv::Point3f Vhat_cur;
  float wheelspeed, steering, steering_estimate, wheelspeed_estimate;
  float throttle_out, steering_out;
  cv::Point3f target_WP, cur_target, cur_Vhat, target_Vhat;
  float control_Tc, speed_time_constant, wp_dist;
  bool auto_flag, setup_complete;
  int wp_index, max_index;

  float error[1000];

  Juicebox(ros::NodeHandle &nh)
  {
    Tnb = cv::Matx33f(0,0,0, 0,0,0, 0,0,0);
    Tbn = cv::Matx33f(0,0,0, 0,0,0, 0,0,0);
    P0 = cv::Point3f(0,0,0);
    control_Tc = 0.02f;
    odom_sub.subscribe(nh, "odom", 2);
    imu_sub.subscribe(nh, "imu", 2);
    rc_sub.subscribe(nh, "rcout", 2);

    sync.reset(new Sync(MySyncPolicy(10), imu_sub, odom_sub, rc_sub));   
    sync->registerCallback(boost::bind(&Juicebox::state_cb, this, _1, _2,_3));

    control_publisher = nh.advertise<ackermann_msgs::AckermannDriveStamped>("control",10);

    for(int i = 0;i < 1000; i++)
    {
      error[i] = 0;
    }

    wheelbase = 0.29f;
    D_const = 1.2f;
    C_const = 1.4f;
    B_const = 6.5f;
    mass = 3.0f;
    cg_height = 0.1;
    throttle_out = 0;

    ROS_INFO("initialized");
  }


  void rpy_from_quat(float rpy[3]) 
  { 
    float q[4];
    q[0] = Q.x;
    q[1] = Q.y;
    q[2] = Q.z;
    q[3] = Q.w;
    rpy[2] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    rpy[0] = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
    rpy[1] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  }

  void calc_Transform(geometry_msgs::Quaternion q, cv::Matx33f &TNB, cv::Matx33f &TBN)
  {
    float q00,q11,q22,q33,q01,q02,q03,q12,q13,q23;
    cv::Mat Tnb = cv::Mat::zeros(3,3, CV_32FC1);
    cv::Mat Tbn = Tnb.clone();

    float qx, qy,qz,qw;
    qx = -q.w;
    qy = q.x;
    qz = q.y;
    qw = -q.z;

    q00 = qx*qx;
    q11 = qy*qy;
    q22 = qz*qz;
    q33 = qw*qw;
    q01 =  qx*qy;
    q02 =  qx*qz;
    q03 =  qx*qw;
    q12 =  qy*qz;
    q13 =  qy*qw;
    q23 =  qz*qw;
    
    Tbn.at<float>(0,0) = q00 + q11 - q22 - q33;
    Tbn.at<float>(1,1) = q00 - q11 + q22 - q33;
    Tbn.at<float>(2,2) = q00 - q11 - q22 + q33;
    Tbn.at<float>(0,1) = 2*(q12 - q03);
    Tbn.at<float>(0,2) = 2*(q13 + q02);
    Tbn.at<float>(1,0) = 2*(q12 + q03);
    Tbn.at<float>(1,2) = 2*(q23 - q01);
    Tbn.at<float>(2,0) = 2*(q13 - q02);
    Tbn.at<float>(2,1) = 2*(q23 + q01);

    Tnb = Tbn.t(); // transform ned->body

    TBN = cv::Matx33f(Tbn);
    TNB = cv::Matx33f(Tnb);
  }

  cv::Point3f LPF(float alpha, cv::Point3f mes, cv::Point3f est)
  {
    return alpha*mes + (1-alpha)*est;
  }

  float LPF(float alpha, float mes, float est)
  {
    return alpha*mes + (1-alpha)*est;
  }

  float constrain(float val, float min, float max)
  {
    if(val >= max)
    {
      return max;
    }
    if(val <= min)
    {
      return min;
    }
    return val;
  }

  cv::Point3f forward_vector()
  {
    cv::Point3f W;
    W.x = 1 - 2 * (Q.y*Q.y + Q.z*Q.z);
    W.y = 2 * (Q.x*Q.y + Q.w*Q.z);
    W.z = 2 * (Q.x*Q.z - Q.w*Q.y);
    return W;
  }

  cv::Point3f rotate(cv::Point3f vec, float angle)
  {
    cv::Point3f out;
    out.x = vec.x * cosf(angle) - vec.y * sinf(angle);
    out.y = vec.x * sinf(angle) + vec.y * cosf(angle);
    out.z = vec.z;
    return out;
  }

  void state_cb(const sensor_msgs::Imu::ConstPtr imu, const nav_msgs::Odometry::ConstPtr& msg, const mavros_msgs::RCOut::ConstPtr rc)
  {
    pos.x = msg->pose.pose.position.x;
    pos.y = msg->pose.pose.position.y;
    pos.z = msg->pose.pose.position.z;

    velBF.x = msg->twist.twist.linear.x;
    velBF.y = msg->twist.twist.linear.y;
    velBF.z = msg->twist.twist.linear.z;

    rotBF.x = msg->twist.twist.angular.x;
    rotBF.y = msg->twist.twist.angular.y;
    rotBF.z = msg->twist.twist.angular.z;

    cv::Point3f temp_acc;
    temp_acc.x = imu->linear_acceleration.x;
    temp_acc.y = imu->linear_acceleration.y;
    temp_acc.z = imu->linear_acceleration.z;

    accBF = LPF(0.2, temp_acc, accBF);

    throttle_out = LPF(0.5, 14.28f*(rc->channels[2] - 1050.0f)/850.0f, throttle_out);
    steering_out = 0.488f*(rc->channels[3] - 1500.0f)/250.0f;

    Q = msg->pose.pose.orientation;
    calc_Transform(Q, Tnb, Tbn);
    rpy_from_quat(rpy);

    lastVel = vel;
    vel = Tbn * velBF;

    speed = cv::norm(velBF);
    if(fabs(velBF.y) < 0.1 and fabs(velBF.x) < 0.1)
    {
      Beta = 0;
    }
    else
    {
      Beta = LPF(0.2f, atan2f(velBF.y, velBF.x), Beta);
    }
    if(speed > 1)
    {
      cv::Point3f past_vec = Tnb * lastVel / cv::norm(lastVel);
      cv::Point3f cur_vec = velBF/ speed;
      cv::Point3f rot_vec = past_vec.cross(cur_vec);
      if(cv::norm(rot_vec)!=0)
      {  
        rot_vec /= cv::norm(rot_vec);
      }
      else
      {
          rot_vec = cv::Point3f(0,0,1);
      }
      phi_dot = LPF(0.2, 50*rot_vec * acosf( constrain(past_vec.dot(cur_vec), -1.0f, 1.0f )), phi_dot);
    }
    else
    {
      phi_dot = rotBF;
    }

    accelVelBFFrame = rotate(accBF, Beta);

    // std::cout<<"pos m: "<<pos<<std::endl;
    // std::cout<<"vel m/s: "<<vel<<std::endl;
    std::cout<<"accBF m/ss: "<<sqrtf(fabs(cv::norm(accBF)*cv::norm(accBF) - 9.8*9.8))<<std::endl;
    // std::cout<<"velBF m/s: "<<velBF<<std::endl;
    // std::cout<<"rpy deg: "<<rpy[2]*57.3<<std::endl;
    // std::cout<<"phi_dot deg/s: "<<phi_dot.z*57.3<<std::endl;
    // std::cout<<"Beta deg :"<<Beta*57.3<<std::endl;
    // std::cout<<"wheelspeed m/s: "<<throttle_out<<std::endl;
    // std::cout<<"steering deg:" <<57.3 * steering_out<<std::endl;

    Tire_model_prediction();
  }

  void tire_model(float W, float Vbody, float wheelspeed, float Beta, float angle, float side,
                  float &Force, float &gamma, float &slip, float D, float C, float B)
  {
    float Vx, Vy, sx, sy;
    Vx = Vbody*cosf(Beta - angle);
    Vy = Vbody*sinf(Beta - angle);
    sx = (wheelspeed - Vx)/std::max(1.0f, std::max(wheelspeed, Vx));  // prevent 0/0 cases.
    sy = (Vy + side*wheelbase*0.5f*(rotBF.z))/ std::max(wheelspeed,1.0f);  // prevent 0/0 case
    slip = sqrtf(sx*sx + sy*sy);
    gamma = atan2f(-sy, sx);
    Force = W * D * sinf( C * atanf( B * slip));
  }

  cv::Point3f calc_forces(float Wf, float Wr, float speed, float Beta, float steering, float wheelspeed, float D, float C, float B)
  {
    float m_inv = 1.0f/mass;
    float Ff, Fr, gf, gr;
    float th_f, th_r;
    float sf, sr;
    cv::Point3f AccPred;
    tire_model(Wf, speed, wheelspeed, Beta, steering, 1, Ff, gf, sf, D, C, B);
    tire_model(Wr, speed, wheelspeed, Beta, 0,       -1, Fr, gr, sr, D, C, B);  // rear tires have no steering angle.
    
    th_f = steering - (Beta - gf);
    th_r = gr - Beta;

    AccPred.x = m_inv *( Ff*cosf(th_f) + Fr*cosf(th_r) );
    AccPred.y = m_inv *( Ff*sinf(th_f) + Fr*sinf(th_r) );
    AccPred.z = accelVelBFFrame.z;

    return AccPred;
  }

  void Tire_model_prediction()
  {
    float Wf, Wr;
    float Wb2 = mass * 9.8f * 0.5f;
    float Lt = (2 * accBF.x * mass * cg_height) / wheelbase;
    cv::Point3f AccPred;
    Wr = Wb2 + Lt;
    Wf = Wb2 - Lt;
    if(speed < 2)
    {
      return;
    }
    int index;
    float min_error = 1e7;
    float D_test, C_test, B_test;

    float spread = 1;//cv::norm(accelVelBFFrame - calc_forces(Wf, Wr, speed, Beta, steering_out, throttle_out, D_test, C_test, B_test) );
    float D_spread = spread;
    float C_spread = spread;
    float B_spread = 6*spread;
    float D_mean = 1.0, C_mean = 1.5, B_mean = 4.0;

    ros::Time start = ros::Time::now();

    for(int i = 0; i < 10; i++)
    {
      D_test = (D_mean - 0.5f*D_spread) + 0.1f*D_spread*i;
      D_test = constrain(D_test, 0.5, 1.5);
      for(int j = 0; j < 10; j++)
      {
        C_test = (C_mean - 0.5f*C_spread) + 0.1f*C_spread*j;
        C_test = constrain(C_test, 1.0, 2.0);

        for(int k = 0; k < 100; k++)
        {
          B_test = (B_mean - 0.5f*B_spread) + 0.01f*B_spread*k;
          B_test = constrain(B_test, 1.0, 7.0);

          index = 100*i + 10*j + k;
          AccPred = calc_forces(Wf, Wr, speed, Beta, steering_out, throttle_out, D_test, C_test, B_test);
          error[index] *= 0.99;
          error[index] += cv::norm(AccPred - accelVelBFFrame);
          if(error[index] < min_error)
          {
            min_error = error[index];
            D_const = D_test;
            C_const = C_test;
            B_const = B_test;
          }
        }
      }
    }
    
    AccPred = calc_forces(Wf, Wr, speed, Beta, steering_out, throttle_out, D_const, C_const, B_const);
    std::cout<<"predicted: "<<AccPred<<std::endl;
    std::cout<<"measured: "<<accelVelBFFrame << std::endl;
    std::cout<<"DCB : ";
    std::cout<<D_const<<"||";
    std::cout<<C_const<<"||";
    std::cout<<B_const<<"||";
    std::cout<<std::endl;
    return;
  }


  void pub_cur_goal()
  {
    return;
  }

  void publish_controls()
  {
    return;
  }

  // void controller()
  // {
  //   P3 = pos_target - pos;

  //   if(not auto_flag)
  //   {
  //     wheelspeed = 0;
  //     steering = 0;
  //     publish_controls();
  //   }

  //   if(speed < 0.5f)
  //   {
  //     Vhat_cur = forward_vector();
  //   }
  //   else
  //   {
  //     Vhat_cur = vel/speed;
  //   }
  //   pub_cur_goal(); // publish current goal. TBD

  //   Vhat_final = vel_target/cv::norm(vel_target); // what is vel target?

  //   Bezier traj(P0, Vhat_cur, Vhat_final, P3, true);
  //   float t = traj.get_T(speed * control_Tc);

  //   float Curvature, C_max;
  //   cv::Point3f Direction,Normal;
  //   traj.get_CTN(t, Curvature, Direction, Normal);
  //   Normal = Tnb * Normal;
  //   Direction = Tnb * Direction;

  //   C_max = std::max(traj.C[0], traj.C[1]);

  //  /// the following code is specific to cars, sort of like inverse kinematics
  //   cv::Point3f C = Curvature * Normal;
  //   float horizontal_curvature = C[1];
  //   // get the velocity in body frame, rotate normal vector into vel frame then find phi dot.
  //   float phi_dot_setp = horizontal_curvature*speed
  // }


};

int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh("~");
  // subsribe topic
  Juicebox jb(nh);
  ros::spin();

  return 0;
}