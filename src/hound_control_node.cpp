#include <ros/ros.h>

#include <opencv2/opencv.hpp>
// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "ackermann_msgs/AckermannDriveStamped.h"


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

  float get_CTN(float t, float &C, cv::Point3f &Tan, cv::Point3f &Norm)
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

  float step_size(float resolution)
  {
    return resolution/arc_length;
  }
};

class Juicebox
{
public:
  ros::Subscriber sub_odom, sub_imu;
  ros::Publisher control_publisher;
  // model variables
  float D_const, B_const, C_const, wheelbase, trackwidth, cg_height, cg_ratio;
  float steering_limit, mass, wheelspeed_lim;
  float cruise_speed;
  // state variables. ENU for world frame, FLU for body frame:
  float rpy[3];
  geometry_msgs::Quaternion Q;
  cv::Point3f pos, vel, lastVel, velBF, accBF, accNED, rotBF, phi_dot;
  float Beta, speed;
  cv::Matx33f Tnb, Tbn;


  // control variables
  cv::Point3f Vhat_cur;
  float wheelspeed, steering, steering_estimate, wheelspeed_estimate;
  cv::Point3f target_WP, cur_target, cur_Vhat, target_Vhat;
  float control_Tc, speed_time_constant, wp_dist;
  bool auto_flag, setup_complete;
  int wp_index, max_index;

  Juicebox(ros::NodeHandle &nh)
  {
    Tnb = cv::Matx33f(0,0,0, 0,0,0, 0,0,0);
    Tbn = cv::Matx33f(0,0,0, 0,0,0, 0,0,0);

    sub_odom = nh.subscribe("odom", 1, &Juicebox::odom_cb, this);
    sub_imu = nh.subscribe("imu", 1, &Juicebox::imu_cb, this);
    control_publisher = nh.advertise<ackermann_msgs::AckermannDriveStamped>("control",10);
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
    if(val > max)
    {
      return max;
    }
    if(val < min);
    {
      return min;
    }
    return val;
  }

  void imu_cb(const sensor_msgs::Imu::ConstPtr msg)
  {
    accBF.x = msg->linear_acceleration.x;
    accBF.y = msg->linear_acceleration.y;
    accBF.z = msg->linear_acceleration.z;
  }

  void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
  {
    pos.x = msg->pose.pose.position.y;
    pos.y = msg->pose.pose.position.x;
    pos.z = msg->pose.pose.position.z;

    velBF.x = msg->twist.twist.linear.y;
    velBF.y = msg->twist.twist.linear.x;
    velBF.z = msg->twist.twist.linear.z;

    rotBF.x = msg->twist.twist.angular.y;
    rotBF.y = msg->twist.twist.angular.x;
    rotBF.z = msg->twist.twist.angular.z;

    Q = msg->pose.pose.orientation;
    calc_Transform(Q, Tnb, Tbn);
    rpy_from_quat(rpy);

    lastVel = vel;
    vel = Tnb * velBF;

    speed = cv::norm(velBF);
    Beta = LPF(0.2f, atan2f(velBF.y, velBF.x), Beta);
    if(speed > 1)
    {
      cv::Point3f past_vec = Tnb * lastVel / cv::norm(lastVel);
      cv::Point3f cur_vec = vel/ speed;
      cv::Point3f rot_vec = past_vec.cross(cur_vec);
      if(cv::norm(rot_vec)!=0)
      {  
        rot_vec /= cv::norm(rot_vec);
      }
      else
      {
          rot_vec = cv::Point3f(0,0,1);
      }
      phi_dot = LPF(0.2, 50*rot_vec * acosf( constrain(past_vec.dot(cur_vec), -1.0, 1.0) ), phi_dot);
    }
    else
    {
      phi_dot = rotBF;
    }

    std::cout<<"pos: "<<pos<<std::endl;
    std::cout<<"vel: "<<vel<<std::endl;
    std::cout<<"velBF: "<<velBF<<std::endl;
    std::cout<<"rpy: "<<rpy[2]*57.3<<std::endl;
    std::cout<<"phi_dot: "<<phi_dot.z*57.3<<std::endl;
    std::cout<<"Beta "<<Beta<<std::endl;
  }


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