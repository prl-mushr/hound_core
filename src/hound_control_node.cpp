#include <ros/ros.h>

#include <opencv2/opencv.hpp>
// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
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

class Juicebox()
{
public:
  // model variables
  float D_const, B_const, C_const, wheelbase, trackwidth, cg_height, cg_ratio;
  float steering_limit, mass, wheelspeed_lim;
  float cruise_speed;
  // state variables. ENU for world frame, FLU for body frame:
  float quat[4], rpy[3];
  cv::Point3f pos, vel, lastVel, velBF, accBF, accNED, rotBF; 
  float Tnb[3][3], Tbn[3][3];
  float Beta, speed;

  // control variables
  cv::Point3f Vhat_cur;
  float wheelspeed, steering, steering_estimate, wheelspeed_estimate;
  cv::Point3f target_WP, cur_target, cur_Vhat, target_Vhat;
  float control_Tc, speed_time_constant, wp_dist;
  bool auto_flag, setup_complete;
  int wp_index, max_index;
  
};

int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh("~");
  // subsribe topic
  ros::MultiThreadedSpinner spinner(16); // Use one thread per core
  spinner.spin(); // spin() will not return until the node has been shutdown

  return 0;
}