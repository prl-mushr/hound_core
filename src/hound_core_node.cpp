/*
  * OpenCV Example using ROS and CPP
  */

 // Include the ROS library
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <mavros_msgs/ManualControl.h>
#include <boost/thread/thread.hpp>

namespace enc = sensor_msgs::image_encodings;

class Bezier
{
public:
  cv::Point2f P[4]; // 4 points on a bezier curve
  cv::Point2f vec[2];
  cv::Point2f K[3];
  float dist, arc_length, L[3], T[2], C[2];
  Bezier(cv::Point3f start, cv::Point3f end)
  {
    P[0].x = start.x;
    P[0].y = start.y;
    
    P[3].x = end.x;
    P[3].y = end.y;

    vec[0].x = cosf(start.z);
    vec[0].y = sinf(start.z);
    vec[1].x = -cosf(end.z);
    vec[1].y = -sinf(end.z);

    dist = cv::norm(P[3] - P[0]);

    P[1] = P[0] + 0.33*dist*vec[0];
    P[2] = P[3] + 0.33*dist*vec[1];

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

    C[0] = C_from_t(T[0]);
    C[1] = C_from_t(T[1]);
  }

  float distance_to_goal(cv::Point3f goal)
  {
    cv::Point2f g(goal.x, goal.y);
    return cv::norm(P[3] - g);
  }

  float C_from_t(float t)
  {
    cv::Point2f del[2];
    float denominator,Curvature;
    del[0] = t*t*K[0] + t*K[1] + K[2];
    del[1] = 2.0f*t*K[0] + K[1];
    denominator = cv::norm(del[0]); 
    denominator *= denominator*denominator;
    Curvature = ((del[0].x*del[1].y) - (del[0].y*del[1].x));
    Curvature /= denominator;

    return Curvature;
  }

  cv::Point2f curve(float t)
  {
    float t_[4];
    float term_1 = (1.0f - t);
    t_[3] = t*t*t;
    t_[2] = 3.0f*t*t*term_1;
    t_[1] = 3.0f*t*term_1*term_1;
    t_[0] = term_1*term_1*term_1;
    return P[0]*t_[0] + P[1]*t_[1] + P[2]*t_[2] + P[3]*t_[3];
  }

  cv::Point2f normal(float t)
  {
    cv::Point2f del, out;
    del = t*t*K[0] + t*K[1] + K[2];
    out.x = -del.y;
    out.y = del.x;
    float length = cv::norm(out);
    if(length != 0)
    {
      out /= length;
    }
    return out;
  }

  cv::Point2f tangent(float t)
  {
    cv::Point2f out;
    out = t*t*K[0] + t*K[1] + K[2];
    float length = cv::norm(out);
    if(length != 0)
    {
      out /= length;
    }
    return out;
  }

  float step_size(float resolution)
  {
    return resolution/arc_length;
  }
};


class hound_core
{
public:
  ros::Subscriber sub_depth_image, sub_cam_info, sub_pose, sub_scan, sub_path;
  ros::Publisher control_pub;
  ros::Timer process_timer;
  float fx_inv, fy_inv, fx, fy;
  float cx, cy;
  int height, width;
  float max_depth, roi[4], cam_pos[3];
  bool cam_init;

  float max_height;
  float min_height;
  float height_range_inv;

  float costmap_length_m;
  float costmap_width_m;
  float resolution_m;
  int costmap_height_p, costmap_width_p;
  float cam_pitch, depth_err_coeff;
  float cp, sp;
  float map_update_rate;
  cv::Mat translation_matrix, translated_image;

  bool float_flag, depth_init, scan_init;

  geometry_msgs::PoseStamped::ConstPtr cur_pose;
  bool pose_init;

  std::vector<geometry_msgs::PoseStamped::ConstPtr> depth_poses;
  std::vector<cv_bridge::CvImagePtr> depth_image_ptrs;

  std::vector<geometry_msgs::PoseStamped::ConstPtr> scan_poses, last_scan_poses;
  std::vector<sensor_msgs::LaserScan::ConstPtr> scan_msgs;
  float lidar_pos[3];
  float max_scan_range;

  cv::Mat map, gradmap, bound_map, final_map, confidence_map;
  geometry_msgs::PoseStamped::ConstPtr last_map_pose;
  bool new_map;

  //---- planning stuff -------
  bool path_received, visualize_path;
  std::vector<cv::Point3f> path;
  float lookahead_distance;
  std::vector<Bezier> traj;
  float goal_cost_multiplier, car_width, car_length, car_length_full;
  int step_x, step_y;
  float width_y, length_x, resol_x, resol_y;
  float last_curvature, last_length;
  float delta_curvature_cost, delta_length_cost;
  float last_steering, last_speed, speed_meas;

  hound_core(ros::NodeHandle &nh) // constructor
  {
    cam_init = false;
    pose_init = false;
    float_flag = false;
    depth_init = false;
    scan_init = false;

    new_map = false;
    path_received = false;


    sub_depth_image = nh.subscribe("image", 1, &hound_core::depth_image_cb, this);
    sub_cam_info = nh.subscribe("camera_info", 10, &hound_core::cam_info_cb, this);
    sub_pose = nh.subscribe("pose", 1, &hound_core::pose_cb, this);
    sub_scan = nh.subscribe("scan", 1, &hound_core::scan_cb, this);
    sub_path = nh.subscribe("path", 1, &hound_core::path_cb, this);
    control_pub = nh.advertise<mavros_msgs::ManualControl>("/mavros/manual_control/send", 10);

    if(not nh.getParam("hound/cam_pitch",cam_pitch))
    {
      cam_pitch = 0;
    }
    if(not nh.getParam("hound/depth_err_coeff", depth_err_coeff))
    {
      depth_err_coeff = 0.004f; 
    }
    if(not nh.getParam("hound/max_height", max_height))
    {
      max_height = 0.5;
    }
    if(not nh.getParam("hound/min_height", min_height))
    {
      min_height = -1.3;
    }
    if(not nh.getParam("hound/costmap_length_m", costmap_length_m))
    {
      costmap_length_m = 20;
    }
    if(not nh.getParam("hound/costmap_width_m", costmap_width_m))
    {
      costmap_width_m = 20;
    }
    if(not nh.getParam("hound/resolution_m", resolution_m))
    {
      resolution_m = 0.1;
    }
    if(not nh.getParam("hound/float_encoding_img", float_flag))
    {
      float_flag = false;
    }
    if(not nh.getParam("hound/max_depth", max_depth))
    {
      max_depth = 10.0f;
    }
    if(not nh.getParam("hound/max_scan_range", max_scan_range))
    {
      max_scan_range = 10.0f;
    }
    if(not nh.getParam("hound/map_update_rate", map_update_rate))
    {
      map_update_rate = 10;
    }

    for(int i = 0; i< 3; i++)
    {
      if(not nh.getParam("hound/cam_pos_"+std::to_string(i), cam_pos[i]))
      {
        cam_pos[i] = 0;
      }
    }

    for(int i=0;i<3;i++)
    {
      if(not nh.getParam("hound/lidar_pos_"+std::to_string(i), lidar_pos[i]))
      {
        lidar_pos[i] = 0;
      }
    }

    for(int i = 0;i < 4;i++)
    {
      if(not nh.getParam("hound/roi_" + std::to_string(i), roi[i]))
      {
        roi[i] = 1.0f;
      }
      ROS_INFO("roi_%d = %f",i,roi[i]);
    }

    if(not nh.getParam("hound/lookahead_distance", lookahead_distance))
    {
      lookahead_distance = 5;
    }

    if(not nh.getParam("hound/visualize_path", visualize_path))
    {
      visualize_path = false;
    }
    if(not nh.getParam("hound/lookahead_distance", lookahead_distance))
    {
      lookahead_distance = 10;
    }
    if(not nh.getParam("hound/car_width", car_width))
    {
      car_width = 2;
    }
    if(not nh.getParam("hound/car_length", car_length))
    {
      car_length = 2;
    }
    if(not nh.getParam("hound/goal_cost_multiplier", goal_cost_multiplier))
    {
      goal_cost_multiplier = 5;
    }
    if(not nh.getParam("hound/length_x", length_x))
    {
      length_x = 7;
    }
    if(not nh.getParam("hound/width_y", width_y))
    {
      width_y = 3;
    }
    if(not nh.getParam("hound/step_x", step_x))
    {
      step_x = 5;
    }
    if(not nh.getParam("hound/step_y", step_y))
    {
      step_y = 5;
    }
    if(not nh.getParam("hound/delta_curvature_cost", delta_curvature_cost))
    {
      delta_curvature_cost = 10;
    }
    if(not nh.getParam("hound/delta_length_cost", delta_length_cost))
    {
      delta_length_cost = 10;
    }
    costmap_height_p = costmap_length_m/resolution_m;
    costmap_width_p  = costmap_width_m/resolution_m;

    map = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    confidence_map = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    bound_map = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    gradmap = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    cp = cosf(cam_pitch);
    sp = sinf(cam_pitch);
    height_range_inv = 1/(max_height - min_height);


    resol_x = length_x/float(step_x);
    resol_y = 2*width_y/float(step_y);
    last_length = 0;
    last_curvature = 0;
    last_speed = 0;
    last_steering = 0;

    boost::thread proc_thread(&hound_core::mapping_run, this);
    boost::thread plan_thread(&hound_core::planner_run, this);

  }

  void rpy_from_quat(float rpy[3],const geometry_msgs::PoseStamped::ConstPtr& msg) 
  { 
    float q[4];
    q[0] = msg->pose.orientation.x;
    q[1] = msg->pose.orientation.y;
    q[2] = msg->pose.orientation.z;
    q[3] = msg->pose.orientation.w;
    rpy[2] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    rpy[0] = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
    rpy[1] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  }


  void cam_info_cb(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    fx = info_msg->K[0];
    fy = info_msg->K[4];
    fx_inv = 1.0f/fx;
    fy_inv = 1.0f/fy;
    cx = info_msg->K[2];
    cy = info_msg->K[5];
    height = info_msg->height;
    width = info_msg->width;
    if(!cam_init)
    { 
      cam_init = true;
    }
  }

  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    cur_pose = msg;
    if(!pose_init)
    {
      last_map_pose = msg;
      pose_init = true;
    }
    return;
  }

  void calc_Transform(const geometry_msgs::PoseStamped::ConstPtr& msg, cv::Matx33f &TNB, cv::Matx33f &TBN)
  {
    geometry_msgs::Quaternion q = msg->pose.orientation;
    calc_Transform_ll(q, TNB, TBN);
  }
  void calc_Transform_ll(geometry_msgs::Quaternion q, cv::Matx33f &TNB, cv::Matx33f &TBN)
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

  void depth_image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    if(!pose_init)
    {
      return;
    }
    try
    {
      cv_bridge::CvImagePtr cv_ptr;
      if(!float_flag)
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      }
      else
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      }
      if(depth_image_ptrs.size() >= 3)
      {
        depth_image_ptrs.pop_back();
        depth_poses.pop_back();
      }
      depth_image_ptrs.insert(depth_image_ptrs.begin(), cv_ptr);
      depth_poses.insert(depth_poses.begin(), cur_pose);
      if(!depth_init)
      {
        depth_init = true;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    return;
  }

  void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    if(!pose_init)
    {
      return;
    }
    if(last_scan_poses.empty())
    {
      last_scan_poses.insert(last_scan_poses.begin(), cur_pose);
      return;
    }
    if(scan_msgs.size() >= 2)
    {
      scan_msgs.pop_back();
      scan_poses.pop_back();
      last_scan_poses.pop_back();
    }
    scan_msgs.insert(scan_msgs.begin(), msg);
    scan_poses.insert(scan_poses.begin(), cur_pose);
    last_scan_poses.insert(last_scan_poses.begin(), cur_pose); // this vector's back is actually one extra from scan poses so it's ass always points to the t-1 pose
    if(!scan_init)
    {
      scan_init = true;
    }
    return;
  }

  void transform_map(geometry_msgs::PoseStamped::ConstPtr& map_pose)
  {
    float delta[2];
    delta[0] = map_pose->pose.position.x - last_map_pose->pose.position.x;
    delta[1] = map_pose->pose.position.y - last_map_pose->pose.position.y;
    last_map_pose = map_pose;
    
    //  define translation center as camera's location in map image
    float warp_values[] = { 1.0, 0.0, int(delta[0]/resolution_m), 0.0, 1.0, int(-delta[1]/resolution_m) };  // translation values. Must be rounded to nearest integer to avoid blurring of elevation map
    translation_matrix = cv::Mat(2, 3, CV_32F, warp_values);    // fill in the values
    
    cv::warpAffine(map, translated_image, translation_matrix, map.size());  // translate
    map = translated_image.clone();

    cv::warpAffine(confidence_map, translated_image, translation_matrix, map.size());  // translate
    confidence_map = translated_image.clone();
    confidence_map *= 0.95;
  }

  void process_depth(geometry_msgs::PoseStamped::ConstPtr& map_pose)
  {
    cv_bridge::CvImagePtr cv_ptr;
    geometry_msgs::PoseStamped::ConstPtr img_pose;
    cv::Matx33f Tnb = cv::Matx33f(0,0,0, 0,0,0, 0,0,0), Tbn = cv::Matx33f(0,0,0, 0,0,0, 0,0,0);
    cv::Point3f p_odom, p_body, p_delta;
    float x,z, temp_x, temp_z;
    float depth, pixel_height_est, pixel_height_mes, est_err, mes_err, tot_err_inv, gain;
    int row_pix, column_pix;
    int u_start, u_end, v_start, v_end;
    u_start = (1 - roi[0])*width;
    u_end = roi[1]*width;
    v_start = (1 - roi[2])*height;
    v_end = roi[3]*height;
    int depth_image_num = depth_image_ptrs.size();
    ros::Time start = ros::Time::now();

    for(int k = 0; k < depth_image_num; k++)
    {
      cv_ptr = depth_image_ptrs.back();
      img_pose = depth_poses.back();
      // pop the end.
      depth_image_ptrs.pop_back();
      depth_poses.pop_back();

      calc_Transform(img_pose, Tnb, Tbn);
      p_delta.x = map_pose->pose.position.x - img_pose->pose.position.x;
      p_delta.y = map_pose->pose.position.y - img_pose->pose.position.y;
      p_delta.z = map_pose->pose.position.z - img_pose->pose.position.z;

      for(int u = u_start; u < u_end; u++)
      {
        for(int v = v_start; v < v_end; v++)
        {
          if(float_flag)
          {
            depth = cv_ptr->image.at<float>(v,u);
          }
          else
          {
            depth = float(cv_ptr->image.at<u_int16_t>(v,u))*1e-3;
          }
          if(depth > max_depth or depth <= 0.1f)
          {
            continue;
          }
          
          x = depth;
          z = (cy - float(v))*x*fy_inv;
          p_body.y = (cx - float(u))*x*fx_inv + cam_pos[1];

          temp_x = x;
          temp_z = z;
          // find points in body frame-- this is where you should add the body frame offsets
          p_body.x = temp_x*cp - temp_z*sp + cam_pos[0];
          p_body.z = temp_x*sp + temp_z*cp + cam_pos[2];

          p_odom = Tbn * p_body - p_delta;
          if( (p_odom.z < max_height) and (p_body.z > min_height) and fabs(p_odom.x) < costmap_width_m*0.5f and fabs(p_odom.y) < costmap_length_m*0.5 )
          {
            row_pix = int((p_odom.y + costmap_length_m*0.5) / resolution_m);
            column_pix = int((-p_odom.x + costmap_width_m*0.5) / resolution_m);

            pixel_height_est = map.at<float>(row_pix, column_pix);
            est_err = (1 - confidence_map.at<float>(row_pix, column_pix));
            mes_err = depth_err_coeff*depth;
            tot_err_inv = 1/(est_err + mes_err);

            pixel_height_mes = p_odom.z + map_pose->pose.position.z; // add global ref height with the hopes that ardupilot didn't forget to subtract AMSL and fuck me over

            map.at<float>(row_pix, column_pix) = (pixel_height_est * mes_err + pixel_height_mes * est_err ) * tot_err_inv;
            est_err *= mes_err*tot_err_inv;
            confidence_map.at<float>(row_pix, column_pix) = 1.0f - est_err;
          }
        }
      }
      // In some cases processing just 1 image may take up all the compute we have. 
      // Not done for lidar because lidar only has 500 points which can be processed quickly.
      if((ros::Time::now() - start).toSec() > 0.7f / map_update_rate)
      {
        ROS_INFO("skip");
        depth_poses.clear();
        depth_image_ptrs.clear();
        break;
      }
    }
  }

  geometry_msgs::Quaternion slerp(geometry_msgs::Quaternion quat1,geometry_msgs::Quaternion quat2, float t)
  {
    geometry_msgs::Quaternion q;
    float qr = quat1.w * quat2.w + quat1.x * quat2.x + quat1.y * quat2.y + quat1.z * quat2.z;
    float ss = (float)1.0 - qr * qr;
    if (ss == 0.0f)
    {
      return quat1;
    }
    else
    {
      float sp = sqrtf(ss);
      float ph = acosf(qr);
      float pt = ph * t;
      float t1 = sinf(pt) / sp;
      float t0 = sinf(ph - pt) / sp;

      q.w = quat1.w * t0 + quat2.w * t1;
      q.x = quat1.x * t0 + quat2.x * t1;
      q.y = quat1.y * t0 + quat2.y * t1;
      q.z = quat1.z * t0 + quat2.z * t1;
      return q;
    }
  }

  void interpolate(const geometry_msgs::PoseStamped::ConstPtr& start, const geometry_msgs::PoseStamped::ConstPtr& end, float ratio, geometry_msgs::PoseStamped &result)
  {
    result.pose.position.x = (1.0f - ratio) * (start->pose.position.x) + ratio * end->pose.position.x;
    result.pose.position.y = (1.0f - ratio) * (start->pose.position.y) + ratio * end->pose.position.y;
    result.pose.position.z = (1.0f - ratio) * (start->pose.position.z) + ratio * end->pose.position.z;
    result.pose.orientation = slerp(start->pose.orientation, end->pose.orientation, ratio);
    return;
  }

  cv::Point3f polar_to_cartesian_lidar(float angle, float range)
  {
    return cv::Point3f(range*cosf(angle) + lidar_pos[0], range*sinf(angle) + lidar_pos[1], lidar_pos[2]);
  }

  void process_scan(geometry_msgs::PoseStamped::ConstPtr& map_pose)
  {
    int scan_points, row_pix, column_pix, scan_num;
    float ratio, angle_increment, angle_min, angle_max;
    sensor_msgs::LaserScan::ConstPtr scan_ptr;
    geometry_msgs::PoseStamped::ConstPtr scan_pose, last_scan_pose; 
    geometry_msgs::PoseStamped intermediate;

    cv::Matx33f Tnb = cv::Matx33f(0,0,0, 0,0,0, 0,0,0), Tbn = cv::Matx33f(0,0,0, 0,0,0, 0,0,0);
    cv::Point3f p_odom, p_body, p_delta;

    scan_num = scan_msgs.size();

    for(int k = 0; k < scan_num; k++)
    {
      scan_ptr = scan_msgs.back();
      scan_pose = scan_poses.back();
      last_scan_pose = last_scan_poses.back();
      // pop the end.
      scan_msgs.pop_back();
      scan_poses.pop_back();
      last_scan_poses.pop_back();

      scan_points = scan_ptr->ranges.size(); // how many points?
      angle_increment = scan_ptr->angle_increment;
      angle_min = scan_ptr->angle_min;

      for(int i = 0; i < scan_points; i++)
      {
        if(scan_ptr->ranges[i] > 0.5f*std::min(costmap_length_m, costmap_width_m)
           or scan_ptr->ranges[i] > max_scan_range
           or scan_ptr->ranges[i] < 0.2f)
        {
          continue;
        }
        interpolate(last_scan_pose, scan_pose, float(scan_points - i)/float(scan_points), intermediate);
        calc_Transform_ll(intermediate.pose.orientation, Tnb, Tbn); // calling the lower level tf function because I can't modify const pointers
        p_body = polar_to_cartesian_lidar(angle_min + angle_increment*i, scan_ptr->ranges[i]);
        
        p_delta.x = map_pose->pose.position.x - intermediate.pose.position.x;
        p_delta.y = map_pose->pose.position.y - intermediate.pose.position.y;
        p_delta.z = map_pose->pose.position.z - intermediate.pose.position.z;

        p_odom = Tbn * p_body - p_delta;
        
        row_pix = int((p_odom.y + costmap_length_m*0.5) / resolution_m);
        column_pix = int((-p_odom.x + costmap_width_m*0.5) / resolution_m);
        map.at<float>(row_pix, column_pix) = p_odom.z + map_pose->pose.position.z; // add odom ref height
        confidence_map.at<float>(row_pix, column_pix) = 1;
      }

    }
  }

  void process()
  {
    if(!cam_init or !pose_init or !depth_init)
    {
      // ROS_INFO("cam_init: %d, pose_init: %d, depth_init: %d, scan_init: %d", int(cam_init), int(pose_init), int(depth_init), int(scan_init));
      return;
    }
    if(depth_image_ptrs.empty() and scan_msgs.empty())
    {
      return;
    }

    ros::Time begin = ros::Time::now();
    geometry_msgs::PoseStamped::ConstPtr map_pose = cur_pose;
    transform_map(map_pose);

    int depth_image_num = depth_image_ptrs.size();
    ROS_INFO("num_images:%d", depth_image_num);
    // if(depth_image_num > 0)
    // {
    //   process_depth(map_pose);
    // }

    int scan_num = scan_msgs.size();
    ROS_INFO("num_scans:%d", scan_num);

    if(scan_num > 0)
    {
      process_scan(map_pose);
    }

    cv::Mat temp_grad;
    cv::Sobel(map, temp_grad, CV_32F, 1, 1, 3, 1, 0, cv::BORDER_DEFAULT);

    cv::GaussianBlur(temp_grad, gradmap, cv::Size(5,5), 0, 0, cv::BORDER_DEFAULT);

    float delta_time = (ros::Time::now() - begin).toSec();

    new_map = true; // set this to true so that the controller can do it's job
    ROS_INFO("delta: %f ms", delta_time*1e3);
    // cv::Mat display;
    // cv::flip(map, display, -1);
    // cv::namedWindow("map", cv::WINDOW_NORMAL);
    // cv::resizeWindow("map", 800, 800);
    // cv::imshow("map", display);
    // cv::waitKey(3);

  }

  void mapping_run()
  {
    ros::Rate r(map_update_rate);
    while(ros::ok())
    {
      process();
      r.sleep();
    }
  }

  // -------------------- planner stuff -----------------------------

  void path_cb(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    path.clear(); // remove last points
    cv::Point3f temp;
    int num = msg->poses.size();  // how many points?
    for(int i = 0; i < num; i++)
    {
      temp.x = msg->poses[i].position.x;
      temp.y = msg->poses[i].position.y;
      temp.z = msg->poses[i].position.z;

      path.push_back(temp);
    }
    path_received = true;
  }

  cv::Point3f rotate_point(cv::Point2f point, float angle)
  {
    cv::Point3f out;
    out.x = point.x * cosf(angle) - point.y * sinf(angle);
    out.y = point.x * sinf(angle) + point.y * cosf(angle);
    return out;
  }

  void pub_ctrl(float steering, float speed)
  {
      mavros_msgs::ManualControl manual_control_msg;

      float th = speed / 14.28;
      th = std::min(std::max(th,0.0f),3.0f);
      float st = steering/0.488;
      st = std::min(std::max(st,-1.0f),1.0f);

      manual_control_msg.header.stamp = ros::Time::now();
      manual_control_msg.x = 1000;
      manual_control_msg.y = st*1000; // steering;
      manual_control_msg.z = th*1000; // throttle;
      manual_control_msg.r = 1000;
      manual_control_msg.buttons = 0;

      control_pub.publish(manual_control_msg);
  }

  void planner()
  {
    if(!path_received or !pose_init or !new_map)
    {
      return;
    }
    ros::Time start = ros::Time::now();
    new_map = false; // reset new map to false;

    geometry_msgs::PoseStamped::ConstPtr map_pose = last_map_pose;

    cv::Point3f car_pos, goal_pos;
    car_pos.x = cur_pose->pose.position.x - map_pose->pose.position.x;
    car_pos.y = cur_pose->pose.position.y - map_pose->pose.position.y;
    float rpy[3];
    rpy_from_quat(rpy,cur_pose);
    car_pos.z = rpy[2];
    cv::Point2f temp;
    cv::Point3f goal;
    bool goal_found;
    int path_size = path.size();
    int row_pix, column_pix;
    cv::Mat display_map = map.clone();

    for(int i = 0; i < path_size; i++)
    {
      temp.x = path[i].x - map_pose->pose.position.x;
      temp.y = path[i].y - map_pose->pose.position.y;

      if((cv::norm(temp) > lookahead_distance and !goal_found) or i == path_size -1)
      {
        goal.x = temp.x;
        goal.y = temp.y;
        goal.z = path[i].z;
        goal_found = true;
      }
      if(visualize_path)
      {
        row_pix = int((temp.y + costmap_length_m*0.5) / resolution_m);
        column_pix = int((-temp.x + costmap_width_m*0.5) / resolution_m);
        cv::circle(display_map, cv::Point(column_pix, row_pix), 0.2/resolution_m, cv::Scalar(1.0), -1);
      }
    }

    ros::Time start_time = ros::Time::now();
    cv::Point3f end(0,0,goal.z); // start point is actually a tad bit behind
    cv::Point2f track, normal, tangent;
    // generate trajectory coefficients.
    traj.clear();
    int num_index = (step_x * step_y) + 1;
    float cost[num_index], lowest_cost = 100000;
    int lowest_index;
    float arc_length_inv;
    int index;
    float delta_curvature, delta_length, hit_cost;
    float step_size;
    float track_left = -car_width/2, track_right = car_width/2, track_res = car_width*0.1f;
    float car_height = map.at<float>(int(costmap_height_p*0.5),int(costmap_width_p*0.5));


    for(int i = 0; i < step_x; i++)
    {
      temp.x = (i+1)*resol_x;
      for(int j = 0; j < step_y; j++)
      {
        index = step_y*i + j;
        temp.y = (-width_y + j*resol_y)*float(i+1)/float(step_x);
        end = rotate_point(temp, goal.z);
        if(i < step_x/2)
        {
          end.z = goal.z - (1.0f - j*(2.0f/step_y));
        }
        else
        {
          end.z = goal.z;
        } 
        Bezier curve(car_pos,end);
        
        cost[index] = 0;
        step_size = curve.step_size(resolution_m);
        for(float k = 0; k < 1; k += step_size)
        {
          track = curve.curve(k);
          normal = curve.normal(k);
          tangent = curve.tangent(k);
          arc_length_inv = 1.0f/curve.arc_length;
          for(float l= track_left ; l <= track_right; l+= track_res)
          {
            row_pix = (track.y + l * normal.y + car_length_full * tangent.y + 0.5f * costmap_length_m)/resolution_m;
            column_pix = (-(track.x + l * normal.x + car_length_full * tangent.x) + 0.5f * costmap_width_m) /resolution_m;
            hit_cost = (map.at<float>(int(row_pix), int(column_pix)) - car_height);  // double it because half the car's height range = 100% probability of hit
            // hit_cost += gradmap.at<float>(int(row_pix), int(column_pix))/resolution_m;
            // hit_cost /= std::max(confidence_map.at<float>(int(row_pix), int(column_pix)), 0.01f);
            // cost[index] += hit_cost * hit_cost * resolution_m;
          }
        }
        cost[index] += goal_cost_multiplier * curve.distance_to_goal(goal);
        delta_curvature = curve.C[0] - last_curvature;
        delta_length = cv::norm(curve.P[0] - curve.P[3]) - last_length;
        cost[index] += delta_curvature_cost * delta_curvature * delta_curvature;
        cost[index] += delta_length_cost * delta_length * delta_length;

        if(cost[index] < lowest_cost)
        {
          lowest_index = index;
          lowest_cost = cost[index];
        }
        traj.push_back(curve);
      }
    }

    last_curvature = traj[lowest_index].C[0];
    last_length = cv::norm(traj[lowest_index].P[3] - traj[lowest_index].P[0]);

    float speed, steering, curvature, future_dist;
    future_dist = std::min(2*speed_meas, 0.1f*last_length);
    step_size = traj[lowest_index].step_size(future_dist);
    step_size = std::max(step_size, 0.02f);
    
    curvature = traj[lowest_index].C_from_t(step_size);
    
    float speed_curve = std::max(0.001f, float(fabs(last_curvature)));
    speed = std::min(sqrtf(last_length), 1/speed_curve);
    speed = std::min(speed, 5.0f);
    
    if(last_length < car_length*1.5)
    {
      steering = 0;
      speed = 0;
    }
    else
    {
      steering = atanf(car_length * curvature);
    }
    steering = std::min(std::max(-0.3f, steering), 0.3f);
    steering = 0.5*steering + 0.5*last_steering;
    float delta_steering = std::max(std::min((steering - last_steering), 0.02f), -0.02f);
    steering = last_steering + delta_steering;
    speed = 0.5*speed + 0.5*last_speed;

    last_steering = steering;
    last_speed = speed;

    if(visualize_path)
    {
      step_size = traj[lowest_index].step_size(resolution_m);
      for(float t = 0; t<= 1.0; t += step_size)
      {
        track = traj[lowest_index].curve(t);
        normal = traj[lowest_index].normal(t);

        for(float l= - car_width/2; l<= car_width/2; l += car_width * resolution_m)
        { 
          row_pix = (track.y + l * normal.y + car_length_full * tangent.y + 0.5f * costmap_length_m)/resolution_m;
          column_pix = (-(track.x + l * normal.x + car_length_full * tangent.x) + 0.5f * costmap_width_m) /resolution_m;
          display_map.at<float>(int(row_pix), int(column_pix)) = 1.0f;
        }
      }
    }
    float delta = (ros::Time::now() - start).toSec();
    ROS_INFO("delta_MPC: %f", delta*1e3);
    // cv::Mat display;
    // cv::flip(confidence_map, display, -1);

    // cv::imshow("map", display);
    // cv::waitKey(3);
    pub_ctrl(steering, speed);

  }  

  void planner_run()
  {
    ros::Rate r(map_update_rate*100);
    while(ros::ok())
    {
      planner();
      r.sleep();
    }
  }
};


int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "hound_core");
  ros::NodeHandle nh("~");
  hound_core hc(nh);
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
