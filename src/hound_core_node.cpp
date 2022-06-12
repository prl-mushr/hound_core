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
#include <boost/thread/thread.hpp>

namespace enc = sensor_msgs::image_encodings;

class hound_core
{
public:
  ros::Subscriber sub_depth_image, sub_cam_info, sub_pose, sub_scan;
  ros::Publisher control_publisher;
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

  cv::Mat map, gradmap, bound_map, final_map, confidence_map;
  geometry_msgs::PoseStamped::ConstPtr last_map_pose;


  hound_core(ros::NodeHandle &nh) // constructor
  {
    cam_init = false;
    pose_init = false;
    float_flag = false;
    depth_init = false;
    scan_init = false;

    sub_depth_image = nh.subscribe("image", 1, &hound_core::depth_image_cb, this);
    sub_cam_info = nh.subscribe("camera_info", 10, &hound_core::cam_info_cb, this);
    sub_pose = nh.subscribe("pose", 1, &hound_core::pose_cb, this);
    sub_scan = nh.subscribe("scan", 1, &hound_core::scan_cb, this);
    // process_timer = nh.createTimer(ros::Duration(0.2), &hound_core::process, this);

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
      float_flag = true;
    }
    if(not nh.getParam("hound/max_depth", max_depth))
    {
      max_depth = 10.0f;
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
    costmap_height_p = costmap_length_m/resolution_m;
    costmap_width_p  = costmap_width_m/resolution_m;

    map = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    confidence_map = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    bound_map = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    gradmap = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    cp = cosf(cam_pitch);
    sp = sinf(cam_pitch);
    height_range_inv = 1/(max_height - min_height);

    boost::thread proc_thread(&hound_core::run, this);
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
    float warp_values[] = { 1.0, 0.0, delta[0]/resolution_m, 0.0, 1.0, -delta[1]/resolution_m };  // translation matrix
    translation_matrix = cv::Mat(2, 3, CV_32F, warp_values);    // fill in the values
    
    cv::warpAffine(map, translated_image, translation_matrix, map.size());  // translate
    map = translated_image.clone();

    cv::warpAffine(confidence_map, translated_image, translation_matrix, map.size());  // translate
    confidence_map = translated_image.clone();
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
          if(depth > max_depth)
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
          if( (p_body.z < max_height) and (p_body.z > min_height) and fabs(p_odom.x) < costmap_width_m*0.5f and fabs(p_odom.y) < costmap_length_m*0.5 )
          {
            row_pix = int((p_odom.y + costmap_length_m*0.5) / resolution_m);
            column_pix = int((-p_odom.x + costmap_width_m*0.5) / resolution_m);

            pixel_height_est = map.at<float>(row_pix, column_pix);
            est_err = (1 - confidence_map.at<float>(row_pix, column_pix));
            mes_err = depth_err_coeff*depth;
            tot_err_inv = 1/(est_err + mes_err);

            pixel_height_mes = (p_odom.z - min_height);

            map.at<float>(row_pix, column_pix) = (pixel_height_est * mes_err + pixel_height_mes * est_err ) * tot_err_inv;
            est_err *= mes_err*tot_err_inv;
            confidence_map.at<float>(row_pix, column_pix) = 1.0f - est_err;
          }
        }
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
        interpolate(last_scan_pose, scan_pose, float(i)/float(scan_points), intermediate);
        calc_Transform_ll(intermediate.pose.orientation, Tnb, Tbn); // calling the lower level tf function because I can't modify const pointers
        p_body = polar_to_cartesian_lidar(angle_min + angle_increment*i, scan_ptr->ranges[i]);
        
        p_delta.x = map_pose->pose.position.x - intermediate.pose.position.x;
        p_delta.y = map_pose->pose.position.y - intermediate.pose.position.y;
        p_delta.z = map_pose->pose.position.z - intermediate.pose.position.z;

        p_odom = Tbn * p_body - p_delta;
        
        row_pix = int((p_odom.y + costmap_length_m*0.5) / resolution_m);
        column_pix = int((-p_odom.x + costmap_width_m*0.5) / resolution_m);
        map.at<float>(row_pix, column_pix) = p_odom.z - min_height;
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
    if(depth_image_num > 0)
    {
      process_depth(map_pose);
    }

    int scan_num = scan_msgs.size();
    ROS_INFO("num_scans:%d", scan_num);

    if(scan_num > 0)
    {
      process_scan(map_pose);
    }

    cv::Sobel(map, gradmap, CV_32F, 1, 1, 3, 1, 0, cv::BORDER_DEFAULT);

    float delta_time = (ros::Time::now() - begin).toSec();

    cv::Mat display;
    cv::flip(gradmap, display, -1);

    ROS_INFO("%f", delta_time*1000);
    // cv::imshow("gradmap", grad_disp);
    cv::imshow("map", display);
    // // cv::imshow("test", cv_ptr->image);
    cv::waitKey(3);
  }

  void run()
  {
    ros::Rate r(map_update_rate);
    while(ros::ok())
    {
      process();
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