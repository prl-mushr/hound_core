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
#include <tf/tf.h>
namespace enc = sensor_msgs::image_encodings;

class hound_core
{
public:
  ros::Subscriber sub_depth_image, sub_cam_info, sub_pose, sub_scan;
  ros::Publisher control_publisher;
  float fx_inv, fy_inv, fx, fy;
  float cx, cy;
  int height, width;
  float max_depth;
  bool cam_init;

  float max_height;
  float min_height;
  float height_range_inv;

  float costmap_length_m;
  float costmap_width_m;
  float resolution_m;
  int costmap_height_p, costmap_width_p;
  float cam_pitch;
  float cp, sp;

  bool float_flag, depth_init, scan_init;

  geometry_msgs::PoseStamped::ConstPtr cur_pose;
  bool pose_init;

  std::vector<geometry_msgs::PoseStamped::ConstPtr> depth_poses;
  std::vector<cv_bridge::CvImagePtr> depth_image_ptrs;

  std::vector<geometry_msgs::PoseStamped::ConstPtr> scan_poses;
  std::vector<sensor_msgs::LaserScan::ConstPtr> scan_msgs;

  cv::Mat map, gradmap, bound_map, final_map;
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

    if(not nh.getParam("hound/cam_pitch",cam_pitch))
    {
      cam_pitch = 0;
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
    costmap_height_p = costmap_length_m/resolution_m;
    costmap_width_p  = costmap_width_m/resolution_m;

    map = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    bound_map = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    gradmap = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    cp = cosf(cam_pitch);
    sp = sinf(cam_pitch);
    height_range_inv = 1/(max_height - min_height);
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
    float q00,q11,q22,q33,q01,q02,q03,q12,q13,q23;
    cv::Mat Tnb = cv::Mat::zeros(3,3, CV_32FC1);
    cv::Mat Tbn = Tnb.clone();
    geometry_msgs::Quaternion q1 = msg->pose.orientation;
    tf::Quaternion q;
    q.setW(q1.w);
    q.setX(q1.x);
    q.setY(q1.y);
    q.setZ(q1.z);
    // float qx, qy,qz,qw;
    // qx = q.w;
    // qy = q.x;
    // qz = q.y;
    // qw = q.z;

    // q00 = qx*qx;
    // q11 = qy*qy;
    // q22 = qz*qz;
    // q33 = qw*qw;
    // q01 =  qx*qy;
    // q02 =  qx*qz;
    // q03 =  qx*qw;
    // q12 =  qy*qz;
    // q13 =  qy*qw;
    // q23 =  qz*qw;
    
    // Tbn.at<float>(0,0) = q00 + q11 - q22 - q33;
    // Tbn.at<float>(1,1) = q00 - q11 + q22 - q33;
    // Tbn.at<float>(2,2) = q00 - q11 - q22 + q33;
    // Tbn.at<float>(0,1) = 2*(q12 - q03);
    // Tbn.at<float>(0,2) = 2*(q13 + q02);
    // Tbn.at<float>(1,0) = 2*(q12 + q03);
    // Tbn.at<float>(1,2) = 2*(q23 - q01);
    // Tbn.at<float>(2,0) = 2*(q13 - q02);
    // Tbn.at<float>(2,1) = 2*(q23 + q01);
    tf::Matrix3x3 m(q);
    Tbn = m;
    Tnb = Tbn.t(); // transform ned->body

    TBN = cv::Matx33f(Tbn);
    TNB = cv::Matx33f(Tnb);
  }

  void depth_image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
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
    scan_msgs.insert(scan_msgs.begin(), msg);
    scan_poses.insert(scan_poses.begin(), cur_pose);
    if(!scan_init)
    {
      scan_init = true;
    }
    return;
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
    int row_pix, column_pix;

    float delta[2];
    
    delta[0] = cur_pose->pose.position.x - last_map_pose->pose.position.x;
    delta[1] = cur_pose->pose.position.y - last_map_pose->pose.position.y;

    last_map_pose = cur_pose;
    
    //  define translation center as camera's location in map image
    cv::Point2f center(costmap_width_p*0.5, costmap_height_p*0.5);
    cv::Mat translated_image;  // temporary images
    float warp_values[] = { 1.0, 0.0, delta[0]/resolution_m, 0.0, 1.0, -delta[1]/resolution_m };  // translation matrix
    cv::Mat translation_matrix = cv::Mat(2, 3, CV_32F, warp_values);    // fill in the values
    
    cv::warpAffine(map, translated_image, translation_matrix, map.size());  // then we translate
    map = translated_image.clone();

      // u is width, v is height. for opencv "at", row comes first(so height, width or y,x and not x,y)
    int depth_image_num = depth_image_ptrs.size();
    if(depth_image_num>0)
    {
      cv_bridge::CvImagePtr cv_ptr;
      geometry_msgs::PoseStamped::ConstPtr img_pose;
      cv::Matx33f Tnb = cv::Matx33f(0,0,0, 0,0,0, 0,0,0), Tbn = cv::Matx33f(0,0,0, 0,0,0, 0,0,0);
      cv::Point3f p_odom, p_body, p_delta;
      float x,z, temp_x, temp_z;
      float depth;

      for(int k = 0; k < 1; k++)
      {
        cv_ptr = depth_image_ptrs.back();
        img_pose = depth_poses.back();
        // pop the end.
        depth_image_ptrs.pop_back();
        depth_poses.pop_back();

        calc_Transform(img_pose, Tnb, Tbn);
        p_delta.x = cur_pose->pose.position.x - img_pose->pose.position.x;
        p_delta.y = cur_pose->pose.position.y - img_pose->pose.position.y;
        p_delta.z = cur_pose->pose.position.z - img_pose->pose.position.z;

        for(int u = 0; u < width; u++)
        {
          for(int v = int(height*0.5); v < height; v++)
          {
            if(float_flag)
            {
              depth = cv_ptr->image.at<float>(v,u);
            }
            else
            {
              depth = float(cv_ptr->image.at<u_int16_t>(v,u))*1e-3;
            }
            x = depth;
            z = (cy - float(v))*x*fy_inv;
            p_body.y = (cx - float(u))*x*fx_inv;

            temp_x = x;
            temp_z = z;
            // find points in body frame-- this is where you should add the body frame offsets
            p_body.x = temp_x*cp - temp_z*sp;
            p_body.z = temp_x*sp + temp_z*cp;

            p_odom = Tbn * p_body + p_delta;
            if( (p_odom.z < max_height) and (p_odom.z > min_height) and fabs(p_odom.x) < costmap_width_m*0.5f and fabs(p_odom.y) < costmap_length_m*0.5 )
            {
              row_pix = int((p_odom.y + costmap_length_m*0.5) / resolution_m);
              column_pix = int((p_odom.x + costmap_width_m*0.5) / resolution_m);
              map.at<float>(row_pix, column_pix) = std::min((p_odom.z - min_height)*height_range_inv,1.0f);
            }
          }
        }
      }
    }
    float delta_time = (ros::Time::now() - begin).toSec();

    cv::Mat display;
    cv::flip(map, display, -1);

    ROS_INFO("%f", delta_time*1000);
    // cv::imshow("gradmap", grad_disp);
    cv::imshow("map", display);
    // cv::imshow("test", cv_ptr->image);
    cv::waitKey(3);
  }

};
int main(int argc, char **argv)
{
  //initialize node

  ros::init(argc, argv, "hound_core");
  ros::NodeHandle nh("~");
  hound_core hc(nh);
  // subsribe topic
  while(ros::ok())
  {
    hc.process();
    ros::spinOnce();
  }

  return 0;
}