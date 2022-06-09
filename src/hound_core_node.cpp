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

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
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


class depth_to_costmap
{
public:
  ros::Subscriber sub_depth, sub_info, sub_pose, sub_bound, sub_ctrl, sub_path;
  ros::Publisher control_publisher;
  float fx_inv, fy_inv, fx, fy;
  float cx, cy;
  int height, width;
  bool cam_init;

  float max_height;
  float min_height;
  float height_range_inv;

  float costmap_length_m;
  float costmap_width_m;
  float resolution_m;
  int costmap_height_p, costmap_width_p;
  float cam_pitch;

  int filt_size;
  float pose[3], last_pose[3];
  bool pose_init;

  std::vector<cv::Point2f> bound;
  bool bound_received;
  float bound_update_time, bound_time_constant;
  ros::Time last_bound_time;

  cv::Mat map, gradmap, bound_map, final_map;

  bool new_map, path_received, visualize_path;
  std::vector<cv::Point3f> path;
  float lookahead_distance;
  std::vector<Bezier> traj;
  float goal_cost_multiplier, car_width, car_length, car_length_full;
  int step_x, step_y;
  float width_y, length_x, resol_x, resol_y;
  float last_curvature, last_length;
  float delta_curvature_cost, delta_length_cost;
  float last_steering, last_speed, speed_meas;

  bool use_cam_info_preset;

  depth_to_costmap(ros::NodeHandle &nh) // constructor
  {
    cam_init = false;
    pose_init = false;
    bound_received = false;
    path_received = false;
    sub_depth = nh.subscribe("image", 1, &depth_to_costmap::image_cb, this);
    sub_info = nh.subscribe("camera_info", 10, &depth_to_costmap::info_cb, this);
    sub_pose = nh.subscribe("pose", 1, &depth_to_costmap::pose_cb, this);
    sub_bound = nh.subscribe("bound", 1, &depth_to_costmap::bound_cb, this);
    sub_ctrl = nh.subscribe("pose", 1, &depth_to_costmap::control_cb, this);
    sub_path = nh.subscribe("path", 1, &depth_to_costmap::path_cb, this);
    control_publisher = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/depth/control",10);

    if(not nh.getParam("depth/cam_pitch",cam_pitch))
    {
      cam_pitch = 0;
    }
    if(not nh.getParam("depth/max_height", max_height))
    {
      max_height = 0.5;
    }
    if(not nh.getParam("depth/min_height", min_height))
    {
      min_height = -1.3;
    }
    if(not nh.getParam("depth/costmap_length_m", costmap_length_m))
    {
      costmap_length_m = 20;
    }
    if(not nh.getParam("depth/costmap_width_m", costmap_width_m))
    {
      costmap_width_m = 10;
    }
    if(not nh.getParam("depth/resolution_m", resolution_m))
    {
      resolution_m = 0.1;
    }
    if(not nh.getParam("depth/filt_size", filt_size))
    {
      filt_size = 3;
    }
    if(not nh.getParam("depth/bound_time_constant", bound_time_constant))
    {
      bound_time_constant = 0.5;
    }
    if(not nh.getParam("depth/visualize_path", visualize_path))
    {
      visualize_path = false;
    }
    if(not nh.getParam("depth/lookahead_distance", lookahead_distance))
    {
      lookahead_distance = 10;
    }
    if(not nh.getParam("depth/car_width", car_width))
    {
      car_width = 2;
    }
    if(not nh.getParam("depth/car_length", car_length))
    {
      car_length = 2;
    }
    if(not nh.getParam("depth/goal_cost_multiplier", goal_cost_multiplier))
    {
      goal_cost_multiplier = 5;
    }
    if(not nh.getParam("depth/length_x", length_x))
    {
      length_x = 15;
    }
    if(not nh.getParam("depth/width_y", width_y))
    {
      width_y = 3;
    }
    if(not nh.getParam("depth/step_x", step_x))
    {
      step_x = 5;
    }
    if(not nh.getParam("depth/step_y", step_y))
    {
      step_y = 5;
    }
    if(not nh.getParam("depth/delta_curvature_cost", delta_curvature_cost))
    {
      delta_curvature_cost = 10;
    }
    if(not nh.getParam("depth/delta_length_cost", delta_length_cost))
    {
      delta_length_cost = 10;
    }
    if(not nh.getParam("depth/use_cam_info_preset", use_cam_info_preset))
    {
      use_cam_info_preset = false;
    }
    if(not nh.getParam("depth/car_length_full", car_length_full))
    {
      car_length_full = 3.0f;
    }
    if(use_cam_info_preset)
    {
      fx = 548.83068f;
      fy = 548.83068f;
      fx_inv = 1.0f/fx;
      fy_inv = 1.0f/fy;
      cx = 638.52880f;
      cy = 365.00677f;
      height = 720;
      width = 1280;
      cam_init = true;
    }
    costmap_height_p = costmap_length_m/resolution_m;
    costmap_width_p  = costmap_width_m/resolution_m;
    height_range_inv = max_height - min_height;

    map = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    bound_map = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    gradmap = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);

    resol_x = length_x/float(step_x);
    resol_y = 2*width_y/float(step_y);

    last_length = 0;
    last_curvature = 0;
    last_speed = 0;
    last_steering = 0;
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

  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    float rpy[3];
    pose[0] = msg->pose.position.x;
    pose[1] = msg->pose.position.y;
    rpy_from_quat(rpy,msg);
    pose[2] = rpy[2];
    if(not pose_init)
    {
      pose_init = true;
      last_pose[0] = pose[0];
      last_pose[1] = pose[1];
      last_pose[2] = pose[2];
    }
  }

  void bound_cb(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    bound_update_time = (ros::Time::now() - last_bound_time).toSec();
    last_bound_time = ros::Time::now();
    cv::Point2f temp;  // temp to hold values
    bound.clear(); // remove last points
    int num = msg->poses.size();  // how many points?
    for(int i = 0; i < num; i++)
    {
      temp.x = msg->poses[i].position.x;
      temp.y = msg->poses[i].position.y;
      bound.push_back(temp);
    }
    bound_received = true;
  }

  void info_cb(const sensor_msgs::CameraInfoConstPtr& info_msg)
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

  void image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    if(!cam_init or !pose_init)
    {
      return;
    }
    ros::Time begin = ros::Time::now();
    bool float_flag = false;
    float depth, x, y, z, temp_x, temp_z;
    int row_pix, column_pix;

    cv_bridge::CvImagePtr cv_ptr;

    // find delta pose in odom frame
    float delta_x = pose[0] - last_pose[0];
    float delta_y = pose[1] - last_pose[1];
    float delta_t = pose[2] - last_pose[2];
    float delta[3], ct = cosf(-pose[2]), st = sinf(-pose[2]);  // backward rotation
    //  find delta pose in body frame
    delta[0] = delta_x*ct - delta_y*st;
    delta[1] = delta_x*st + delta_y*ct;
    delta[2] = delta_t;
    
    speed_meas = delta[0];

    //  set the last known pose
    last_pose[0] = pose[0];
    last_pose[1] = pose[1];
    last_pose[2] = pose[2];
    
    //  define rotation center as camera's location in map image
    cv::Point2f center(costmap_width_p*0.5, 0);
    cv::Mat rotated_image, translated_image;  // temporary images
    float warp_values[] = { 1.0, 0.0, -delta[1]/resolution_m, 0.0, 1.0, -delta[0]/resolution_m };  // translation matrix
    cv::Mat translation_matrix = cv::Mat(2, 3, CV_32F, warp_values);    // fill in the values
    cv::Mat rotation_matix = getRotationMatrix2D(center, -delta[2]*57.3, 1.0);  // rotate matrix around camera. angle in degrees    
    //  in forward state propagation, we rotate, then translate. Therefore, in backward transform, we translate, then rotate
    cv::warpAffine(map, translated_image, translation_matrix, map.size());  // then we translate
    cv::warpAffine(translated_image, rotated_image, rotation_matix, map.size());  // first we rotate
    map = rotated_image.clone();

    // this is for the bound layer.
    cv::warpAffine(bound_map, rotated_image, rotation_matix, map.size());
    cv::warpAffine(rotated_image, translated_image, translation_matrix, map.size());
    bound_map = translated_image.clone();

    try
    {

      cv::Mat grad_x, grad_y, smoothened;
      if(msg->encoding == enc::TYPE_16UC1 || msg->encoding == enc::MONO16)
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        float_flag = false;
      }
      else if(msg->encoding == enc::TYPE_32FC1)
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        float_flag = true;
      }
      // u is width, v is height. for opencv "at", row comes first(so height, width or y,x and not x,y)
      for(int u = 0; u < width; u++)
      {
        for(int v = 0; v < height*0.8; v++)
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
          y = (cx - float(u))*x*fx_inv;

          temp_x = x;
          temp_z = z;

          x = temp_x*cosf(cam_pitch) - temp_z*sinf(cam_pitch);
          z = temp_x*sinf(cam_pitch) + temp_z*cosf(cam_pitch);

          if(z < max_height and z > min_height and x < costmap_length_m and fabs(y) < costmap_width_m*0.5 )
          {
            row_pix = int(x / resolution_m);
            column_pix = int((y + costmap_width_m*0.5) / resolution_m);
            map.at<float>(row_pix, column_pix) = std::min((z - min_height)*height_range_inv,1.0f);  // normalize using height range.
          } 
        }
      }
      cv::Sobel(map, gradmap, CV_32F, 1, 1, 3, 1, 0, cv::BORDER_DEFAULT);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    if(bound_received)
    {
      bound_received = false;
      cv::Point2f dum, temp;
      float freq = 1/bound_update_time;
      float bound_freq = 1/bound_time_constant;
      float coeff = powf(0.5f, bound_freq/freq);
      bound_map *= coeff;// halve the value when new information is made available
      for(int i = 0; i < bound.size(); i++)
      {
        dum.x = bound[i].x - pose[0];
        dum.y = bound[i].y - pose[1];
        temp.x = ct*dum.x - st*dum.y;  // interchange the x and y
        temp.y = st*dum.x + ct*dum.y;
        row_pix = temp.x/resolution_m;
        column_pix = (temp.y + 0.5*costmap_width_m)/resolution_m;
        cv::circle(bound_map, cv::Point(column_pix, row_pix), 0.4/resolution_m, cv::Scalar(1.0), -1);
      }
    }

    final_map = map + bound_map; // + gradmap*delta[0]/resolution_m;
    new_map = true;

    cv::Mat gradmap_new;
    cv::GaussianBlur(gradmap, gradmap_new, cv::Size(filt_size, filt_size), 0, 0, cv::BORDER_DEFAULT);
    cv::Mat display;
    cv::flip(final_map, display, -1);
    cv::Mat grad_disp;
    cv::flip(gradmap_new, grad_disp, -1);

    float delta_time = (ros::Time::now() - begin).toSec();
    // ROS_INFO("%f", delta_time*1000);
    // cv::imshow("gradmap", grad_disp);
    // cv::imshow("map", display);
    // cv::imshow("test", cv_ptr->image);
    // cv::waitKey(3);

  }

  // -------------------------- control systems stuff------------------------------

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

  void control_pub(float steer, float speed)
  {
    ackermann_msgs::AckermannDriveStamped output_msg;
    output_msg.header.stamp = ros::Time::now();
    output_msg.header.frame_id = "";
    output_msg.drive.steering_angle = steer;
    output_msg.drive.speed = speed;
    control_publisher.publish(output_msg);
  }

  void control_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    float rpy[3], control_pose[3];
    control_pose[0] = msg->pose.position.x;
    control_pose[1] = msg->pose.position.y;
    rpy_from_quat(rpy,msg);
    control_pose[2] = rpy[2];
    if(not new_map)
    {
      return;
    }
    // new_map = false;
    if(not path_received)
    {
      return;
    }

    float delta_x = control_pose[0] - last_pose[0];
    float delta_y = control_pose[1] - last_pose[1];
    float delta_t = control_pose[2] - last_pose[2];
    float delta[3], ct = cosf(-last_pose[2]), st = sinf(-last_pose[2]);  // backward rotation
    //  find delta pose in body frame
    delta[0] = delta_x*ct - delta_y*st;
    delta[1] = delta_x*st + delta_y*ct;
    delta[2] = delta_t;

    // super impose the path on to the costmap (this is for visualization only)
    cv::Point2f dum, temp;
    cv::Point3f goal;
    bool goal_found;
    float row_pix, column_pix;
    int path_size = path.size();
    for(int i = 0; i < path_size; i++)
    {
      dum.x = path[i].x - last_pose[0];
      dum.y = path[i].y - last_pose[1];
      temp.x = ct*dum.x - st*dum.y;  // interchange the x and y
      temp.y = st*dum.x + ct*dum.y;
      if((temp.x > lookahead_distance and !goal_found) or i == path_size -1)
      {
        goal.x = temp.x;
        goal.y = temp.y;
        goal.z = path[i].z - last_pose[2];  // take offset for heading
        goal_found = true;
      }
    }

    ros::Time start_time = ros::Time::now();
    cv::Point3f start(delta[0] - car_length*cosf(delta[2]), delta[1] - car_length*sinf(delta[2]), delta[2]), end(0,0,goal.z); // start point is actually a tad bit behind
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

    for(int i = 0; i < step_x; i++)
    {
      end.x = (i+1)*resol_x;
      for(int j = 0; j < step_y; j++)
      {
        index = step_y*i + j;
        end.y = (-width_y + j*resol_y)*float(i+1)/float(step_x);
        if(end.x - start.x < car_length_full*2.0f)
        {
          end.z = -(0.2f - j*(0.4f/step_y));
        }
        else
        {
          end.z = goal.z;
        }
        Bezier curve(start,end);
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
            row_pix = (track.x + l * normal.x + car_length_full * tangent.x ) /resolution_m;
            row_pix = std::max(std::min(row_pix, float(costmap_height_p)), 0.0f);
            column_pix = (track.y + l * normal.y + car_length_full * tangent.y + 0.5 * costmap_width_m)/resolution_m;
            column_pix = std::max(std::min(column_pix, float(costmap_width_p)), 0.0f);
            hit_cost = 2 * final_map.at<float>(int(row_pix), int(column_pix));  // double it because half the car's height range = 100% probability of hit
            cost[index] += hit_cost * hit_cost * resolution_m;
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
    control_pub(steering, speed);
    // ------------------------------- visualisation only ------------------------
    cv::Mat display_map = final_map.clone();
    if(visualize_path)
    {
      for(float t = 0; t<= 1.0; t += 0.01)
      {
        track = traj[lowest_index].curve(t);
        row_pix = (track.x) /resolution_m;
        row_pix = std::max(std::min(row_pix, float(costmap_height_p)), 0.0f);
        column_pix = (track.y + 0.5 * costmap_width_m)/resolution_m;
        column_pix = std::max(std::min(column_pix, float(costmap_width_p)), 0.0f);
        display_map.at<float>(int(row_pix), int(column_pix)) = 1.0f;

        normal = traj[lowest_index].normal(t);
        for(float l= - car_width/2; l<= car_width/2; l += car_width * resolution_m)
        { 
          row_pix = (track.x + l * normal.x + car_length_full * tangent.x) /resolution_m;
          row_pix = std::max(std::min(row_pix, float(costmap_height_p)), 0.0f);
          column_pix = (track.y + l * normal.y + car_length_full * tangent.y + 0.5 * costmap_width_m)/resolution_m;
          column_pix = std::max(std::min(column_pix, float(costmap_width_p)), 0.0f);
          display_map.at<float>(int(row_pix), int(column_pix)) = 1.0f;
        }
      }
      for(int i = 0; i < path.size(); i++)
      {
        dum.x = path[i].x - last_pose[0];
        dum.y = path[i].y - last_pose[1];
        temp.x = ct*dum.x - st*dum.y;  // interchange the x and y
        temp.y = st*dum.x + ct*dum.y;
        row_pix = temp.x/resolution_m;
        column_pix = (temp.y + 0.5*costmap_width_m)/resolution_m;
        cv::circle(display_map, cv::Point(column_pix, row_pix), 0.2/resolution_m, cv::Scalar(1.0), -1);
      }
    }

    cv::Mat display;
    cv::flip(display_map, display, -1);
    cv::imshow("map", display);
    // cv::imshow("test", cv_ptr->image);
    cv::waitKey(3);
    ROS_INFO("speed: %f, steering: %f,processing time: %f", speed, steering * 57.3f, (ros::Time::now() - start_time).toSec() );

  }

};
int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "cv_example");
  ros::NodeHandle nh("~");
  depth_to_costmap d2c(nh);
  // subsribe topic
  ros::MultiThreadedSpinner spinner(16); // Use one thread per core
  spinner.spin(); // spin() will not return until the node has been shutdown

  return 0;
}