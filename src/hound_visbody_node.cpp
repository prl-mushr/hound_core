/*
  * OpenCV Example using ROS and CPP
  */

 // Include the ROS library
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>


using namespace message_filters;

class hound_visbody
{
public:
  ros::Subscriber sub_cam_info;
  message_filters::Subscriber<sensor_msgs::Image> infra_sub, depth_sub;
  ros::Publisher odom_publisher;

  typedef sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;
  typedef Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;

  float fx_inv, fy_inv, fx, fy;
  float cx, cy;
  int height, width;
  float max_depth, roi[4], cam_pos[3];
  bool cam_init, depth_init, float_flag;

  cv::Mat depth_image_curr, depth_image_prev, infra_image_curr, infra_image_prev;
  cv::Ptr<cv::FastFeatureDetector> fastDetector;
  std::vector<cv::KeyPoint> keypoints;
  std::vector<cv::Point2f> points1, points2;
  cv::TermCriteria criteria;
  cv::Ptr<cv::Feature2D> orb;
  cv::Ptr<cv::DescriptorMatcher> matcher;
  bool matching;
  Eigen::Vector3f trans_est;

  hound_visbody(ros::NodeHandle &nh)
  {
    infra_sub.subscribe(nh,"infra", 3);
    depth_sub.subscribe(nh,"depth", 3);
    sub_cam_info = nh.subscribe("camera_info", 10, &hound_visbody::cam_info_cb, this);
    float_flag = false;
    sync.reset(new Sync(MySyncPolicy(10), infra_sub, depth_sub));   
    sync->registerCallback(boost::bind(&hound_visbody::image_processor_cb, this, _1, _2));
    fastDetector = cv::FastFeatureDetector::create(20, true);
    criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.01);
    matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    orb = cv::ORB::create(100);
    matching = true;
    trans_est = Eigen::Vector3f(0);
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

  void pix_to_coord(cv::Mat depth_image, cv::Point2f pt_pix, Eigen::MatrixXf &xyz, int i)
  {
      float depth_pt = float(depth_image.at<u_int16_t>(int(pt_pix.x), int(pt_pix.y)))*1e-4;
      xyz(0,i) = depth_pt;
      xyz(1,i) = (cx - pt_pix.x)*depth_pt*fx_inv;
      xyz(2,i) = (cy - pt_pix.y)*depth_pt*fy_inv;
  }

  void LPF(Eigen::Vector3f measurement,Eigen::Vector3f &estimate)
  {
    estimate = 0.5*estimate + 0.5*measurement;
  }

  void image_processor_cb(const sensor_msgs::ImageConstPtr& clr, const sensor_msgs::ImageConstPtr& dpt)
  {
    if(not cam_init)
    {
      return;
    }
    cv_bridge::CvImagePtr cv_ptr_dpt, cv_ptr_clr;
    try
    {
      if(!float_flag)
      {
        cv_ptr_dpt = cv_bridge::toCvCopy(dpt, sensor_msgs::image_encodings::TYPE_16UC1);
      }
      else
      {
        cv_ptr_dpt = cv_bridge::toCvCopy(dpt, sensor_msgs::image_encodings::TYPE_32FC1);
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    try
    {
      cv_ptr_clr = cv_bridge::toCvCopy(dpt, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat depth = cv_ptr_dpt->image;
    cv::Mat infra = cv_ptr_clr->image;
    if(not depth_init)
    {
      depth_init = true;
      depth_image_curr = depth.clone();
      infra_image_curr = infra.clone();
      return;
    }

    depth_image_prev = depth_image_curr.clone();
    infra_image_prev = infra_image_curr.clone();
    depth_image_curr = depth.clone();
    infra_image_curr = infra.clone();


    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    int indexCorrection = 0;
    cv::Point2f pt;

    ros::Time start = ros::Time::now();  
    if(matching)
    {
      orb->detectAndCompute(infra_image_prev, cv::Mat(), keypoints1, descriptors1);

      orb->detectAndCompute(infra_image_curr, cv::Mat(), keypoints2, descriptors2);

      std::vector<cv::DMatch> matches;
      
      matcher->match(descriptors1, descriptors2, matches, cv::Mat());

      std::sort(matches.begin(), matches.end());
      const int numGoodMatches = matches.size() * 0.5f;
      matches.erase(matches.begin()+numGoodMatches, matches.end());

      points1.clear();
      points2.clear();
      for( size_t i = 0; i < matches.size(); i++ )
      {
        points1.push_back( keypoints1[ matches[i].queryIdx ].pt );
        points2.push_back( keypoints2[ matches[i].trainIdx ].pt );
      }
    }
    else
    {
      keypoints.clear();
      fastDetector->detect(infra_image_prev, keypoints);  //
      cv::KeyPointsFilter::retainBest(keypoints, 100);

      points1.clear();
      cv::KeyPoint::convert(keypoints, points1);
      std::vector<uchar> status;
      std::vector<float> err;

      points2.clear();
      cv::calcOpticalFlowPyrLK(infra_image_prev, infra_image_curr, points1, points2, status, err, cv::Size(15,15), 2, criteria, 0, 0.001);    
      indexCorrection = 0;
      for( int i=0; i<status.size(); i++)
      {  
        pt = points2.at(i- indexCorrection);

        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))  
        {
          if((pt.x<0)||(pt.y<0))  
          {
            status.at(i) = 0;
          }
          points1.erase (points1.begin() + (i - indexCorrection));
          points2.erase (points2.begin() + (i - indexCorrection));
          indexCorrection++;
        }
      }
      //backwards pass
      cv::calcOpticalFlowPyrLK(infra_image_curr, infra_image_prev, points2, points1, status, err, cv::Size(35,35), 3, criteria, 0, 0.001);    
      indexCorrection = 0;
      for( int i=0; i<status.size(); i++)
      {  
        pt = points1.at(i- indexCorrection);
        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))  
        {
          if((pt.x<0)||(pt.y<0))  
          {
            status.at(i) = 0;
          }
          points1.erase (points1.begin() + (i - indexCorrection));
          points2.erase (points2.begin() + (i - indexCorrection));
          indexCorrection++;
        }
      }
    }
    
    float depth_pt;
    indexCorrection = 0;
    int _num_inliers = points1.size();
    std::cout<<_num_inliers<<std::endl;
    for( int i=0; i< _num_inliers; i++)
    {  
      pt = points1.at(i- indexCorrection);
      depth_pt = float(depth_image_prev.at<u_int16_t>(int(pt.x), int(pt.y)))*1e-4 ;
      if (depth_pt > 4 )  
      {
        points1.erase (points1.begin() + (i - indexCorrection));
        points2.erase (points2.begin() + (i - indexCorrection));
        indexCorrection++;
      }
    }
    _num_inliers = points1.size();

    Eigen::MatrixXf target_xyz(3, _num_inliers);
    Eigen::MatrixXf ref_xyz(3, _num_inliers);
    cv::Point2f pt1_pix, pt2_pix;

    for(int i = 0; i < _num_inliers; i++)
    {
        pt1_pix = points1.at(i);
        pt2_pix = points2.at(i);

        pix_to_coord(depth_image_prev, pt1_pix, ref_xyz, i);
        pix_to_coord(depth_image_curr, pt2_pix, target_xyz, i);
    }

    Eigen::Matrix4f ume_estimate = Eigen::umeyama(target_xyz, ref_xyz);
    Eigen::Isometry3f motion_estimate = Eigen::Isometry3f(ume_estimate); 
    float delta = (ros::Time::now() - start).toSec();

    LPF(motion_estimate.translation(), trans_est);

    std::cout<<trans_est(0)<<" "<<trans_est(1)<<" "<<trans_est(2)<<std::endl; //<<result.at<float>(1,3) << result.at<float>(2,3);
    std::cout<<"delta: "<<delta*1e3<<" num_points = "<<_num_inliers<<std::endl;
  }
};

int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "cv_example");
  ros::NodeHandle nh("~");
  hound_visbody depth_odom(nh);
  // subsribe topic
  ros::MultiThreadedSpinner spinner(2); // Use one thread per core
  spinner.spin(); // spin() will not return until the node has been shutdown

  return 0;
}