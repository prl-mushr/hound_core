#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class PointCloudNode
{
public:
    ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub_;
    tf::TransformListener listener_;
    laser_geometry::LaserProjection projector_;

    PointCloudNode(ros::NodeHandle &nh_)
    {   
        // Subscribe to the laser scan topic
        ROS_INFO("initialized");
        scan_sub_ = nh_.subscribe("/scan", 1, &PointCloudNode::scanCallback, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("converted_pc2", 1);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        ROS_INFO("here!");
        if(!
          listener_.waitForTransform(
                "laser_frame",
                "map",
                scan_in->header.stamp - ros::Duration().fromSec(scan_in->ranges.size() * scan_in->time_increment),
                ros::Duration(1.0)))
        {
          return;
        }
        ROS_INFO("got transform");

        sensor_msgs::PointCloud2 cloud;
        projector_.transformLaserScanToPointCloud("map", *scan_in, cloud, listener_);
        cloud_pub_.publish(cloud);
    }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_node");
  ros::NodeHandle nh("~");
  PointCloudNode point_cloud_node(nh);
  ROS_INFO("node started");
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
