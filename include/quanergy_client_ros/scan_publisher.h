/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef QUANERGY_CLIENT_ROS_LASERSCAN_PUBLISHER_H
#define QUANERGY_CLIENT_ROS_LASERSCAN_PUBLISHER_H

#include <mutex>

#include <angles/angles.h>
#include <quanergy/common/point_hvdir.h>
#include <pcl/point_cloud.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/** \brief SimplePublisher publishes point clouds of type PointT */
class ScanPublisher
{
public:
  using Cloud = pcl::PointCloud<quanergy::PointHVDIR>;
  using CloudConstPtr = typename Cloud::ConstPtr;

  ScanPublisher(
    ros::NodeHandle& nh,
    const std::string& topic,
    float min_range,
    float max_range,
    double frame_rate,
    bool use_ros_time = true)
    : use_ros_time_(use_ros_time)
  {
    publisher_ = nh.advertise<sensor_msgs::LaserScan>(nh.resolveName(topic), 10);

    scan_.range_min = min_range;
    scan_.range_max = max_range;
    scan_.scan_time = 1.0f / static_cast<float>(frame_rate);
  }

  void slot(const CloudConstPtr& cloud)
  {
    if(!ros::ok() || !cloud || cloud->empty()) return;

    const auto& cloud_deref = *cloud;

    scan_.ranges.reserve(cloud_deref.size());
    scan_.intensities.reserve(cloud_deref.size());
    scan_.ranges.clear();
    scan_.intensities.clear();

    size_t start_i = 0;

    for (size_t i = 0; i < cloud_deref.size(); ++i)
    {
      const auto& point = cloud_deref[i];

      if (i > 0 && cloud_deref[i - 1].h > point.h)
      {
        scan_.ranges.back() = point.d;
        scan_.intensities.back() = point.intensity;
        start_i = i;
        continue;
      }

      scan_.ranges.push_back(point.d);
      scan_.intensities.push_back(point.intensity);
    }

    scan_.angle_increment = 0.0;
    scan_.time_increment = 0.0;

    if (cloud_deref.size() > 0)
    {
      scan_.angle_increment = (cloud_deref.back().h - cloud_deref[start_i].h) / static_cast<float>(cloud_deref.size());
      scan_.time_increment = scan_.scan_time / static_cast<float>(cloud_deref.size());
    }

    scan_.angle_min = angles::normalize_angle(cloud_deref[start_i].h);
    scan_.angle_max = angles::normalize_angle(cloud_deref.back().h);
    if (cloud_deref.size() > 1)
    {
      scan_.angle_increment = (scan_.angle_max - scan_.angle_min) / static_cast<float>(cloud->size() - 1);
    }

    scan_.header.stamp.fromSec(static_cast<double>(cloud->header.stamp) / 1e6);
    scan_.header.frame_id = cloud->header.frame_id;

    // don't block if publisher isn't available
    std::unique_lock<std::timed_mutex> lock(publisher_mutex_, std::chrono::milliseconds(10));
    if (lock)
    {
      publisher_.publish(scan_);
    }
  }

  /** \brief run the publisher; blocks until done */
  void run(double frequency = 50.)
  {
    ros::Rate r(frequency);
    while (ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }
  }

  void stop()
  {
    ros::shutdown();
  }

private:
  std::timed_mutex publisher_mutex_;
  bool use_ros_time_;
  sensor_msgs::LaserScan scan_;
  ros::Publisher publisher_;
};


#endif
