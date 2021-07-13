/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef QUANERGY_CLIENT_ROS_LASERSCAN_PUBLISHER_H
#define QUANERGY_CLIENT_ROS_LASERSCAN_PUBLISHER_H

#include <mutex>

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

  ScanPublisher(ros::NodeHandle& nh, std::string topic, std::string frame_id, bool use_ros_time = false)
    : use_ros_time_(use_ros_time)
  {
    topic = nh.resolveName(topic);
    publisher_ = nh.advertise<sensor_msgs::LaserScan>(topic, 10);
    scan_.header.frame_id = frame_id;
  }

  void slot(const CloudConstPtr& cloud)
  {
    if(!ros::ok() || !cloud || cloud->empty()) return;

    scan_.angle_min = cloud->front().h;
    scan_.angle_max = cloud->back().h;

    scan_.ranges.resize(cloud->size());
    scan_.intensities.resize(cloud->size());

    // scan.time_increment = ?
    scan_.range_min = std::numeric_limits<float>::max();
    scan_.range_max = std::numeric_limits<float>::lowest();  // Should have a fixed value

    scan_.angle_increment = 0.0;

    if (cloud->size() > 1)
    {
      scan_.angle_increment = (scan_.angle_max - scan_.angle_min) / static_cast<float>(cloud->size() - 1);
    }

    for (const auto& point : *cloud)
    {
      scan_.range_min = std::min(point.d, scan_.range_min);
      scan_.range_max = std::max(point.d, scan_.range_max);
    }

    if (use_ros_time_)
    {
      scan_.header.stamp = ros::Time::now();
    }

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
