/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef QUANERGY_CLIENT_ROS_LASERSCAN_PUBLISHER_H
#define QUANERGY_CLIENT_ROS_LASERSCAN_PUBLISHER_H

#include <pcl/point_cloud.h>
#include <quanergy/common/point_hvdir.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <mutex>

class ScanPublisher
{
public:
  using CloudConstPtr = pcl::PointCloud<quanergy::PointHVDIR>::ConstPtr;

  ScanPublisher(
    ros::NodeHandle& nh,
    const std::string& topic,
    double frame_rate,
    float range_min,
    float range_max,
    bool use_ros_time = true)
    : use_ros_time_(use_ros_time)
  {
    publisher_ = nh.advertise<sensor_msgs::LaserScan>(topic, 10);

    scan_.range_min = range_min;
    scan_.range_max = range_max;
    scan_.scan_time = 1.0f / static_cast<float>(frame_rate);
  }

  void slot(const CloudConstPtr& cloud)
  {
    if(!ros::ok() || !cloud || cloud->empty()) return;

    scan_.header.stamp = ros::Time::now();

    const auto& cloud_deref = *cloud;

    scan_.ranges.reserve(cloud_deref.size());
    scan_.intensities.reserve(cloud_deref.size());
    scan_.ranges.clear();
    scan_.intensities.clear();

    size_t start_i = 0;

    for (size_t i = 0; i < cloud_deref.size(); ++i)
    {
      const auto& point = cloud_deref[i];

      // Some scans have points near pi at the start (possible left over from a previous scan?)
      if (i > 0 && cloud_deref[i - 1].h > point.h)
      {
        scan_.ranges.clear();
        scan_.intensities.clear();
        start_i = i;
      }

      scan_.ranges.push_back(point.d);
      scan_.intensities.push_back(point.intensity);
    }

    scan_.time_increment = scan_.scan_time / static_cast<float>(cloud_deref.size());

    scan_.angle_min = cloud_deref[start_i].h;
    scan_.angle_max = cloud_deref.back().h;
    scan_.angle_increment = (scan_.angle_max - scan_.angle_min) / static_cast<float>(cloud_deref.size() - start_i);
    scan_.header.frame_id = cloud->header.frame_id;

    // don't block if publisher isn't available
    std::unique_lock<std::timed_mutex> lock(publisher_mutex_, std::chrono::milliseconds(10));
    if (lock)
    {
      publisher_.publish(scan_);
    }
  }

private:
  std::timed_mutex publisher_mutex_;
  bool use_ros_time_;
  sensor_msgs::LaserScan scan_;
  ros::Publisher publisher_;
};


#endif
