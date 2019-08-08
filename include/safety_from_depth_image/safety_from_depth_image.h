/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SCAN_H
#define SCAN_H

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <safety_from_depth_image/WfiFromDepthSensorConfig.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <safety_from_depth_image/depth_traits.h>
#include <sstream>
#include <limits.h>
#include <math.h>

using namespace cv_bridge; // CvImage, toCvShare

namespace safety_from_depth_image
{ 
class ScanNodelet : public nodelet::Nodelet
{

  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_in_, it_out_;
  image_transport::CameraSubscriber sub_image_;
  int queue_size_;
  std::string target_frame_id_;
  boost::mutex connect_mutex_;
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Publisher pub_image_safe_;
  ros::Publisher pub_image_min_;
  ros::Publisher pub_image_max_;
  ros::Publisher pub_image_mean_;
  ros::Publisher pub_image_median_;
  
  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef safety_from_depth_image::WfiFromDepthSensorConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  void onInit();

  // Handles (un)subscribing when clients (un)subscribe;
  void connectCb();

  void configCb(Config &config, uint32_t level);

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  // Publish horizontal laserscan (direct from depth image, distorted, not used)
  void analyzeImage(const sensor_msgs::ImageConstPtr& image_msg,
                    const sensor_msgs::CameraInfoConstPtr& info_msg);

  // Declare parameters
  float safety_distance_min_;
  float safety_distance_max_;

  float limit_min_value_;
  float limit_max_value_;
  
  int x_offset_;
  int y_offset_;
  int width_;
  int height_;
  
  // Declare global variables
  CvImageConstPtr source;
  int width_cropped; 
  int height_cropped;
  cv::Mat h_depth_sat;
  cv::Mat v_depth_sat;
  cv::Mat h_depth_raw;

};
  
}; // depthimage_to_laserscan

#endif
