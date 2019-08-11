/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
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
#include <random>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <boost/circular_buffer.hpp>
#include <numeric>
#include <assert.h>

#include <assert.h>

#include <iterator>
#include <vector>

#include <safety_from_depth_image/safety_from_depth_image.h>

namespace safety_from_depth_image
{

    void ScanNodelet::onInit()
    {

        nh         = getNodeHandle();
        private_nh = getPrivateNodeHandle();
        it_in_ .reset(new image_transport::ImageTransport(nh));

        // Read parameters
        private_nh.param("queue_size", queue_size_, 5);
        private_nh.param("target_frame_id", target_frame_id_, std::string());

        // Set up dynamic reconfigure
        reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
        ReconfigureServer::CallbackType f = boost::bind(&ScanNodelet::configCb, this, _1, _2);
        reconfigure_server_->setCallback(f);

        // Monitor whether anyone is subscribed to the cropped_int
        image_transport::SubscriberStatusCallback connect_cb = boost::bind(&ScanNodelet::connectCb, this);
        ros::SubscriberStatusCallback connect_cb_info = boost::bind(&ScanNodelet::connectCb, this);

        // Make sure we don't enter connectCb() between advertising and assigning to pub_h_scans_
        boost::lock_guard<boost::mutex> lock(connect_mutex_);
        image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());

        // Subscriber RealSense depth image
        sub_image_ = it_in_->subscribeCamera("image_raw", queue_size_, &ScanNodelet::imageCb, this, hints);

        // Publisher image getting_close boolean
        pub_image_closeness_ = nh.advertise<std_msgs::Int32>("status", 10);

        // Publisher image statistic
        pub_image_min_ = nh.advertise<std_msgs::Float32>("min", 10);

    };

    void ScanNodelet::connectCb()
    {
        
    };

    void ScanNodelet::configCb(Config &config, uint32_t level)
    {

        config_ = config;

        // Hole Detection Parameters
        getting_close_distance_ = config.getting_close_distance;
        too_close_distance_ = config.too_close_distance;

        // Depth Limit Parameters
        limit_min_value_ = config.limit_min_value;
        limit_max_value_ = config.limit_max_value;

        // Depth Image Horizontal Strip Cropping Parameters
        x_offset_ = config.x_offset;
        y_offset_ = config.y_offset;
        width_ = config.width;
        height_ = config.height;

    };

    // Called when RealSense outputs depth image
    void ScanNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                              const sensor_msgs::CameraInfoConstPtr& info_msg)
    {

        // Get parameters
        Config config;
        {

            boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
            config = config_;

        }

        // Detects hole
        analyzeImage(image_msg, info_msg);

    };

    ////////////////////////////
    // EXECUTE HOLE DETECTION //
    ////////////////////////////
    
    void ScanNodelet::analyzeImage(const sensor_msgs::ImageConstPtr& image_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        
        int max_h_width = image_msg->width - x_offset_;
        int max_h_height = image_msg->height - y_offset_;
        width_cropped = width_;
        height_cropped = height_;

        if (width_cropped == 0 || width_cropped > max_h_width)
        width_cropped = max_h_width;
        if (height_cropped == 0 || height_cropped > max_h_height)
        height_cropped = max_h_height;

        // Get cv::Mat view of the source data
        source = toCvShare(image_msg);

        ////////////////
        // Crop Image //
        ////////////////

        CvImage cropped_int(source->header, source->encoding);
        CvImage cropped_float(source->header, source->encoding);

        cropped_int.image = source->image(cv::Rect(x_offset_, y_offset_, width_cropped, height_cropped));
    
        ////////////////////////////////
        // Calculate Image Statistics //
        ////////////////////////////////

        // Convert to from 2 bytes (16 bit) to float, and scale from mm to m
        cropped_int.image.convertTo(cropped_float.image, CV_32FC1,0.001); // [mm] --> [m]
        
        // Set close values to NAN so ignored when calculating statistics
        cropped_float.image.setTo(NAN, cropped_float.image < limit_min_value_);

        // Calculate min and max values
        double image_min, image_max;
        cv::minMaxLoc(cropped_float.image, &image_min, &image_max);

        // For some reason these get set to inf (-inf), reset to NAN so ingored
        if( (isinf(image_min) == 1) || (isnan(image_min) == 1) )
        {

            image_min = -1000;

        }

        //////////////////////////////
        // Determine Safety Boolean //
        //////////////////////////////

        // Set to unsafe by default
        int image_closeness = 2;

        // Set to getting close or too close
        if( (image_min < too_close_distance_) )
        {
            image_closeness = 2; // Too Close --> Stop
        }

	else if( (image_min >= too_close_distance_) && (image_min < getting_close_distance_) )
        {
            image_closeness = 1; // Getting Close --> Slow Down
        }

	else if( (image_min >= getting_close_distance_) && (image_min < 100) )
        {
            image_closeness = 0; // Not Close --> Keep Going
        }

        /////////////////////////////////////
        // Publish Image Closeness Message //
        /////////////////////////////////////

        // 0: Not Close
	// 1: Getting Close
	// 2: Too Close
	std_msgs::Int32 image_closeness_msg;
	
        image_closeness_msg.data = (long int)image_closeness;

        pub_image_closeness_.publish(image_closeness_msg);

        /////////////////////////////////////////
        // Publish Depth Image Minimum Message //
        /////////////////////////////////////////

        std_msgs::Float32 image_min_msg;
        image_min_msg.data = image_min;
        pub_image_min_.publish(image_min_msg);

    }

}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( safety_from_depth_image::ScanNodelet, nodelet::Nodelet)
