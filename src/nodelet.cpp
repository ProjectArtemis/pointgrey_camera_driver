/*
This code was developed by the National Robotics Engineering Center (NREC), part of the Robotics Institute at Carnegie Mellon University.
Its development was funded by DARPA under the LS3 program and submitted for public release on June 7th, 2012.
Release was granted on August, 21st 2012 with Distribution Statement "A" (Approved for Public Release, Distribution Unlimited).

This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Carnegie Mellon University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/



/**
   @file nodelet.cpp
   @author Chad Rockey
   @date July 13, 2011
   @brief ROS nodelet for the Point Grey Chameleon Camera

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

// ROS and associated nodelet interface and PLUGINLIB declaration header
#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "pointgrey_camera_driver/PointGreyCamera.h" // The actual standalone library for the PointGreys

#include <image_transport/image_transport.h> // ROS library that allows sending compressed images
#include <camera_info_manager/camera_info_manager.h> // ROS library that publishes CameraInfo topics
#include <sensor_msgs/CameraInfo.h> // ROS message header for CameraInfo

#include <mavros_msgs/CamIMUStamp.h>
#include <mavros_msgs/CommandTriggerControl.h>

#include <boost/thread.hpp> // Needed for the nodelet to launch the reading thread.

#include <dynamic_reconfigure/server.h> // Needed for the dynamic_reconfigure gui service to run

#include <fstream>

#define MAX_BUFFER_SIZE 5

namespace pointgrey_camera_driver
{

class PointGreyCameraNodelet: public nodelet::Nodelet
{
public:
  PointGreyCameraNodelet() {}

  ~PointGreyCameraNodelet()
  {
    boost::mutex::scoped_lock scopedLock(connect_mutex_);

    if(pubThread_)
    {
      pubThread_->interrupt();
      pubThread_->join();

      try
      {
        NODELET_DEBUG("Stopping camera capture.");
        pg_.stop();
        NODELET_DEBUG("Disconnecting from camera.");
        pg_.disconnect();
      }
      catch(std::runtime_error& e)
      {
        NODELET_ERROR("%s", e.what());
      }
    }
  }

private:
  /*!
  * \brief Function that allows reconfiguration of the camera.
  *
  * This function serves as a callback for the dynamic reconfigure service.  It simply passes the configuration object to the driver to allow the camera to reconfigure.
  * \param config  camera_library::CameraConfig object passed by reference.  Values will be changed to those the driver is currently using.
  * \param level driver_base reconfiguration level.  See driver_base/SensorLevels.h for more information.
  */
  void paramCallback(pointgrey_camera_driver::PointGreyConfig &config, uint32_t level)
  {
    config_ = config;

    try
    {
      NODELET_DEBUG("Dynamic reconfigure callback with level: %d", level);
      // TODO ; fix this nicely
      pg_.setNewConfiguration(config, level, false);
    }
    catch(std::runtime_error& e)
    {
      NODELET_ERROR("Reconfigure Callback failed with error: %s", e.what());
    }
  }

  /*!
  * \brief Connection callback to only do work when someone is listening.
  *
  * This function will connect/disconnect from the camera depending on who is using the output.
  */
  void connectCb()
  {
    NODELET_DEBUG("Connect callback!");
    boost::mutex::scoped_lock scopedLock(connect_mutex_); // Grab the mutex.  Wait until we're done initializing before letting this function through.
    // Check if we should disconnect (there are 0 subscribers to our data)
    if(it_pub_.getNumSubscribers() == 0 && dynamic_connect_)
    {
      if (pubThread_)
      {
        NODELET_DEBUG("Disconnecting.");
        pubThread_->interrupt();
        scopedLock.unlock();
        pubThread_->join();
        scopedLock.lock();
        pubThread_.reset();
        vi_sync_sub_.shutdown();
        capture_control_srv_.shutdown();

        try
        {
          NODELET_DEBUG("Stopping camera capture.");
          pg_.stop();
        }
        catch(std::runtime_error& e)
        {
          NODELET_ERROR("%s", e.what());
        }

        try
        {
          NODELET_DEBUG("Disconnecting from camera.");
          pg_.disconnect();
        }
        catch(std::runtime_error& e)
        {
          NODELET_ERROR("%s", e.what());
        }
      }
    } 
    else if(!pubThread_ && dynamic_connect_)     // We need to connect
    {
      NODELET_DEBUG("Connecting.");
      // Start the thread to loop through and publish messages
      pubThread_.reset(new boost::thread(boost::bind(&pointgrey_camera_driver::PointGreyCameraNodelet::devicePoll, this)));
    }

    if(!pubThread_ && !dynamic_connect_)     // We need to connect
    {
      NODELET_DEBUG("Connecting.");
      // Start the thread to loop through and publish messages
      pubThread_.reset(new boost::thread(boost::bind(&pointgrey_camera_driver::PointGreyCameraNodelet::devicePoll, this)));
    }
  }

  /*!
  * \brief Serves as a psuedo constructor for nodelets.
  *
  * This function needs to do the MINIMUM amount of work to get the nodelet running.  Nodelets should not call blocking functions here.
  */
  void onInit()
  {
    // Get nodeHandles
    ros::NodeHandle &nh = getMTNodeHandle();
    ros::NodeHandle &pnh = getMTPrivateNodeHandle();

    // Get a serial number through ros
    int serial = 0;

    XmlRpc::XmlRpcValue serial_xmlrpc;
    pnh.getParam("serial", serial_xmlrpc);
    if (serial_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      pnh.param<int>("serial", serial, 0);
    }
    else if (serial_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      std::string serial_str;
      pnh.param<std::string>("serial", serial_str, "0");
      std::istringstream(serial_str) >> serial;
    }
    else
    {
      NODELET_DEBUG("Serial XMLRPC type.");
      serial = 0;
    }

    std::string camera_serial_path;
    pnh.param<std::string>("camera_serial_path", camera_serial_path, "");
    NODELET_INFO("Camera serial path %s", camera_serial_path.c_str());
    // If serial has been provided directly as a param, ignore the path
    // to read in the serial from.
    while (serial == 0 && !camera_serial_path.empty())
    {
      serial = readSerialAsHexFromFile(camera_serial_path);
      if (serial == 0)
      {
        NODELET_WARN("Waiting for camera serial path to become available");
        ros::Duration(1.0).sleep(); // Sleep for 1 second, wait for serial device path to become available
      }
    }

    NODELET_INFO("Using camera serial %d", serial);

    pg_.setDesiredCamera((uint32_t)serial);

    // Get GigE camera parameters:
    pnh.param<int>("packet_size", packet_size_, 1400);
    pnh.param<bool>("auto_packet_size", auto_packet_size_, true);
    pnh.param<int>("packet_delay", packet_delay_, 4000);

    // Set GigE parameters:
    pg_.setGigEParameters(auto_packet_size_, packet_size_, packet_delay_);

    // 
    pnh.param<bool>("enable_synchronisation", enable_synchronisation_, true);

    // Get the location of our camera config yaml
    std::string camera_info_url;
    pnh.param<std::string>("camera_info_url", camera_info_url, "");
    // Get the desired frame_id, set to 'camera' if not found
    pnh.param<std::string>("frame_id", frame_id_, "camera");
    // Get dynamic connect-on-subscription setting
    pnh.param<bool>("dynamic_connect", dynamic_connect_, false);
    // 
    pnh.param<bool>("commit_settings", commit_settings_, false);
    // 
    pnh.param<bool>("factory_reset", factory_reset_, false);

    // Do not call the connectCb function until after we are done initializing.
    boost::mutex::scoped_lock scopedLock(connect_mutex_);

    // Start up the dynamic_reconfigure service, note that this needs to stick around after this function ends
    dynamic_reconfigure_srv_ = boost::make_shared <dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig> > (pnh);
    dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig>::CallbackType f =
      boost::bind(&pointgrey_camera_driver::PointGreyCameraNodelet::paramCallback, this, _1, _2);
    dynamic_reconfigure_srv_->setCallback(f);

    // Start the camera info manager and attempt to load any configurations
    std::stringstream cinfo_name;
    cinfo_name << serial;
    cinfo_.reset(new camera_info_manager::CameraInfoManager(nh, cinfo_name.str(), camera_info_url));

    image_transport::SubscriberStatusCallback cb;
    if(dynamic_connect_) {
      cb = boost::bind(&PointGreyCameraNodelet::connectCb, this);
    }
    // Publish topics using ImageTransport through camera_info_manager (gives cool things like compression)
    it_.reset(new image_transport::ImageTransport(nh));
    it_pub_ = it_->advertiseCamera("image_raw", 5, cb, cb);

    // Setup synchronisation buffers
    if(enable_synchronisation_) {
      frame_buffer_.reserve(MAX_BUFFER_SIZE);
      timestamp_buffer_.reserve(MAX_BUFFER_SIZE);
    }

    if(!dynamic_connect_) {
      scopedLock.unlock();
      connectCb();
    }

  }

  /**
   * @brief Reads in the camera serial from a specified file path.
   * The format of the serial is expected to be base 16.
   * @param camera_serial_path The path of where to read in the serial from. Generally this
   * is a USB device path to the serial file.
   * @return int The serial number for the given path, 0 if failure.
   */
  int readSerialAsHexFromFile(std::string camera_serial_path)
  {
    NODELET_DEBUG("Reading camera serial file from: %s", camera_serial_path.c_str());

    std::ifstream serial_file(camera_serial_path.c_str());
    std::stringstream buffer;
    int serial = 0;

    if (serial_file.is_open())
    {
      std::string serial_str((std::istreambuf_iterator<char>(serial_file)), std::istreambuf_iterator<char>());
      NODELET_DEBUG("Serial file contents: %s", serial_str.c_str());
      buffer << std::hex << serial_str;
      buffer >> serial;
      NODELET_DEBUG("Serial discovered %d", serial);

      return serial;
    }

    NODELET_WARN("Unable to open serial path: %s", camera_serial_path.c_str());
    return 0;
  }


  /*!
  * \brief Function for the boost::thread to grabImages and publish them.
  *
  * This function continues until the thread is interupted.  Responsible for getting sensor_msgs::Image and publishing them.
  */
  void devicePoll()
  {
    enum State
    {
        NONE
      , ERROR
      , STOPPED
      , DISCONNECTED
      , CONNECTED
      , START_CAPTURE
      , STARTED
    };

    State state = DISCONNECTED;
    State previous_state = NONE;

    while(!boost::this_thread::interruption_requested())   // Block until we need to stop this thread.
    {
      bool state_changed = state != previous_state;

      previous_state = state;

      switch(state)
      {
        case ERROR:
          // Generally there's no need to stop before disconnecting after an
          // error. Indeed, stop will usually fail.
#if STOP_ON_ERROR
          // Try stopping the camera
          {
            boost::mutex::scoped_lock scopedLock(connect_mutex_);
            vi_sync_sub_.shutdown();
            capture_control_srv_.shutdown();
          }
          try
          {
            NODELET_DEBUG("Stopping camera.");
            pg_.stop();
            NODELET_DEBUG("Stopped camera.");

            state = STOPPED;
          }
          catch(std::runtime_error& e)
          {
            NODELET_ERROR_COND(state_changed,
                "Failed to stop error: %s", e.what());
            ros::Duration(1.0).sleep(); // sleep for one second each time
          }

          break;
#endif
        case STOPPED:
          // Try disconnecting from the camera
          try
          {
            NODELET_DEBUG("Disconnecting from camera.");
            pg_.disconnect();
            NODELET_DEBUG("Disconnected from camera.");

            state = DISCONNECTED;
          }
          catch(std::runtime_error& e)
          {
            NODELET_ERROR_COND(state_changed,
                "Failed to disconnect with error: %s", e.what());
            ros::Duration(1.0).sleep(); // sleep for one second each time
          }

          break;
        case DISCONNECTED:
          // Try connecting to the camera

          try
          {
            NODELET_DEBUG("Connecting to camera.");
            pg_.connect();
            NODELET_DEBUG("Connected to camera.");

            // Set last configuration, forcing the reconfigure level to stop
            if (commit_settings_) {
              pg_.setNewConfiguration(config_, PointGreyCamera::LEVEL_RECONFIGURE_STOP, true);
              NODELET_WARN("Saved settings. Power cycle camera and restart driver without commit_settings.");
              return;
            }

            if (factory_reset_) {
              pg_.commitSettings(true);
              NODELET_WARN("Factory reset settings. Power cycle camera and restart driver without factory_reset.");
              return;
            }

            // Set the timeout for grabbing images.
            try
            {
              double timeout;
              getMTPrivateNodeHandle().param("timeout", timeout, 1.0);

              NODELET_DEBUG("Setting timeout to: %f.", timeout);
              pg_.setTimeout(timeout);
            }
            catch(std::runtime_error& e)
            {
              NODELET_ERROR("%s", e.what());
            }

            if(enable_synchronisation_)
            {
              // Subscribe to synchronisation messages
              boost::mutex::scoped_lock scopedLock(connect_mutex_);
              vi_sync_sub_ = getMTNodeHandle().subscribe("/mavros/cam_imu_sync/cam_imu_stamp", 10, &pointgrey_camera_driver::PointGreyCameraNodelet::VISyncCallback, this);
              capture_control_srv_ = getMTNodeHandle().serviceClient<mavros_msgs::CommandTriggerControl>("/mavros/cmd/trigger_control");
            }

            state = CONNECTED;
          }
          catch(std::runtime_error& e)
          {
            NODELET_ERROR_COND(state_changed,
                "Failed to connect with error: %s", e.what());
            ros::Duration(1.0).sleep(); // sleep for one second each time
          }

          break;
        case CONNECTED:
          // Try starting the camera
          try
          {
            NODELET_DEBUG("Starting camera.");
            pg_.start();
            NODELET_DEBUG("Started camera.");

            if (!enable_synchronisation_) 
            {
              // Start sensor readout
              pg_.controlSensorReadout(true);
              state = STARTED;
            } 
            else
            {
              state = START_CAPTURE;
            } 
          }
          catch(std::runtime_error& e)
          {
            NODELET_ERROR_COND(state_changed,
                "Failed to start with error: %s", e.what());
            ros::Duration(1.0).sleep(); // sleep for one second each time
          }

          break;
        case START_CAPTURE:
          // Start synchronised frame grabbing with external strobe
          try
          {
            NODELET_DEBUG("Starting synchronised capture.");

            // Stop external signal capture if already running
            controlSignalCapture(false);

            // Stop sensor readout
            pg_.controlSensorReadout(false);

            // Enable external signal capture
            controlSignalCapture(true);

            // Start synchronised capture
            pg_.startSynchronizedCapture(base_sequence_);

            NODELET_DEBUG("Started synchronised capture.");
            state = STARTED;
          }
          catch(std::runtime_error& e)
          {
            NODELET_ERROR_COND(state_changed,
                "Failed to start synchronised capture with error: %s", e.what());
            ros::Duration(1.0).sleep(); // sleep for one second each time
          }

          break;
        case STARTED:
          try
          {
            // Get the image from the camera library
            sensor_msgs::Image frame;
            pg_.grabImage(frame, frame_id_, base_sequence_);

            if(enable_synchronisation_)
            {
              NODELET_DEBUG("Added image with sequence %u", frame.header.seq);
              
              // Lock buffer and push
              frame_mutex_.lock();
              frame_buffer_.push_back(frame);
              frame_mutex_.unlock();

              // Synchronise and publish
              synchroniseAndPublishImages();

            } else {
              // Just directly publish image
              ros::Time now = ros::Time::now();
              publishStampedFrame(frame, now);

            }

          }
          catch(CameraTimeoutException& e)
          {
            NODELET_WARN("%s", e.what());
          }
          catch(CameraImageConsistencyError& e)
          {
            NODELET_WARN("%s", e.what());
          }
          catch(std::runtime_error& e)
          {
            NODELET_ERROR("%s", e.what());

            state = ERROR;
          }

          break;
        default:
          NODELET_ERROR("Unknown camera state %d!", state);
      }

    }
    NODELET_DEBUG("Leaving thread.");
  }

  void VISyncCallback(const mavros_msgs::CamIMUStamp& msg)
  {
    // Lock buffer and push
    timestamp_mutex_.lock();
    timestamp_buffer_.push_back(msg);
    timestamp_mutex_.unlock();

    NODELET_DEBUG("Added timestamp with sequence %u", msg.frame_seq_id);

    synchroniseAndPublishImages();

  }

  int synchroniseAndPublishImages()
  {
    // Lock synchronisation buffers
    boost::mutex::scoped_lock scopedLockImage(frame_mutex_);
    boost::mutex::scoped_lock scopedLockTimestamp(timestamp_mutex_);

    int num_images_published = 0;

    if(frame_buffer_.size() < 1 || timestamp_buffer_.size() < 1)
    {
      return 0;
    }

    // Go through image buffer
    for(int image_index = frame_buffer_.size() - 1; image_index >= 0; image_index--)
    {
      uint32_t image_sequence = frame_buffer_[image_index].header.seq;

      // Go through timestamp buffer
      for(int stamp_index = timestamp_buffer_.size() - 1; stamp_index >= 0; stamp_index--)
      {
        uint32_t stamp_sequence = timestamp_buffer_[stamp_index].frame_seq_id;

        // For every image frame, try to locate timestamp message with same sequence number
        if(image_sequence == stamp_sequence) 
        {
          // Publish the data
          publishStampedFrame(frame_buffer_[image_index], timestamp_buffer_[stamp_index].frame_stamp);

          // Erase published frames
          frame_buffer_.erase(frame_buffer_.begin() + image_index);
          timestamp_buffer_.erase(timestamp_buffer_.begin() + stamp_index);

          num_images_published++;
        }
      }
    }

    // Trim the buffers if needed
    trimSynchronisationBuffers();

    return num_images_published;
  }

  void publishStampedFrame(sensor_msgs::Image &frame, ros::Time &timestamp)
  {
    NODELET_DEBUG("Age at publish time : %0.2f ms", double(ros::Time::now().toNSec() - timestamp.toNSec())/1000000.0 );

    // Create CameraInfo
    sensor_msgs::CameraInfo cinfo = cinfo_->getCameraInfo();
    cinfo.header = frame.header;
    // TODO : binning and other parameters

    // Publish image and camera info
    it_pub_.publish(frame, cinfo, timestamp);

  }

  void trimSynchronisationBuffers()
  {
    // Trim timestamp buffer
    if(timestamp_buffer_.size() > MAX_BUFFER_SIZE) { // TODO : do this by age of stamp/image
      timestamp_buffer_.erase(timestamp_buffer_.begin());
      NODELET_WARN("Erase timestamp");
    }

    // Trim image buffer
    if(frame_buffer_.size() > MAX_BUFFER_SIZE) { // TODO : do this by age of stamp/image
      frame_buffer_.erase(frame_buffer_.begin());
      NODELET_WARN("Erase image");
    }

  }

  void controlSignalCapture(bool enable)
  {
    mavros_msgs::CommandTriggerControl srv;
    srv.request.trigger_enable = enable;
    srv.request.reset_sequence = true;

    do {
      capture_control_srv_.call(srv);
      if(!srv.response.success) 
      {
        NODELET_ERROR("Failed to call capture control service.");
        ros::Duration(1.0).sleep(); // sleep for one second each time
      }
    } while(!srv.response.success);

    NODELET_DEBUG("%s camera signal capture.", enable ? "Enabled" : "Disabled");

  }

  boost::shared_ptr<dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig> > dynamic_reconfigure_srv_; ///< Needed to initialize and keep the dynamic_reconfigure::Server in scope.
  boost::shared_ptr<image_transport::ImageTransport> it_; ///< Needed to initialize and keep the ImageTransport in scope.
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_; ///< Needed to initialize and keep the CameraInfoManager in scope.
  image_transport::CameraPublisher it_pub_; ///< CameraInfoManager ROS publisher

  ros::Subscriber vi_sync_sub_;
  ros::ServiceClient capture_control_srv_;
  std::vector<sensor_msgs::Image> frame_buffer_;
  std::vector<mavros_msgs::CamIMUStamp> timestamp_buffer_;
  uint32_t base_sequence_;

  boost::mutex connect_mutex_;
  boost::mutex frame_mutex_;
  boost::mutex timestamp_mutex_;

  PointGreyCamera pg_; ///< Instance of the PointGreyCamera library, used to interface with the hardware.
  sensor_msgs::CameraInfoPtr ci_; ///< Camera Info message.
  std::string frame_id_; ///< Frame id for the camera messages, defaults to 'camera'
  boost::shared_ptr<boost::thread> pubThread_; ///< The thread that reads and publishes the images.

  bool enable_synchronisation_;
  bool dynamic_connect_;
  bool commit_settings_;
  bool factory_reset_;

  // Parameters for cameraInfo
  size_t binning_x_; ///< Camera Info pixel binning along the image x axis.
  size_t binning_y_; ///< Camera Info pixel binning along the image y axis.

  // For GigE cameras:
  /// If true, GigE packet size is automatically determined, otherwise packet_size_ is used:
  bool auto_packet_size_;
  /// GigE packet size:
  int packet_size_;
  /// GigE packet delay:
  int packet_delay_;

  /// Configuration:
  pointgrey_camera_driver::PointGreyConfig config_;
};

PLUGINLIB_EXPORT_CLASS(pointgrey_camera_driver::PointGreyCameraNodelet, nodelet::Nodelet)  // Needed for Nodelet declaration
}
