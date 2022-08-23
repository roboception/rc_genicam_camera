/*
 * Copyright (c) 2019 Roboception GmbH
 * All rights reserved
 *
 * Author: Heiko Hirschmueller
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "genicam_camera_node.h"
#include "timestamp_corrector.h"

#include <rc_genicam_api/device.h>
#include <rc_genicam_api/stream.h>
#include <rc_genicam_api/buffer.h>
#include <rc_genicam_api/config.h>
#include <rc_genicam_api/pixel_formats.h>

#include <pluginlib/class_list_macros.h>
#include <exception>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cctype>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace rcgccam
{
// TODO check this 
#define ROS_HAS_STEADYTIME (ROS_VERSION_MINIMUM(1, 13, 1) || ((ROS_VERSION_MINOR == 12) && ROS_VERSION_PATCH >= 8))

explicit GenICamCameraNode::GenICamCameraNode(const std::string& node_name)
: Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
  timestamp_tolerance_ = 0;
  sync_tolerance_ = 0;
  rotate_ = false;
  running_ = false;
  
  this->declare_parameter("frame_id");
  this->declare_parameter("device");
  this->declare_parameter("gev_access");
  this->declare_parameter("config_file");
  this->declare_parameter("calib_file");
  this->declare_parameter("calib_id");
  this->declare_parameter("host_timestamp", false);
  this->declare_parameter("timestamp_tolerance_", 0.01);
  this->declare_parameter("sync_info");
  this->declare_parameter("sync_tolerance", 0.019);
  this->declare_parameter("image_prefix");
  this->declare_parameter("rotate");
}

GenICamCameraNode::~GenICamCameraNode()
{
  RCLCPP_INFO(this->get_logger(), "rc_genicam_camera: Shutting down");

  // signal running_ threads and wait until they finish

  running_ = false;
  if (grab_thread_.joinable())
  {
    grab_thread_.join();
  }

  rcg::System::clearSystems();
}

void GenICamCameraNode::onInit()
{
  RCLCPP_INFO(this->get_logger(), "rc_genicam_camera: Initialization");

  // get parameter configuration

  std::string device = "";
  std::string access = "control";
  std::string config = "";
  std::string calib = "";
  int calib_id=-1;

  frame_id_ = this->get_parameter("frame_id").as_string();

  if (frame_id_.size() == 0)
  {
    std::string ns = this->get_namespace();

    if (ns.size() > 0 && ns[0] == '/')
    {
      ns = ns.substr(1);
    }

    if (ns.size() > 0)
    {
      frame_id_ = ns + "_camera";
    }
    else
    {
      frame_id_ = "camera";
    }

    int cid=-1;
    cid = this->get_parameter("calib_id").as_int();

    if (cid >= 0)
    {
      frame_id_ = frame_id_ + std::to_string(cid);
    }
  }

  device = this->get_parameter("device").as_string();
  access = this->get_parameter("gev_access").as_string();
  config = this->get_parameter("config_file").as_string();
  calib = this->get_parameter("calib_file").as_string();
  calib_id = this->get_parameter("calib_id").as_int();

  if (device.size() == 0)
  {
    RCLCPP_FATAL(this->get_logger(), "The GenICam device ID must be given in the private parameter 'device'!");
    return;
  }

  rcg::Device::ACCESS access_id;
  if (access == "exclusive")
  {
    access_id = rcg::Device::EXCLUSIVE;
  }
  else if (access == "control")
  {
    access_id = rcg::Device::CONTROL;
  }
  else
  {
    RCLCPP_FATAL_STREAM(this->get_logger(), "rc_visard_driver: Access must be 'control' or 'exclusive': " << access);
    return;
  }

  // optional parameters for timestamp correction

  auto host_timestamp = this->get_parameter("host_timestamp").as_bool();
  timestamp_tolerance_ = this->get_parameter("timestamp_tolerance_").as_double();

  if (!host_timestamp)
  {
    timestamp_tolerance_ = -1;
  }

  // optional parameters for timestamp synchronization

  auto sync_info = this->get_parameter("sync_info").as_string();
  sync_tolerance_ = this->get_parameter("sync_tolerance").as_double();

  if (sync_info.size() > 0)
  {
    sub_sync_info_ptr_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      sync_info,
      rclcpp::SensorDataQoS(),
      std::bind(&GenICamCameraNode::syncInfo, this, std::placeholders::_1)
    );

    image_list_.setSize(25);
    image_list_.setTolerance(static_cast<uint64_t>(sync_tolerance_ * 1000000000.0));

    info_list_.setSize(25);
    info_list_.setTolerance(static_cast<uint64_t>(sync_tolerance_ * 1000000000.0));
  }
  else
  {
    sync_tolerance_ = -1;
  }

  // setup service for getting and setting parameters
  get_param_service_ptr_ = this->create_service<rc_genicam_camera::srv::GetGenICamParameter>(
    "get_genicam_parameter",
    std::bind(&GenICamCameraNode::getGenICamParameter, this, std::placeholders::_1)
  );

  set_param_service_ptr_ = this->create_service<rc_genicam_camera::srv::SetGenICamParameter>(
    "set_genicam_parameter",
    std::bind(&GenICamCameraNode::setGenICamParameter, this, std::placeholders::_1)
  );

  // initialize publishers

  caminfo_pub_.init(nh, calib.c_str(), calib_id);

  image_transport::ImageTransport it(nh);
  image_pub_.init(it);

  // get optional prefix for storing all grabbed images

  pnh.param("image_prefix", image_prefix_, image_prefix_);

  // rotating images by 180 degrees?

  pnh.param("rotate", rotate_, rotate_);

  // start grabbing threads

  running_ = true;
  grab_thread_ = std::thread(&GenICamCameraNode::grab, this, device, access_id, config);
}

namespace
{
// read file as string

std::string loadConfig(const std::string& filename)
{
  if (filename.size() > 0)
  {
    std::ifstream in(filename);
    std::stringstream buffer;

    if (in)
    {
      buffer << in.rdbuf();
      return buffer.str();
    }
    else
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "rc_genicam_camera: Cannot load config: " << filename);
    }
  }

  return std::string();
}

// Expect one or more GenICam parameter name and values in the format
// <name>[=<value>], which must not contain a white space, separated by white
// spaces and applies them to the given nodemap.
//
// An expection is thrown in case of an error. Execution stops on the first
// error.

void applyParameters(const std::shared_ptr<GenApi::CNodeMapRef>& nodemap, const std::string& parameters)
{
  size_t i = 0;

  while (i < parameters.size())
  {
    // eat all white spaces

    while (i < parameters.size() && std::isspace(parameters[i]))
      i++;

    // skip comments which start with # and end by \n

    if (i < parameters.size() && parameters[i] == '#')
    {
      while (i < parameters.size() && parameters[i] != '\n')
        i++;
    }
    else
    {
      size_t k = i;
      while (k < parameters.size() && !std::isspace(parameters[k]))
        k++;

      size_t j = parameters.find('=', i);

      if (j <= k)
      {
        std::string name = parameters.substr(i, j - i);
        std::string value = parameters.substr(j + 1, k - j - 1);

        rcg::setString(nodemap, name.c_str(), value.c_str(), true);
      }
      else if (i < k)
      {
        std::string name = parameters.substr(i, k - i);

        rcg::callCommand(nodemap, name.c_str(), true);
      }

      i = k;
    }
  }
}

}  // namespace

bool GenICamCameraNode::getGenICamParameter(rc_genicam_camera::GetGenICamParameter::Request& req,
                                               rc_genicam_camera::GetGenICamParameter::Response& resp)
{
  std::lock_guard<std::mutex> lock(device_mtx_);

  if (rcgnodemap_)
  {
    try
    {
      resp.value = rcg::getString(rcgnodemap_, req.name.c_str(), true);
      resp.return_code.value = resp.return_code.SUCCESS;
      resp.return_code.message = "ok";
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "rc_genicam_camera: Cannot get parameter: " << ex.what());

      resp.return_code.value = resp.return_code.INVALID_ARGUMENT;
      resp.return_code.message = ex.what();
    }
  }

  return true;
}

bool GenICamCameraNode::setGenICamParameter(rc_genicam_camera::SetGenICamParameter::Request& req,
                                               rc_genicam_camera::SetGenICamParameter::Response& resp)
{
  std::lock_guard<std::mutex> lock(device_mtx_);

  if (rcgnodemap_)
  {
    try
    {
      applyParameters(rcgnodemap_, req.parameters);

      resp.return_code.value = resp.return_code.SUCCESS;
      resp.return_code.message = "ok";
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "rc_genicam_camera: Cannot set parameters: " << ex.what());

      resp.return_code.value = resp.return_code.INVALID_ARGUMENT;
      resp.return_code.message = ex.what();
    }
  }

  return true;
}

namespace
{

void storeImage(const std::string &prefix, const sensor_msgs::ImagePtr &image)
{
  // prepare file name

  std::ostringstream name;

  uint64_t t_sec = image->header.stamp.sec;
  uint64_t t_nsec = image->header.stamp.nsec;

  name << prefix << "_" << t_sec << "." << std::setfill('0') << std::setw(9) << t_nsec << ".pgm";

  // store 8 bit images

  if (image->encoding == sensor_msgs::image_encodings::MONO8 ||
    image->encoding == sensor_msgs::image_encodings::BAYER_BGGR8 ||
    image->encoding == sensor_msgs::image_encodings::BAYER_GBRG8 ||
    image->encoding == sensor_msgs::image_encodings::BAYER_GRBG8 ||
    image->encoding == sensor_msgs::image_encodings::BAYER_RGGB8)
  {
    size_t width = image->width;
    size_t height = image->height;
    uint8_t* p = reinterpret_cast<uint8_t*>(&image->data[0]);

    FILE *out = fopen(name.str().c_str(), "w");

    if (out)
    {
      fprintf(out, "P5\n%lu %lu\n255\n", width, height);
      size_t n = fwrite(p, 1, width*height, out);

      if (n < width*height)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot write to file " << name.str() <<
                         " (" << n << " < " << width*height << ")");
      }

      fclose(out);
    }
    else
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot create file " << name.str());
    }
  }
}

}

void GenICamCameraNode::syncInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
{
  sensor_msgs::ImagePtr image;

  {
    std::lock_guard<std::mutex> lock(sync_mtx_);

    // find image that corresponds to camera info

    image = image_list_.find(info->header.stamp);

    if (image != 0)
    {
      // remove all older images and infos

      int n = image_list_.removeOld(image->header.stamp) - 1;
      info_list_.removeOld(info->header.stamp);

      if (n > 0)
      {
        RCLCPP_WARN_STREAM(this->get_logger(),"rc_genicam_camera: Dropped unused images: " << n);
      }

      // correct time stamp of image

      image->header.stamp = info->header.stamp;
    }
    else
    {
      // store info

      info_list_.add(info);
    }
  }

  // publish images

  if (image)
  {
    caminfo_pub_.publish(image);
    image_pub_.publish(image);

    // store images

    if (image_prefix_.size() > 0)
    {
      storeImage(image_prefix_, image);
    }
  }
}

void GenICamCameraNode::grab(std::string device, rcg::Device::ACCESS access, std::string config)
{
  try
  {
    RCLCPP_INFO(this->get_logger(), "rc_genicam_camera: Grabbing thread started");

    // load initial configuration for camera into string

    std::string init_params = loadConfig(config);

    // initialize optional timestamp correction

    TimestampCorrector ts_host;
    ts_host.setMaximumTolerance(static_cast<int64_t>(timestamp_tolerance_ * 1000000000));
    ts_host.setInterval(1000000000ll);

    // loop until nodelet is killed

    while (running_)
    {
      // report standard exceptions and try again

      try
      {
        {
          std::lock_guard<std::mutex> lock(device_mtx_);

          // open device and get nodemap

          rcgdev_ = rcg::getDevice(device.c_str());

          if (!rcgdev_)
          {
            throw std::invalid_argument("Cannot find device");
          }

          rcgdev_->open(access);
          rcgnodemap_ = rcgdev_->getRemoteNodeMap();

          // initialize camera

          try
          {
            applyParameters(rcgnodemap_, init_params);
          }
          catch (const std::exception& ex)
          {
            RCLCPP_ERROR_STREAM(this->get_logger(), "rc_genicam_camera: Error during initial camera configuration: " << ex.what());
          }
        }

        // initialize timestamp correction

        if (!ts_host.determineOffset(rcgnodemap_))
        {
          RCLCPP_ERROR_STREAM(this->get_logger(),
              "rc_genicam_camera: Cannot determine offset between host and camera clock with maximum tolerance of "
              << timestamp_tolerance_ << " s");
        }

        // start streaming

        std::vector<std::shared_ptr<rcg::Stream> > stream = rcgdev_->getStreams();

        if (stream.size() == 0)
        {
          throw std::invalid_argument("Device does not offer streams");
        }

        stream[0]->open();
        stream[0]->startStreaming();

        ROS_INFO_STREAM("rc_genicam_camera: Start streaming");

        // grabbing thread

        while (running_)
        {
          const rcg::Buffer* buffer = stream[0]->grab(50);

          if (buffer != 0)
          {
            if (!buffer->getIsIncomplete())
            {
              uint32_t npart = buffer->getNumberOfParts();
              for (uint32_t part = 0; part < npart; part++)
              {
                if (buffer->getImagePresent(part))
                {
                  // convert image to ROS

                  sensor_msgs::ImagePtr image = rosImageFromBuffer(frame_id_, buffer, part, rotate_);

                  if (image)
                  {
                    // correct timestamp

                    ts_host.correct(image->header.stamp);

                    // optionally take timestamp of approximately synchronized
                    // camera info

                    if (sync_tolerance_ > 0)
                    {
                      std::lock_guard<std::mutex> lock(sync_mtx_);

                      // find camera info that corresponds to the image

                      sensor_msgs::CameraInfoPtr info = info_list_.find(image->header.stamp);

                      if (info != 0)
                      {
                        // remove all older images and infos

                        int n = image_list_.removeOld(image->header.stamp);
                        info_list_.removeOld(info->header.stamp);

                        if (n > 0)
                        {
                          RCLCPP_WARN_STREAM(this->get_logger(), "rc_genicam_camera: Dropped images: " << n);
                        }

                        // correct time stamp of image

                        image->header.stamp = info->header.stamp;
                      }
                      else
                      {
                        // store image in internal list for later
                        // synchronization to camera info

                        image = image_list_.add(image);

                        if (image)
                        {
                          RCLCPP_WARN_THROTTLE(this->get_logger(), 10, "rc_genicam_camera: Input queue full, dropping image");
                        }

                        image.reset();
                      }
                    }

                    // publish images

                    if (image)
                    {
                      caminfo_pub_.publish(image);
                      image_pub_.publish(image);

                      // store images

                      if (image_prefix_.size() > 0)
                      {
                        storeImage(image_prefix_, image);
                      }
                    }
                  }
                  else
                  {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "rc_genicam_camera: Unsupported pixel format");
                  }
                }
              }

              // re-determine offset of host and camera clock

              if (!ts_host.determineOffset(rcgnodemap_))
              {
                RCLCPP_ERROR_STREAM(this->get_logger(), "rc_genicam_camera: Cannot determine offset between host and camera clock with "
                                 "maximum tolerance of "
                                 << timestamp_tolerance_ << " s");
              }
            }
            else
            {
              RCLCPP_WARN_STREAM(this->get_logger(), "rc_genicam_camera: Incomplete buffer received");
            }
          }
        }

        // stop streaming

        stream[0]->stopStreaming();
        stream[0]->close();
      }
      catch (const std::exception& ex)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "rc_genicam_camera: " << ex.what());
        sleep(1);
      }

      // close device

      {
        std::lock_guard<std::mutex> lock(device_mtx_);

        if (rcgdev_)
          rcgdev_->close();

        rcgdev_.reset();
        rcgnodemap_.reset();
      }
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_FATAL_STREAM(this->get_logger(), "rc_genicam_camera: " << ex.what());
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(this->get_logger(),"rc_genicam_camera: Unknown exception");
  }

  running_ = false;
  RCLCPP_INFO(this->get_logger(), ("rc_genicam_camera: Grabbing thread stopped");
}

}  // namespace rcgccam

PLUGINLIB_EXPORT_CLASS(rcgccam::GenICamCameraNode, nodelet::Nodelet)
