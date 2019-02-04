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

#include "genicam_camera_nodelet.h"
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

#include <ros/ros.h>

namespace rcgccam
{
#define ROS_HAS_STEADYTIME (ROS_VERSION_MINIMUM(1, 13, 1) || ((ROS_VERSION_MINOR == 12) && ROS_VERSION_PATCH >= 8))

GenICamCameraNodelet::GenICamCameraNodelet()
{
  timestamp_tolerance_ = 0;
  sync_tolerance_ = 0;
  running_ = false;
}

GenICamCameraNodelet::~GenICamCameraNodelet()
{
  ROS_INFO("rc_genicam_camera: Shutting down");

  // signal running_ threads and wait until they finish

  running_ = false;
  if (grab_thread_.joinable())
  {
    grab_thread_.join();
  }

  rcg::System::clearSystems();
}

void GenICamCameraNodelet::onInit()
{
  ROS_INFO("rc_genicam_camera: Initialization");

  // get parameter configuration

  ros::NodeHandle pnh(getPrivateNodeHandle());
  ros::NodeHandle nh(getNodeHandle(), "");

  std::string device = "";
  std::string access = "control";
  std::string config = "";
  std::string calib = "";
  int calib_id=-1;

  std::string ns = ros::this_node::getNamespace();
  frame_id_ = ns + "camera";

  pnh.param("device", device, device);
  pnh.param("gev_access", access, access);
  pnh.param("config_file", config, config);
  pnh.param("calib_file", calib, calib);
  pnh.param("calib_id", calib_id, calib_id);

  if (device.size() == 0)
  {
    ROS_FATAL("The GenICam device ID must be given in the private parameter 'device'!");
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
    ROS_FATAL_STREAM("rc_visard_driver: Access must be 'control' or 'exclusive': " << access);
    return;
  }

  // optional parameters for timestamp correction

  bool host_timestamp = false;
  pnh.param("host_timestamp", host_timestamp, host_timestamp);
  pnh.param("timestamp_tolerance_", timestamp_tolerance_, 0.01);

  if (!host_timestamp)
  {
    timestamp_tolerance_ = -1;
  }

  // optional parameters for timestamp synchronization

  std::string sync_info;
  pnh.param("sync_info", sync_info, sync_info);
  pnh.param("sync_tolerance", sync_tolerance_, 0.019);

  if (sync_info.size() > 0)
  {
    sub_sync_info_ = nh.subscribe(sync_info, 10, &GenICamCameraNodelet::syncInfo, this);

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

  get_param_service_ = pnh.advertiseService("get_genicam_parameter", &GenICamCameraNodelet::getGenICamParameter, this);

  set_param_service_ = pnh.advertiseService("set_genicam_parameter", &GenICamCameraNodelet::setGenICamParameter, this);

  // initialize publishers

  caminfo_pub_.init(nh, calib.c_str(), calib_id);

  image_transport::ImageTransport it(nh);
  image_pub_.init(it);

  // start grabbing threads

  running_ = true;
  grab_thread_ = std::thread(&GenICamCameraNodelet::grab, this, device, access_id, config);
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
    }
    else
    {
      ROS_ERROR_STREAM("rc_genicam_camera: Cannot load config: " << filename);
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

bool GenICamCameraNodelet::getGenICamParameter(rc_genicam_camera::GetGenICamParameter::Request& req,
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
      ROS_ERROR_STREAM("rc_genicam_camera: Cannot get parameter: " << ex.what());

      resp.return_code.value = resp.return_code.INVALID_ARGUMENT;
      resp.return_code.message = ex.what();
    }
  }

  return true;
}

bool GenICamCameraNodelet::setGenICamParameter(rc_genicam_camera::SetGenICamParameter::Request& req,
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
      ROS_ERROR_STREAM("rc_genicam_camera: Cannot set parameters: " << ex.what());

      resp.return_code.value = resp.return_code.INVALID_ARGUMENT;
      resp.return_code.message = ex.what();
    }
  }

  return true;
}

void GenICamCameraNodelet::syncInfo(sensor_msgs::CameraInfoPtr info)
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
        ROS_WARN_STREAM("rc_genicam_camera: Dropped unused images: " << n);
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
  }
}

void GenICamCameraNodelet::grab(std::string device, rcg::Device::ACCESS access, std::string config)
{
  try
  {
    ROS_INFO("rc_genicam_camera: Grabbing thread started");

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
            ROS_ERROR_STREAM("rc_genicam_camera: Error during initial camera configuration: " << ex.what());
          }
        }

        // initialize timestamp correction

        if (!ts_host.determineOffset(rcgnodemap_))
        {
          ROS_ERROR_STREAM(
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

                  sensor_msgs::ImagePtr image = rosImageFromBuffer(frame_id_, buffer, part);

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
                          ROS_WARN_STREAM("rc_genicam_camera: Dropped images: " << n);
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
                          ROS_WARN_THROTTLE(10, "rc_genicam_camera: Input queue full, dropping image");
                        }

                        image.reset();
                      }
                    }

                    // publish images

                    if (image)
                    {
                      caminfo_pub_.publish(image);
                      image_pub_.publish(image);
                    }
                  }
                  else
                  {
                    ROS_ERROR_STREAM("rc_genicam_camera: Unsupported pixel format");
                  }
                }
              }

              // re-determine offset of host and camera clock

              if (!ts_host.determineOffset(rcgnodemap_))
              {
                ROS_ERROR_STREAM("rc_genicam_camera: Cannot determine offset between host and camera clock with "
                                 "maximum tolerance of "
                                 << timestamp_tolerance_ << " s");
              }
            }
            else
            {
              ROS_WARN_STREAM("rc_genicam_camera: Incomplete buffer received");
            }
          }
        }

        // stop streaming

        stream[0]->stopStreaming();
        stream[0]->close();
      }
      catch (const std::exception& ex)
      {
        ROS_ERROR_STREAM("rc_genicam_camera: " << ex.what());
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
    ROS_FATAL_STREAM("rc_genicam_camera: " << ex.what());
  }
  catch (...)
  {
    ROS_FATAL_STREAM("rc_genicam_camera: Unknown exception");
  }

  running_ = false;
  ROS_INFO("rc_genicam_camera: Grabbing thread stopped");
}

}  // namespace rcgccam

PLUGINLIB_EXPORT_CLASS(rcgccam::GenICamCameraNodelet, nodelet::Nodelet)
