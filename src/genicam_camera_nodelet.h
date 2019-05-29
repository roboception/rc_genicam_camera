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

#ifndef RC_GENICAM_CAMERA_NODELET_H
#define RC_GENICAM_CAMERA_NODELET_H

#include "imagelist.h"
#include "camerainfolist.h"
#include "publishers/camera_info_publisher.h"
#include "publishers/image_publisher.h"

#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/CameraInfo.h>

#include <GenApi/GenApi.h>
#include <rc_genicam_api/device.h>

#include <thread>
#include <mutex>
#include <atomic>

#include <rc_genicam_camera/GetGenICamParameter.h>
#include <rc_genicam_camera/SetGenICamParameter.h>

#include <diagnostic_updater/diagnostic_updater.h>

namespace rcgccam
{

class GenICamCameraNodelet : public nodelet::Nodelet
{

public:

  GenICamCameraNodelet();
  virtual ~GenICamCameraNodelet();

  virtual void onInit();

  bool getGenICamParameter(rc_genicam_camera::GetGenICamParameter::Request& req,
                           rc_genicam_camera::GetGenICamParameter::Response& resp);

  bool setGenICamParameter(rc_genicam_camera::SetGenICamParameter::Request& req,
                           rc_genicam_camera::SetGenICamParameter::Response& resp);

  void syncInfo(sensor_msgs::CameraInfoPtr info);

private:

  void grab(std::string device, rcg::Device::ACCESS access, std::string config);

  double timestamp_tolerance_;
  double sync_tolerance_;

  ros::Subscriber sub_sync_info_;

  ros::ServiceServer get_param_service_;
  ros::ServiceServer set_param_service_;

  std::string frame_id_;

  std::shared_ptr<rcg::Device> rcgdev_;
  std::shared_ptr<GenApi::CNodeMapRef> rcgnodemap_;
  std::mutex device_mtx_;

  ImageList image_list_;
  CameraInfoList info_list_;
  std::mutex sync_mtx_;

  CameraInfoPublisher caminfo_pub_;
  ImagePublisher image_pub_;
  std::string image_prefix_;

  std::thread grab_thread_;
  std::atomic_bool running_;
};

}  // namespace rcgccam

#endif
