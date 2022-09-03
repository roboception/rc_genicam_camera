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

#ifndef RCGCCAM_CAMERAINFOPUBLISHER_H
#define RCGCCAM_CAMERAINFOPUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>

namespace rcgccam
{
class CameraInfoPublisher
{
public:
  CameraInfoPublisher();

  /**
    Initialization of publisher.

    @param nh              Node handle.
    @param calib_file      Path and name of calibration file. The topic is not
                           advertised and nothing is published if the
                           calibration cannot be loaded from this file.
    @param id              Camera ID, i.e. < 0 for no ID, 0 for left and 1 for
                           right camera.
  */
  void init(rclcpp::Node::SharedPtr node, const char* calib_file, int id);

  bool used();

  void publish(const sensor_msgs::msg::Image::ConstSharedPtr& image);

private:
  CameraInfoPublisher(const CameraInfoPublisher&);             // forbidden
  CameraInfoPublisher& operator=(const CameraInfoPublisher&);  // forbidden

  sensor_msgs::msg::CameraInfo info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_ptr_;
};

}  // namespace rcgccam

#endif
