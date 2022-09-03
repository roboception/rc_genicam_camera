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

#ifndef RCGCCAM_IMAGEPUBLISHER_H
#define RCGCCAM_IMAGEPUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rc_genicam_api/buffer.h>

#include <string>

namespace rcgccam
{
class ImagePublisher
{
public:
  ImagePublisher();

  /**
    Initialization of publisher.

    @param it              Image transport handle.
  */
  void init(image_transport::ImageTransport& id);

  bool used();

  void publish(sensor_msgs::msg::Image::ConstSharedPtr image);

private:
  ImagePublisher(const ImagePublisher&);             // forbidden
  ImagePublisher& operator=(const ImagePublisher&);  // forbidden

  image_transport::Publisher pub_;
};

/**
  Translates pixel format from GenICam to ROS. An empty string is returned, if
  lthe format is not supported.
*/
std::string rosPixelformat(int& bytes_per_pixel, uint64_t pixelformat);

/**
  Converts a (supported) image in a GenICam buffer into a ROS image.
*/
sensor_msgs::msg::Image::SharedPtr rosImageFromBuffer(const std::string& frame_id, const rcg::Buffer* buffer,
                                                      uint32_t part, bool rotate);

}  // namespace rcgccam

#endif
