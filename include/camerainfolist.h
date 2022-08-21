/*
 * This file is part of the rc_genicam_camera package.
 *
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

#ifndef RC_GENICAM_CAMERA_CAMERAINFOLIST
#define RC_GENICAM_CAMERA_CAMERAINFOLIST

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace rcgccam
{
class CameraInfoList
{
public:
  /**
    Create a camera info list.
  */
  CameraInfoList();

  /**
    Set maximum size of the list.

    @param maxsize Maximum number of elements that the list can hold. The
                   default is 25.
  */
  void setSize(size_t maxsize);

  /**
    Set tolerance for finding corresponding timestamps.

    @param tolerance Tolerance in nano seconds. Default is 0.
  */
  void setTolerance(uint64_t tolerance);

  /**
    Adds the given camera info to the internal list. If the maximum number of
    elements is exceeded, then the oldest camera info will be dropped.

    @param info Camera info message to be added.
    @return     Dropped camera info message, null pointer if nothing is
                dropped.
  */
  sensor_msgs::msg::CameraInfo::ConstSharedPtr add(sensor_msgs::msg::CameraInfo::ConstSharedPtr info);

  /**
    Remove all camera infos that have a timestamp that is older or equal than
    the given timestamp.

    @param timestamp Timestamp.
    @return          Number of removed camera infos.
  */
  int removeOld(const rclcpp::Time& timestamp);

  /**
    Returns the oldest camera info that has a timestamp within the tolerance
    of the given timestamp. If the camera info cannot be found, then a
    nullptr is returned.

    @param timestamp Timestamp.
    @param tolerance Maximum tolarance added or subtracted to the timestamp.
    @return          Pointer to camera info or 0.
  */
  sensor_msgs::msg::CameraInfo::ConstSharedPtr find(const rclcpp::Time& timestamp) const;

private:
  size_t maxsize_;
  uint64_t tolerance_;
  std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr> list_;
};

}  // namespace rcgccam

#endif
