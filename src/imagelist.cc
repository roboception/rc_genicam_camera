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

#include "imagelist.h"

#include <algorithm>

namespace rcgccam
{
ImageList::ImageList()
{
  maxsize_ = 25;
}

void ImageList::setSize(size_t maxsize)
{
  maxsize_ = std::max(static_cast<size_t>(1), maxsize);
}

void ImageList::setTolerance(uint64_t tolerance)
{
  tolerance_ = tolerance;
}

sensor_msgs::msg::Image::SharedPtr ImageList::add(sensor_msgs::msg::Image::SharedPtr image)
{
  list_.push_back(image);

  sensor_msgs::msg::Image::SharedPtr ret;

  if (list_.size() > maxsize_)
  {
    ret = list_[0];
    list_.erase(list_.begin());
  }

  return ret;
}

int ImageList::removeOld(const rclcpp::Time& timestamp)
{
  size_t i = 0;
  int n = 0;

  while (i < list_.size())
  {
    if (rclcpp::Time(list_[i]->header.stamp) <= timestamp)
    {
      list_.erase(list_.begin() + static_cast<int>(i));
      n++;
    }
    else
    {
      i++;
    }
  }

  return n;
}

sensor_msgs::msg::Image::SharedPtr ImageList::find(const rclcpp::Time& timestamp) const
{
  for (size_t i = 0; i < list_.size(); i++)
  {
    uint64_t ts = timestamp.nanoseconds();
    uint64_t image_ts = list_[i]->header.stamp.nanosec;

    if (image_ts >= ts - tolerance_ && image_ts <= ts + tolerance_)
    {
      return list_[i];
    }
  }

  return sensor_msgs::msg::Image::SharedPtr();
}

}  // namespace rcgccam
