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

#include "camerainfolist.h"

#include <algorithm>

namespace rcgccam
{
CameraInfoList::CameraInfoList()
{
  maxsize = 25;
}

void CameraInfoList::setSize(size_t _maxsize)
{
  maxsize = std::max(static_cast<size_t>(1), _maxsize);
}

void CameraInfoList::setTolerance(uint64_t _tolerance)
{
  tolerance = _tolerance;
}

sensor_msgs::CameraInfoPtr CameraInfoList::add(const sensor_msgs::CameraInfoPtr& image)
{
  list.push_back(image);

  sensor_msgs::CameraInfoPtr ret;

  if (list.size() > maxsize)
  {
    ret = list[0];
    list.erase(list.begin());
  }

  return ret;
}

int CameraInfoList::removeOld(const ros::Time& timestamp)
{
  size_t i = 0;
  int n = 0;

  while (i < list.size())
  {
    if (list[i]->header.stamp <= timestamp)
    {
      list.erase(list.begin() + static_cast<int>(i));
      n++;
    }
    else
    {
      i++;
    }
  }

  return n;
}

sensor_msgs::CameraInfoPtr CameraInfoList::find(const ros::Time& timestamp) const
{
  for (size_t i = 0; i < list.size(); i++)
  {
    uint64_t ts = timestamp.toNSec();
    uint64_t image_ts = list[i]->header.stamp.toNSec();

    if (image_ts >= ts - tolerance && image_ts <= ts + tolerance)
    {
      return list[i];
    }
  }

  return sensor_msgs::CameraInfoPtr();
}

}  // namespace rcgccam
