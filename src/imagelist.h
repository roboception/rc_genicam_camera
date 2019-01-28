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

#ifndef RC_GENICAM_CAMERA_IMAGELIST
#define RC_GENICAM_CAMERA_IMAGELIST

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace rcgccam
{
class ImageList
{
public:
  /**
    Create an image list.
  */
  ImageList();

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
    Adds the given image to the internal list. If the maximum number of
    elements is exceeded, then the oldest image will be dropped.

    @param image Image to be added.
    @return      Dropped image, null pointer if no image is dropped.
  */
  sensor_msgs::ImagePtr add(const sensor_msgs::ImagePtr& image);

  /**
    Remove all images that have a timestamp that is older or equal than the
    given timestamp.

    @param timestamp Timestamp.
    @return          Number of removed images.
  */
  int removeOld(const ros::Time& timestamp);

  /**
    Returns the oldest image that has a timestamp within the tolerance of the
    given timestamp. If the image cannot be found, then a nullptr is
    returned.

    @param timestamp Timestamp.
    @param tolerance Maximum tolarance added or subtracted to the timestamp.
    @return Pointer to image or 0.
  */
  sensor_msgs::ImagePtr find(const ros::Time& timestamp) const;

private:
  size_t maxsize_;
  uint64_t tolerance_;
  std::vector<sensor_msgs::ImagePtr> list_;
};

}  // namespace rcgccam

#endif
