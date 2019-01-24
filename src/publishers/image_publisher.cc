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

#include "image_publisher.h"

#include <rc_genicam_api/image.h>
#include <rc_genicam_api/pixel_formats.h>

#include <sensor_msgs/image_encodings.h>

namespace rcgccam
{

ImagePublisher::ImagePublisher()
{ }

void ImagePublisher::init(image_transport::ImageTransport& it)
{
  pub = it.advertise("image_raw", 1);
}

bool ImagePublisher::used()
{
  return pub.getNumSubscribers() > 0;
}

void ImagePublisher::publish(const sensor_msgs::ImagePtr &image)
{
  if (image && pub.getNumSubscribers() > 0)
  {
    pub.publish(image);
  }
}

sensor_msgs::ImagePtr rosImageFromBuffer(const std::string &frame_id, const rcg::Buffer* buffer,
                                         uint32_t part)
{
  sensor_msgs::ImagePtr im;

  uint64_t pixelformat = buffer->getPixelFormat(part);

  if (pixelformat == Mono8)
  {
    // create image and initialize header

    im = boost::make_shared<sensor_msgs::Image>();

    const uint64_t freq = 1000000000ul;
    uint64_t time = buffer->getTimestampNS();

    im->header.seq = 0;
    im->header.stamp.sec = time / freq;
    im->header.stamp.nsec = time - freq * im->header.stamp.sec;
    im->header.frame_id = frame_id;

    // set image size

    im->width = static_cast<uint32_t>(buffer->getWidth(part));
    im->height = static_cast<uint32_t>(buffer->getHeight(part));
    im->is_bigendian = false;

    // get pointer to image data in buffer

    const uint8_t* ps = static_cast<const uint8_t*>(buffer->getBase(part));
    size_t pstep = im->width + buffer->getXPadding(part);

    // convert image data

    if (pixelformat == Mono8)
    {
      im->encoding = sensor_msgs::image_encodings::MONO8;
      im->step = im->width * sizeof(uint8_t);

      im->data.resize(im->step * im->height);
      uint8_t* pt = reinterpret_cast<uint8_t*>(&im->data[0]);

      if (pixelformat == Mono8)  // copy monochrome image
      {
        for (uint32_t k = 0; k < im->height; k++)
        {
          for (uint32_t i = 0; i < im->width; i++)
          {
            *pt++ = ps[i];
          }

          ps += pstep;
        }
      }
    }
  }

  return im;
}

}
