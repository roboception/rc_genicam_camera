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

#include "publishers/image_publisher.h"

#include <rc_genicam_api/image.h>
#include <rc_genicam_api/pixel_formats.h>

#include <sensor_msgs/image_encodings.hpp>

#ifdef __SSSE3__
#include <tmmintrin.h>
#endif

namespace rcgccam
{

ImagePublisher::ImagePublisher()
{ }

void ImagePublisher::init(image_transport::ImageTransport& it)
{
  pub_ = it.advertise("image_raw", 1);
}

bool ImagePublisher::used()
{
  return pub_.getNumSubscribers() > 0;
}

void ImagePublisher::publish(sensor_msgs::msg::Image::ConstSharedPtr image)
{
  if (image && pub_.getNumSubscribers() > 0)
  {
    pub_.publish(image);
  }
}

std::string rosPixelformat(int& bytes_per_pixel, uint64_t pixelformat)
{
  std::string ret;

  switch (pixelformat)
  {
    case Mono8:
      ret = sensor_msgs::image_encodings::MONO8;
      bytes_per_pixel = 1;
      break;

    case Mono16:
      ret = sensor_msgs::image_encodings::MONO16;
      bytes_per_pixel = 2;
      break;

    case BayerBG8:
      ret = sensor_msgs::image_encodings::BAYER_BGGR8;
      bytes_per_pixel = 1;
      break;

    case BayerBG16:
      ret = sensor_msgs::image_encodings::BAYER_BGGR16;
      bytes_per_pixel = 2;
      break;

    case BayerGB8:
      ret = sensor_msgs::image_encodings::BAYER_GBRG8;
      bytes_per_pixel = 1;
      break;

    case BayerGB16:
      ret = sensor_msgs::image_encodings::BAYER_GBRG16;
      bytes_per_pixel = 2;
      break;

    case BayerGR8:
      ret = sensor_msgs::image_encodings::BAYER_GRBG8;
      bytes_per_pixel = 1;
      break;

    case BayerGR16:
      ret = sensor_msgs::image_encodings::BAYER_GRBG16;
      bytes_per_pixel = 2;
      break;

    case BayerRG8:
      ret = sensor_msgs::image_encodings::BAYER_RGGB8;
      bytes_per_pixel = 1;
      break;

    case BayerRG16:
      ret = sensor_msgs::image_encodings::BAYER_RGGB16;
      bytes_per_pixel = 2;
      break;

    case RGB8:
      ret = sensor_msgs::image_encodings::RGB8;
      bytes_per_pixel = 3;
      break;

    case RGB16:
      ret = sensor_msgs::image_encodings::RGB16;
      bytes_per_pixel = 6;
      break;

    case RGBa8:
      ret = sensor_msgs::image_encodings::RGBA8;
      bytes_per_pixel = 4;
      break;

    case RGBa16:
      ret = sensor_msgs::image_encodings::RGBA16;
      bytes_per_pixel = 8;
      break;

    case BGRa8:
      ret = sensor_msgs::image_encodings::BGRA8;
      bytes_per_pixel = 4;
      break;

    case BGRa16:
      ret = sensor_msgs::image_encodings::BGRA16;
      bytes_per_pixel = 8;
      break;

    case YUV422_8:
      ret = sensor_msgs::image_encodings::YUV422;
      bytes_per_pixel = 2;
      break;

    default:
      ret = "";
      bytes_per_pixel = 0;
      break;
  }

  return ret;
}

namespace
{

/*
  Copies nmemb elements in inverse order. Each element consists of m bytes. The
  byte order within each element is not inversed.
*/

void copyInverse(uint8_t *tp, const uint8_t *sp, size_t nmemb, size_t m)
{
  if (m == 1)
  {
    // optimization of special case of inversing bytewise

    tp += nmemb;
    size_t i=0;

#if defined (__SSSE3__)
    __m128i inv_index=_mm_set_epi8(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);

    while (i+16 <= nmemb)
    {
      tp-=16;

      __m128i v=_mm_loadu_si128(reinterpret_cast<const __m128i *>(sp));
      v=_mm_shuffle_epi8(v, inv_index);
      _mm_storeu_si128(reinterpret_cast<__m128i *>(tp), v);

      sp+=16;
      i+=16;
    }
#endif

    while (i < nmemb)
    {
      tp--;
      *tp = *sp++;
      i++;
    }
  }
  else
  {
    // general case, i.e. m > 1

    tp += m*nmemb;

    for (size_t i=0; i<nmemb; i++)
    {
      tp -= m;

      for (size_t k=0; k<m; k++)
      {
        tp[k] = *sp++;
      }
    }
  }
}

}

sensor_msgs::msg::Image::SharedPtr rosImageFromBuffer(const std::string& frame_id, const rcg::Buffer* buffer,
  uint32_t part, bool rotate)
{
  sensor_msgs::msg::Image::SharedPtr im;
  std::string pixelformat;
  int bytes_per_pixel;

  pixelformat = rosPixelformat(bytes_per_pixel, buffer->getPixelFormat(part));

  // rotation is not supported for all pixel formats

  if (rotate && pixelformat == sensor_msgs::image_encodings::YUV422)
  {
    rotate=false;

    RCLCPP_WARN_STREAM(rclcpp::get_logger("rc_genicam_camera::image_publisher::rosImageFromBuffer"), 
    "Rotation is not supporte for image format: " << pixelformat);
  }

  if (pixelformat.size() > 0)
  {
    // create image and initialize header

    im = std::make_shared<sensor_msgs::msg::Image>();
    im->encoding = pixelformat;

    const uint64_t freq = 1000000000ul;
    uint64_t time = buffer->getTimestampNS();

    im->header.stamp.sec = time / freq;
    im->header.stamp.nanosec = time - freq * im->header.stamp.sec;
    im->header.frame_id = frame_id;

    // set image size

    im->width = static_cast<uint32_t>(buffer->getWidth(part));
    im->height = static_cast<uint32_t>(buffer->getHeight(part));
    im->is_bigendian = buffer->isBigEndian();

    // get pointer to image data in buffer

    const uint8_t* ps = static_cast<const uint8_t*>(buffer->getBase(part));
    size_t pstep = im->width * bytes_per_pixel + buffer->getXPadding(part);

    // copy image data

    im->step = im->width * bytes_per_pixel;

    im->data.resize(im->step * im->height);
    uint8_t* pt = reinterpret_cast<uint8_t*>(&im->data[0]);

    if (im->step != pstep)
    {
      if (rotate)
      {
        pt += im->step*im->height;
        for (uint32_t k = 0; k < im->height; k++)
        {
          pt -= im->step;

          copyInverse(pt, ps, im->width, bytes_per_pixel);

          ps += pstep;
        }
      }
      else
      {
        for (uint32_t k = 0; k < im->height; k++)
        {
          memcpy(pt, ps, im->step*sizeof(uint8_t));

          pt += im->step;
          ps += pstep;
        }
      }
    }
    else
    {
      if (rotate)
      {
        copyInverse(pt, ps, im->height*im->width, bytes_per_pixel);
      }
      else
      {
        memcpy(pt, ps, im->height*im->step*sizeof(uint8_t));
      }
    }
  }

  return im;
}

}  // namespace rcgccam
