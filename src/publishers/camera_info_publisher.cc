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

#include "camera_info_publisher.h"

#include <string>
#include <map>
#include <stdexcept>

#include <iostream>
#include <fstream>

namespace rcgccam
{

namespace
{

void trim(std::string &s)
{
  size_t pos;

  pos=0;

  while (pos < s.size() && isspace(s[pos]))
  {
    pos++;
  }

  if (pos > 0)
  {
    s=s.substr(pos);
  }

  pos=s.size();

  while (pos > 0 && isspace(s[pos-1]))
  {
    pos--;
  }

  if (pos < s.size())
  {
    s=s.substr(0, pos);
  }
}


// load key/value pairs from an ascii file into a map

bool loadFile(std::map<std::string, std::string> &data, const char *name)
{
  std::ifstream in;
  std::string   line;

  in.open(name);

  if (!in.good())
  {
    return false;
  }

  while (in.good())
  {
    getline(in, line);

    trim(line);

    size_t pos=line.find('#');

    if (pos != line.npos)
    {
      line=line.substr(0, pos);
    }

    if (line.size() > 0)
    {
      pos=line.find('=');

      if (pos != line.npos)
      {
        std::string key=line.substr(0, pos);
        std::string value=line.substr(pos+1);

        trim(key);
        trim(value);

        data.insert(std::pair<std::string,std::string>(key, value));
      }
      else
      {
        return false;
      }
    }
  }

  in.close();

  return true;
}

template <class T> void getValue(const std::map<std::string, std::string> &data,
  const char *key, T &value, const char *defvalue)
{
  std::map<std::string, std::string>::const_iterator it=data.find(key);
  std::string v;

  if (it != data.end())
  {
    v=it->second;
  }
  else
  {
    v=defvalue;
  }

  std::istringstream in(v);
  in >> value;
}

bool getMatrix33(const std::map<std::string, std::string> &data,
  const char *key, boost::array<double, 9> &values)
{
  std::map<std::string, std::string>::const_iterator it=data.find(key);
  std::string v;

  if (it != data.end())
  {
    v=it->second;
  }
  else
  {
    v="[1 0 0; 0 1 0; 0 0 1]";
  }

  std::istringstream in(v);

  char c;
  in >> c;

  if (c == '[')
  {
    int j=0;
    for (int k=0; k<3 && in; k++)
    {
      for (int i=0; i<3 && in; i++)
      {
        in >> values[j++];
      }

      in >> c;

      if (k+1 < 3 && c != ';')
      {
        return false;
      }
    }

    if (c != ']')
    {
      return false;
    }
  }
  else
  {
    return false;
  }

  return true;
}

}

CameraInfoPublisher::CameraInfoPublisher()
{ }

void CameraInfoPublisher::init(ros::NodeHandle& nh, const char *calib_file)
{
  // prepare camera info message

  if (calib_file != 0 && calib_file[0] != '\0')
  {
    std::map<std::string, std::string> data;

    if (loadFile(data, calib_file))
    {
      getValue(data, "camera.width", info.width, "0");
      getValue(data, "camera.height", info.width, "0");

      if (!getMatrix33(data, "camera.A", info.K))
      {
        ROS_ERROR("Getting camera.A from calibration file failed");
        return;
      }

      double e1, e2, e3, e4;

      getValue(data, "camera.e1", e1, "0");
      getValue(data, "camera.e2", e2, "0");
      getValue(data, "camera.e3", e3, "0");
      getValue(data, "camera.e4", e4, "0");

      if (e1 != 0 || e2 != 0 || e3 != 0 || e4 != 0)
      {
        info.distortion_model="equidistant";
        info.D.resize(4);

        info.D[0]=e1;
        info.D[1]=e2;
        info.D[2]=e3;
        info.D[3]=e4;
      }
      else
      {
        double k1, k2, k3, p1, p2;

        getValue(data, "camera.k1", k1, "0");
        getValue(data, "camera.k2", k2, "0");
        getValue(data, "camera.k3", k3, "0");
        getValue(data, "camera.p1", p1, "0");
        getValue(data, "camera.p2", p2, "0");

        info.distortion_model="plumb_bob";
        info.D.resize(5);

        info.D[0]=k1;
        info.D[1]=k2;
        info.D[2]=k3;
        info.D[3]=p1;
        info.D[4]=p2;
      }

      info.R[0] = 1;
      info.R[1] = 0;
      info.R[2] = 0;
      info.R[3] = 0;
      info.R[4] = 1;
      info.R[5] = 0;
      info.R[6] = 0;
      info.R[7] = 0;
      info.R[8] = 1;

      info.P[0] = info.K[0];
      info.P[1] = info.K[1];
      info.P[2] = info.K[2];
      info.P[3] = 0;
      info.P[4] = info.K[3];
      info.P[5] = info.K[4];
      info.P[6] = info.K[5];
      info.P[7] = 0;
      info.P[8] = info.K[6];
      info.P[9] = info.K[7];
      info.P[10] = info.K[8];
      info.P[11] = 0;

      info.binning_x = 1;
      info.binning_y = 1;

      // advertise topic

      pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    }
  }
}

bool CameraInfoPublisher::used()
{
  return pub.getNumSubscribers() > 0;
}

void CameraInfoPublisher::publish(const sensor_msgs::ImagePtr &image)
{
  if (image && pub.getNumSubscribers() > 0)
  {
    info.header = image->header;
    info.width = image->width;
    info.height = image->height;

    pub.publish(info);
  }
}

}
