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
#include <sstream>
#include <array>

namespace rcgccam
{
namespace
{

/*
  Remove leading and trailing spaces from string.
*/

void trim(std::string& s)
{
  size_t pos;

  pos = 0;

  while (pos < s.size() && isspace(s[pos]))
  {
    pos++;
  }

  if (pos > 0)
  {
    s = s.substr(pos);
  }

  pos = s.size();

  while (pos > 0 && isspace(s[pos - 1]))
  {
    pos--;
  }

  if (pos < s.size())
  {
    s = s.substr(0, pos);
  }
}

/*
  Load key/value pairs from an ascii file into a map.
*/

bool loadFile(std::map<std::string, std::string>& data, const char* name)
{
  std::ifstream in;
  std::string line;

  in.open(name);

  if (!in.good())
  {
    return false;
  }

  while (in.good())
  {
    getline(in, line);

    trim(line);

    size_t pos = line.find('#');

    if (pos != line.npos)
    {
      line = line.substr(0, pos);
    }

    if (line.size() > 0)
    {
      pos = line.find('=');

      if (pos != line.npos)
      {
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);

        trim(key);
        trim(value);

        data.insert(std::pair<std::string, std::string>(key, value));
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

/*
  Create a key like camera[.<id>].<name>
*/

inline std::string createKey(const char *name, int id)
{
  std::ostringstream out;

  out << "camera.";
  if (id >= 0) out << id << '.';
  out << name;

  return out.str();
}

/*
  Get the value with the given key from the map. Use the given default value if
  the key does not exist in the map.
*/

template <class T>
void getValue(const std::map<std::string, std::string>& data, const std::string &key, T& value, const char* defvalue)
{
  std::map<std::string, std::string>::const_iterator it = data.find(key.c_str());
  std::string v;

  if (it != data.end())
  {
    v = it->second;
  }
  else
  {
    v = defvalue;
  }

  std::istringstream in(v);
  in >> value;
}

/*
  Get a 3x3 matrix from the given key of the map. An identity matrix is
  returned if the key does not exist.
*/

bool getMatrix33(const std::map<std::string, std::string>& data, const std::string &key,
                 double M[3][3])
{
  std::map<std::string, std::string>::const_iterator it = data.find(key.c_str());
  std::string v;

  if (it != data.end())
  {
    v = it->second;
  }
  else
  {
    v = "[1 0 0; 0 1 0; 0 0 1]";
  }

  std::istringstream in(v);

  char c;
  in >> c;

  if (c == '[')
  {
    for (int k = 0; k < 3 && in; k++)
    {
      for (int i = 0; i < 3 && in; i++)
      {
        in >> M[k][i];
      }

      in >> c;

      if (k + 1 < 3 && c != ';')
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

/*
  Get a vector of size 3 from the given key of the map. A null vector is
  returned if the key does not exist.
*/

bool getVector3(const std::map<std::string, std::string>& data, const std::string &key,
                double A[3])
{
  std::map<std::string, std::string>::const_iterator it = data.find(key.c_str());
  std::string v;

  if (it != data.end())
  {
    v = it->second;
  }
  else
  {
    v = "[0 0 0]";
  }

  std::istringstream in(v);

  char c;
  in >> c;

  if (c == '[')
  {
    for (int i = 0; i < 3 && in; i++)
    {
      in >> A[i];
    }

    in >> c;

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

inline void mulMatrix33Matrix33(double ret[3][3], double A[3][3], double B[3][3])
{
  for (int k=0; k<3; k++)
  {
    for (int i=0; i<3; i++)
    {
      ret[k][i] = 0;

      for (int j=0; j<3; j++)
      {
        ret[k][i] += A[k][j] * B[j][i];
      }
    }
  }
}

inline void mulMatrix33Vector3(double ret[3], double M[3][3], double V[3])
{
  for (int k=0; k<3; k++)
  {
    ret[k] = 0;
    for (int i=0; i<3; i++)
    {
      ret[k] += M[k][i] * V[i];
    }
  }
}

inline void transposeMatrix33(double M[3][3])
{
  double v = M[0][1];
  M[0][1] = M[1][0];
  M[1][0] = v;

  v = M[0][2];
  M[0][2] = M[2][0];
  M[2][0] = v;

  v = M[1][2];
  M[1][2] = M[2][1];
  M[2][1] = v;
}

/*
  Store the given 3x3 matrix in a linear array.
*/

inline void storeMatrix(std::array<double, 9>& values, double M[3][3])
{
  int j = 0;
  for (int k = 0; k < 3; k++)
  {
    for (int i = 0; i < 3; i++)
    {
      values[j++] = M[k][i];
    }
  }
}

}  // namespace

CameraInfoPublisher::CameraInfoPublisher()
{
}

void CameraInfoPublisher::init(rclcpp::Node::SharedPtr node, const char* calib_file, int id)
{
  // advertise topic

  pub_ptr_ = node->create_publisher<sensor_msgs::msg::CameraInfo>(
    "camera_info",
    rclcpp::SensorDataQoS()
  );

  // check id

  if (id > 1)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Invalid camera ID, only < 0, 0 and 1 allowed: " << id);
    return;
  }

  // prepare camera info message

  if (calib_file != 0 && calib_file[0] != '\0')
  {
    std::map<std::string, std::string> data;

    if (loadFile(data, calib_file))
    {
      // get width and height

      getValue(data, createKey("width", id), info_.width, "0");
      getValue(data, createKey("height", id), info_.height, "0");

      // get camera matrix

      double A[3][3];
      if (!getMatrix33(data, createKey("A", id), A))
      {
        RCLCPP_ERROR(node->get_logger(), "Getting camera.A from calibration file failed");
        info_ = sensor_msgs::msg::CameraInfo();
        return;
      }

      storeMatrix(info_.k, A);

      // get lens distortion

      double e1, e2, e3, e4;

      getValue(data, createKey("e1", id), e1, "0");
      getValue(data, createKey("e2", id), e2, "0");
      getValue(data, createKey("e3", id), e3, "0");
      getValue(data, createKey("e4", id), e4, "0");

      if (e1 != 0 || e2 != 0 || e3 != 0 || e4 != 0)
      {
        info_.distortion_model = "equidistant";
        info_.d.resize(4);

        info_.d[0] = e1;
        info_.d[1] = e2;
        info_.d[2] = e3;
        info_.d[3] = e4;
      }
      else
      {
        double k1, k2, k3, p1, p2;

        getValue(data, createKey("k1", id), k1, "0");
        getValue(data, createKey("k2", id), k2, "0");
        getValue(data, createKey("k3", id), k3, "0");
        getValue(data, createKey("p1", id), p1, "0");
        getValue(data, createKey("p2", id), p2, "0");

        info_.distortion_model = "plumb_bob";
        info_.d.resize(5);

        info_.d[0] = k1;
        info_.d[1] = k2;
        info_.d[2] = p1;
        info_.d[3] = p2;
        info_.d[4] = k3;
      }

      // determine focal length after rectification

      double f = 0;

      getValue(data, "rect.f", f, "0");

      if (f == 0)
      {
        if (id >= 0)
        {
          double A0[3][3], A1[3][3];
          if (getMatrix33(data, createKey("A", 0), A0) && getMatrix33(data, createKey("A", 1), A1))
          {
            f = (A0[0][0] + A0[1][1] + A1[0][0] + A1[1][1])/4;
          }
          else
          {
            RCLCPP_ERROR(node->get_logger(), "Getting camera.A0 and camera.A1 from calibration file failed");
            info_ = sensor_msgs::msg::CameraInfo();
            return;
          }
        }
        else
        {
          f = (A[0][0] + A[1][1])/2;
        }
      }

      // rectification rotation (only relevant for stereo)

      double t = 0;

      if (id >= 0)
      {
        // get transformations of left and right camera

        double R0[3][3], R1[3][3];
        double T0[3], T1[3];

        getMatrix33(data, createKey("R", 0), R0);
        getMatrix33(data, createKey("R", 1), R1);
        getVector3(data, createKey("T", 0), T0);
        getVector3(data, createKey("T", 1), T1);

        // compute relative orientation between cameras

        double R[3][3];
        double T[3];

        transposeMatrix33(R1);
        mulMatrix33Matrix33(R, R1, R0);

        T1[0] -= T0[0];
        T1[1] -= T0[1];
        T1[2] -= T0[2];

        transposeMatrix33(R0);
        mulMatrix33Vector3(T, R0, T1);

        // get length of baseline

        t = std::sqrt(T[0] * T[0] + T[1] * T[1] + T[2] * T[2]);
        double l = std::sqrt(T[0] * T[0] + T[1] * T[1]);

        // compute rectification matrix of left camera

        double Rrect[3][3];

        Rrect[0][0] = T[0]/t;
        Rrect[1][0] = T[1]/t;
        Rrect[2][0] = T[2]/t;

        Rrect[0][1] = -T[1]/l;
        Rrect[1][1] = T[0]/l;
        Rrect[2][1] = 0;

        Rrect[0][2] = Rrect[1][0] * Rrect[2][1] - Rrect[2][0] * Rrect[1][1];
        Rrect[1][2] = Rrect[2][0] * Rrect[0][1] - Rrect[0][0] * Rrect[2][1];
        Rrect[2][2] = Rrect[0][0] * Rrect[1][1] - Rrect[1][0] * Rrect[0][1];

        if (id == 0)
        {
          transposeMatrix33(Rrect);
          storeMatrix(info_.r, Rrect);
        }
        else
        {
          double Rrect_right[3][3];
          mulMatrix33Matrix33(Rrect_right, R, Rrect);

          transposeMatrix33(Rrect_right);
          storeMatrix(info_.r, Rrect_right);
        }
      }
      else
      {
        // identity for mono camera

        info_.r[1] = info_.r[2] = info_.r[3] = 0;
        info_.r[0] = info_.r[4] = info_.r[8] = 1;
        info_.r[5] = info_.r[6] = info_.r[7] = 0;
      }

      // define projection matrix after rectification

      info_.p[0] = f;
      info_.p[1] = 0;
      info_.p[2] = info_.width/2.0;
      info_.p[3] = 0;
      info_.p[4] = 0;
      info_.p[5] = f;
      info_.p[6] = info_.height/2.0;
      info_.p[7] = 0;
      info_.p[8] = 0;
      info_.p[9] = 0;
      info_.p[10] = 1;
      info_.p[11] = 0;

      if (id == 1)
      {
        info_.p[3] = -f * t;
      }

      info_.binning_x = 1;
      info_.binning_y = 1;
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "gc_genicam_camera: Cannot load camera calibration: " << calib_file);
    }
  }
}

bool CameraInfoPublisher::used()
{
  return pub_ptr_->get_subscription_count() > 0;
}

void CameraInfoPublisher::publish(const sensor_msgs::msg::Image::ConstSharedPtr& image)
{
  if (image && pub_ptr_->get_subscription_count() > 0)
  {
    info_.header = image->header;

    if (info_.k[0] == 0)
    {
      info_.width = image->width;
      info_.height = image->height;
    }

    pub_ptr_->publish(info_);
  }
}

}  // namespace rcgccam
