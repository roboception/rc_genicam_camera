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

#ifndef RC_GENICAM_CAMERA_TIMESTAMP_CORRECTOR_H
#define RC_GENICAM_CAMERA_TIMESTAMP_CORRECTOR_H

#include <ros/ros.h>

#include <rc_genicam_api/config.h>

namespace rcgccam
{

/**
  This class uses the GenICam command "TimestampLatch" and the parameter
  "Timestamp" to determine the offset between the system clock and the
  camera clock.
*/

class TimestampCorrector
{

public:

  TimestampCorrector();
  ~TimestampCorrector();

  /**
    Set the maximum tolerance for the offset between system and camera clock.

    NOTE: Setting this number negative disables correction.

    @param tol_ns Maximum tolerance in nano seconds, e.g. 10000000 (= 0.01 s)
  */

  void setMaximumTolerance(int64_t tol_ns);

  /**
    Set minimum time between determination of offset.

    @param interval_ns Interval time in nano seconds, e.g. 1000000000 (= 1 s)
  */

  void setInterval(int64_t interval_ns);

  /**
    Determine the offset between the system and camera clock. This method
    should be called regularly, e.g. every second or minute.

    This method does nothing if the last call to determineOffset() was
    successful and was called less than the interval time ago.

    @param  nodemap Nodemap of the camera.
    @return         False if determination of offset cannot be determined with
                    less than the maximum tolerance.
  */

  bool determineOffset(const std::shared_ptr<GenApi::CNodeMapRef> &nodemap);

  /**
    Correct the given camera timestamp to system time.

    @param time Timestamp to correct.
    @return     Accuracy of correction in nano seconds, which must be at most
                the specified maximum tolerance. < 0 is returned in case
                determineOffset() has never been called or if it delivered an
                error on the last call.
  */

  int64_t correct(ros::Time &time);

private:

  int64_t tolerance;
  int64_t interval;

  int64_t last;

  int64_t accuracy;
  int64_t offset;
};

}

#endif
