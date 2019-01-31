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

#include "timestamp_corrector.h"

#include <time.h>

namespace rcgccam
{
TimestampCorrector::TimestampCorrector()
{
  tolerance_ = 2 * 10000000ll;
  interval_ = 1000000000ll;

  last_ = 0;

  accuracy_ = -1;
  offset_ = 0;
}

TimestampCorrector::~TimestampCorrector()
{
}

void TimestampCorrector::setMaximumTolerance(int64_t tol_ns)
{
  tolerance_ = 2 * tol_ns;
}

void TimestampCorrector::setInterval(int64_t interval_ns)
{
  interval_ = interval_ns;
}

namespace
{
inline int64_t getClock(clockid_t id)
{
  struct timespec ts;
  clock_gettime(id, &ts);
  return ts.tv_sec * 1000000000ll + ts.tv_nsec;
}

}  // namespace

bool TimestampCorrector::determineOffset(const std::shared_ptr<GenApi::CNodeMapRef>& nodemap)
{
  // do nothing if tolerance is negative

  if (tolerance_ < 0)
  {
    return true;
  }

  // do nothing if last successful call is not long ago

  int64_t now_ns = getClock(CLOCK_MONOTONIC);

  if (accuracy_ > 0 && now_ns - last_ <= interval_)
  {
    return true;
  }

  // determine offset of host and camera clock

  last_ = now_ns;

  int64_t before_ns = 0;
  int64_t after_ns = 0;

  accuracy_ = tolerance_ + 1;

  int n = 3;
  while (n > 0 && accuracy_ > tolerance_)
  {
    before_ns = getClock(CLOCK_REALTIME);
    rcg::callCommand(nodemap, "TimestampLatch", true);
    after_ns = getClock(CLOCK_REALTIME);

    accuracy_ = after_ns - before_ns;

    n--;
  }

  if (accuracy_ <= tolerance_)
  {
    int64_t ts = rcg::getInteger(nodemap, "TimestampLatchValue");

    if (ts == 0)
    {
      ts = rgc::getInteger(nodemap, "Timestamp"); // fallback for Matrix Vision USB3 cameras
    }

    offset_ = before_ns + (accuracy_ >> 1) - ts;

    return true;
  }

  accuracy_ = -1;
  offset_ = 0;

  return false;
}

int64_t TimestampCorrector::correct(ros::Time& time)
{
  if (tolerance_ >= 0 && accuracy_ >= 0)
  {
    int64_t t = static_cast<int64_t>(time.toNSec());
    time.fromNSec(static_cast<uint64_t>(t + offset_));

    return accuracy_;
  }

  return -1;
}

}  // namespace rcgccam
