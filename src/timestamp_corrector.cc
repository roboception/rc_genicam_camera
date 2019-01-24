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
  tolerance=2*10000000ll;
  interval=1000000000ll;

  last=0;

  accuracy=-1;
  offset=0;
}

TimestampCorrector::~TimestampCorrector()
{ }

void TimestampCorrector::setMaximumTolerance(int64_t tol_ns)
{
  tolerance=2*tol_ns;
}

void TimestampCorrector::setInterval(int64_t interval_ns)
{
  interval=interval_ns;
}

namespace
{

inline int64_t getClock(clockid_t id)
{
  struct timespec ts;
  clock_gettime(id, &ts);
  return ts.tv_sec*1000000000ll+ts.tv_nsec;
}

}

bool TimestampCorrector::determineOffset(const std::shared_ptr<GenApi::CNodeMapRef> &nodemap)
{
  // do nothing if tolerance is negative

  if (tolerance < 0)
  {
    return true;
  }

  // do nothing if last successful call is not long ago

  int64_t now_ns=getClock(CLOCK_MONOTONIC);

  if (accuracy > 0 && now_ns-last <= interval)
  {
    return true;
  }

  // determine offset of host and camera clock

  last=now_ns;

  int64_t before_ns=0;
  int64_t after_ns=0;

  accuracy=tolerance+1;

  int n=3;
  while (n > 0 && accuracy > tolerance)
  {
    before_ns=getClock(CLOCK_REALTIME);
    rcg::callCommand(nodemap, "TimestampLatch", true);
    after_ns=getClock(CLOCK_REALTIME);

    accuracy=after_ns-before_ns;

    n--;
  }

  if (accuracy <= tolerance)
  {
    offset=before_ns+(accuracy>>1)-rcg::getInteger(nodemap, "Timestamp");

    return true;
  }

  accuracy=-1;
  offset=0;

  return false;
}

int64_t TimestampCorrector::correct(ros::Time &time)
{
  if (tolerance >= 0 && accuracy >= 0)
  {
    int64_t t=static_cast<int64_t>(time.toNSec());
    time.fromNSec(static_cast<uint64_t>(t+offset));

    return accuracy;
  }

  return -1;
}

}
