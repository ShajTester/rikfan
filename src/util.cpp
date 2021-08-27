/**
 * Copyright 2017 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "util.hpp"

#include "pid.hpp"

#include <cstring>
#include <iostream>

void initializePIDStruct(ec::pid_info_t* info, const ec::pidinfo& initial)
{
    std::memset(info, 0x00, sizeof(ec::pid_info_t));

    info->ts = initial.ts;
    info->proportionalCoeff = initial.proportionalCoeff;
    info->integralCoeff = initial.integralCoeff;
    info->feedFwdOffset = initial.feedFwdOffset;
    info->feedFwdGain = initial.feedFwdGain;
    info->integralLimit.min = initial.integralLimit.min;
    info->integralLimit.max = initial.integralLimit.max;
    info->outLim.min = initial.outLim.min;
    info->outLim.max = initial.outLim.max;
    info->slewNeg = initial.slewNeg;
    info->slewPos = initial.slewPos;
    info->negativeHysteresis = initial.negativeHysteresis;
    info->positiveHysteresis = initial.positiveHysteresis;
}

void dumpPIDStruct(std::ostream &os, ec::pid_info_t* info)
{
    os << " ts: " << info->ts
              << "\n proportionalCoeff: " << info->proportionalCoeff
              << "\n integralCoeff: " << info->integralCoeff
              << "\n feedFwdOffset: " << info->feedFwdOffset
              << "\n feedFwdGain: " << info->feedFwdGain
              << "\n integralLimit.min: " << info->integralLimit.min
              << "\n integralLimit.max: " << info->integralLimit.max
              << "\n outLim.min: " << info->outLim.min
              << "\n outLim.max: " << info->outLim.max
              << "\n slewNeg: " << info->slewNeg << "\n slewPos: " << info->slewPos
              << "\n last_output: " << info->lastOutput
              << "\n integral: " << info->integral << std::endl;

    return;
}
