/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Shadow Robot Company Ltd. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/* Copyright 2018 Google LLC. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "dexterous_hand_driver/calibration.h"
#include "dexterous_hand_driver/sr_math_utils.h"

#include <algorithm>
#include <iostream>
#include <vector>

namespace dexterous_hand_driver {
JointCalibration::JointCalibration(std::vector<Point> calibration_table) {
  calibration_table_ = calibration_table;
  // calibration size - 1 because we use this number to access the last value in
  // the vector
  if (calibration_table.size())
    calibration_table_size_ = calibration_table.size() - 1;
  else
    calibration_table_size_ = 0;
  /*
   * make sure that the given calibration table is ordered by
   * growing values of the raw_value
   */
  std::sort(calibration_table_.begin(), calibration_table_.end(),
            sort_growing_raw_operator);
}

/**
 * Computes the calibrated joint position from the ADC raw reading.
 *
 * @param raw_reading the reading from the ADC
 *
 * @return the calibrated joint position in radians.
 */
double JointCalibration::compute(int raw_reading) {
  /**
   * the two points from the calibration which we'll use
   * to do the linear interpolation.
   */
  // That takes care of computing a reading that's before
  // the calibration table as well as a reading that's in the
  // first calibration table case.
  Point *low_point, *high_point;
  low_point = &calibration_table_[0];
  high_point = &calibration_table_[1];

  bool found = false;
  // if we have more than 2 points in our calibration table
  // or if the raw value isn't before the calibration table
  if (raw_reading > calibration_table_[0].raw_value) {
    if (calibration_table_size_ > 1) {
      for (unsigned int index_cal = 1; index_cal < calibration_table_size_;
           ++index_cal) {
        if ((raw_reading >= calibration_table_[index_cal - 1].raw_value) &&
            (raw_reading < calibration_table_[index_cal].raw_value)) {
          low_point = &calibration_table_[index_cal - 1];
          high_point = &calibration_table_[index_cal];

          found = true;
          break;
        }
      }  // end for

      // the point is outside of the table
      if (!found) {
        low_point = &calibration_table_[calibration_table_size_ - 1];
        high_point = &calibration_table_[calibration_table_size_];
      }
    }  // end if 2 values only in the table
  }    // end if raw_reading before table

  double calibrated_position = sr_math_utils::linear_interpolate_(
      raw_reading, low_point->raw_value, low_point->calibrated_value,
      high_point->raw_value, high_point->calibrated_value);
  return calibrated_position;
}
}  // namespace dexterous_hand_driver
