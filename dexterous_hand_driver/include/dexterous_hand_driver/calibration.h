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

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace dexterous_hand_driver {
/**
 * A point in the N-point piecewise
 * linear calibration
 */
struct Point {
  double raw_value;
  double calibrated_value;
};

/*
 * Calibration file downloaded from
 * https://github.com/shadow-robot/sr-config/blob/shadowrobot_180504/sr_ethercat_hand_config/calibrations/rh/calibration.yaml
 */
static std::map<std::string, std::vector<Point>> joint_calibration_values_map =
    {
        {"FFJ1",
         {{2999.0, 0.0},
          {2343.0, 22.5},
          {1509.0, 45.0},
          {785.0, 67.5},
          {311.0, 90.0}}},
        {"FFJ2",
         {{2863.0, 0.0},
          {2675.0, 22.5},
          {2327.0, 45.0},
          {1869.0, 67.5},
          {1329.0, 90.0}}},
        {"FFJ3",
         {{1630.0, 0.0},
          {2068.0, 22.5},
          {2502.0, 45.0},
          {2882.0, 67.5},
          {3141.0, 90.0}}},
        {"FFJ4",
         {{496.0, -20.0},
          {1349.0, -10.0},
          {1769.0, 0.0},
          {1952.0, 10.0},
          {2026.0, 20.0}}},
        {"MFJ1",
         {{3242.0, 0.0},
          {2473.0, 22.5},
          {1774.0, 45.0},
          {1114.0, 67.5},
          {492.0, 90.0}}},
        {"MFJ2",
         {{2811.0, 0.0},
          {2668.0, 22.5},
          {2396.0, 45.0},
          {2022.0, 67.5},
          {1583.0, 90.0}}},
        {"MFJ3",
         {{1660.0, 0.0},
          {2224.0, 22.5},
          {2734.0, 45.0},
          {3140.0, 67.5},
          {3391.0, 90.0}}},
        {"MFJ4",
         {{1230.0, -20.0},
          {1643.0, -10.0},
          {1892.0, 0.0},
          {1993.0, 10.0},
          {2047.0, 20.0}}},
        {"RFJ1",
         {{3433.0, 0.0},
          {3016.0, 22.5},
          {2299.0, 45.0},
          {1565.0, 67.5},
          {1003.0, 90.0}}},
        {"RFJ2",
         {{2899.0, 0.0},
          {2706.0, 22.5},
          {2375.0, 45.0},
          {1971.0, 67.5},
          {1569.0, 90.0}}},
        {"RFJ3",
         {{1620.0, 0.0},
          {2069.0, 22.5},
          {2519.0, 45.0},
          {2904.0, 67.5},
          {3122.0, 90.0}}},
        {"RFJ4",
         {{2061.0, -20.0},
          {2006.0, -10.0},
          {1865.0, 0.0},
          {1559.0, 10.0},
          {1037.0, 20.0}}},
        {"LFJ1",
         {{3150.0, 0.0},
          {2435.0, 22.5},
          {1549.0, 45.0},
          {817.0, 67.5},
          {52.0, 90.0}}},
        {"LFJ2",
         {{2904.0, 0.0},
          {2724.0, 22.5},
          {2364.0, 45.0},
          {1836.0, 67.5},
          {1437.0, 90.0}}},
        {"LFJ3",
         {{1703.0, 0.0},
          {2157.0, 22.5},
          {2617.0, 45.0},
          {3056.0, 67.5},
          {3315.0, 90.0}}},
        {"LFJ4",
         {{2055.0, -20.0},
          {2001.0, -10.0},
          {1859.0, 0.0},
          {1613.0, 10.0},
          {1100.0, 20.0}}},
        {"LFJ5", {{2529.0, 0.0}, {1791.0, 22.5}, {791.0, 45.0}}},
        {"THJ1",
         {{2515.0, 0.0},
          {2023.0, 22.5},
          {1527.0, 45.0},
          {1132.0, 67.5},
          {888.0, 90.0}}},
        {"THJ2",
         {{1629.0, -40.0},
          {1678.0, -20.0},
          {1792.0, 0.0},
          {1941.0, 20.0},
          {2098.0, 40.0}}},
        {"THJ3", {{1465.0, -15.0}, {2075.0, 0.0}, {2479.0, 15.0}}},
        {"THJ4",
         {{1601.0, 0.0}, {1906.0, 22.5}, {2195.0, 45.0}, {2460.0, 67.5}}},
        {"THJ5",
         {{531.0, -60.0},
          {1122.0, -30.0},
          {1761.0, 0.0},
          {2676.0, 30.0},
          {3188.0, 60.0}}},
        {"WRJ1",
         {{2207.0, -45.0},
          {1703.0, -22.5},
          {1263.0, 0.0},
          {1032.0, 15.0},
          {873.0, 30.0}}},
        {"WRJ2", {{2730.0, -30.0}, {1487.0, 0.0}, {990.0, 10.0}}},
};

/**
 * This function is used by the sort algorithm to compare
 * 2 different points and sort the calibration table by
 * growing values of raw_value.
 *
 * @param p1 the first point to compare
 * @param p2 the second point to compare
 *
 * @return true if p1.raw_value < p2.raw_value
 */
static bool sort_growing_raw_operator(const Point &p1, const Point &p2) {
  return p1.raw_value < p2.raw_value;
}

/**
 * This class is used to compute the calibrated joint position, given a raw
 * ADC sensor reading, using a N-point piecewise linear calibration table.
 */
class JointCalibration {
 public:
  explicit JointCalibration(std::vector<Point> calibration_table);

  /**
   * Computes the calibrated joint position from the ADC raw reading.
   *
   * @param raw_reading the reading from the ADC
   *
   * @return the calibrated joint position in radians.
   */
  double compute(int raw_reading);

  /**
   * Overload the << operator, for easier debugging.
   */
  friend std::ostream &operator<<(std::ostream &out,
                                  const JointCalibration &calib) {
    out << " calibration = {";
    out << "size: " << calib.calibration_table_size_;
    for (unsigned int i = 0; i < calib.calibration_table_.size(); ++i) {
      out << " [raw: " << calib.calibration_table_[i].raw_value;
      out << ", cal: " << calib.calibration_table_[i].calibrated_value << "]";
    }
    out << " }";
    return out;
  };

 private:
  /**
   * The calibration table. The vector is ordered by growing values
   * of the raw_data
   */
  std::vector<Point> calibration_table_;
  /**
   * The size of the calibration table, used for quick access
   */
  unsigned int calibration_table_size_;
};
}  // namespace dexterous_hand_driver

#endif  // CALIBRATION_H
