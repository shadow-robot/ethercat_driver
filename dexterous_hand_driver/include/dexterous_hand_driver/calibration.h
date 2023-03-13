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
         {{3102.0, 0.0},
          {2244.0, 22.5},
          {1458.0, 45.0},
          {741.0, 67.5},
          {262.0, 90.0}}},
        {"FFJ2",
         {{3338.0, 0.0},
          {3192.0, 22.5},
          {2726.0, 45.0},
          {2195.0, 67.5},
          {1652.0, 90.0}}},
        {"FFJ3",
         {{1408.0, 0.0},
          {1910.0, 22.5},
          {2400.0, 45.0},
          {2819.0, 67.5},
          {3160.0, 90.0}}},
        {"FFJ4",
         {{827.0, -20.0},
          {1530.0, -10.0},
          {1870.0, 0.0},
          {1968.0, 10.0},
          {2023.0, 20.0}}},
        {"MFJ1",
         {{3206.0, 0.0},
          {2418.0, 22.5},
          {1695.0, 45.0},
          {961.0, 67.5},
          {526.0, 90.0}}},
        {"MFJ2",
         {{3232.0, 0.0},
          {2968.0, 22.5},
          {2549.0, 45.0},
          {2143.0, 67.5},
          {1670.0, 90.0}}},
        {"MFJ3",
         {{1645.0, 0.0},
          {2108.0, 22.5},
          {2555.0, 45.0},
          {2929.0, 67.5},
          {3175.0, 90.0}}},
        {"MFJ4",
         {{3291.0, -20.0},
          {2655.0, -10.0},
          {2220.0, 0.0},
          {2076.0, 10.0},
          {2011.0, 20.0}}},
        {"RFJ1",
         {{3099.0, 0.0},
          {2342.0, 22.5},
          {1505.0, 45.0},
          {887.0, 67.5},
          {403.0, 90.0}}},
        {"RFJ2",
         {{3110.0, 0.0},
          {2897.0, 22.5},
          {2500.0, 45.0},
          {2063.0, 67.5},
          {1570.0, 90.0}}},
        {"RFJ3",
         {{1713.0, 0.0},
          {2155.0, 22.5},
          {2556.0, 45.0},
          {2882.0, 67.5},
          {3084.0, 90.0}}},
        {"RFJ4",
         {{2033.0, -20.0},
          {1964.0, -10.0},
          {1843.0, 0.0},
          {1469.0, 10.0},
          {940.0, 20.0}}},
        {"LFJ1",
         {{3262.0, 0.0},
          {2584.0, 22.5},
          {1752.0, 45.0},
          {1118.0, 67.5},
          {606.0, 90.0}}},
        {"LFJ2",
         {{3146.0, 0.0},
          {2950.0, 22.5},
          {2482.0, 45.0},
          {1966.0, 67.5},
          {1512.0, 90.0}}},
        {"LFJ3",
         {{1703.0, 0.0},
          {2207.0, 22.5},
          {2675.0, 45.0},
          {3030.0, 67.5},
          {3186.0, 90.0}}},
        {"LFJ4",
         {{2044.0, -20.0},
          {2000.0, -10.0},
          {1826.0, 0.0},
          {1543.0, 10.0},
          {1158.0, 20.0}}},
        {"LFJ5",
          {{2827.77, 0.0},
           {2301.02, 22.5},
           {1851.36, 45.0},
           {1314.56, 67.5}}},
        {"THJ1",
         {{2776.0, 0.0},
          {2384.0, 22.5},
          {1980.0, 45.0},
          {1484.0, 67.5},
          {1135.0, 90.0}}},
        {"THJ2",
         {{1649.0, -40.0},
          {1799.0, -20.0},
          {1994.0, 0.0},
          {2204.0, 20.0},
          {2386.0, 40.0}}},
        {"THJ3",
         {{1348.0, -15.0},
          {2353.0, 0.0},
          {2737.0, 15.0}}},
        {"THJ4",
         {{1647.0, 0.0},
          {1978.0, 22.5},
          {2323.0, 45.0},
          {2531.0, 67.5}}},
        {"THJ5",
         {{357.0, -60.0},
          {674.0, -45.0},
          {941.0, -30.0},
          {1277.0, -15.0},
          {1535.0, 0.0},
          {1847.0, 15.0},
          {2318.0, 30.0},
          {2634.0, 45.0},
          {2942.0, 60.0}}},
        {"WRJ1",
         {{2464.0, -45.0},
          {2361.0, -22.5},
          {2162.0, 0.0},
          {1996.0, 15.0},
          {1853.0, 30.0}}},
        {"WRJ2", {{2796.0, -30.0}, {1052.0, 0.0}, {326.0, 10.0}}},
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
