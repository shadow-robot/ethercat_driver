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

#ifndef HAND_DRIVER_0220_H
#define HAND_DRIVER_0220_H

#include <ethercat_hardware/ethercat_device.h>
#include "dexterous_hand_driver/calibration.h"
#include "dexterous_hand_driver/external/0220_palm_edc/0220_palm_edc_ethercat_protocol.h"
#include "dexterous_hand_driver/sr_math_utils.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <array>
#include <map>
#include <memory>
#include <string>

#define HAND_DRIVER_0220_NB_JOINTS 24
#define HAND_DRIVER_0220_NB_RAW_SENSORS 37
#define HAND_DRIVER_0220_IMU_FIELDS 11
#define HAND_DRIVER_0220_NB_MOTORS 20
#define HAND_DRIVER_0220_NB_BIOTAC_ELECTRODES 24
#define HAND_DRIVER_0220_NB_FINGERS 5
// HAND_DRIVER_0220_NB_JOINTS plus combined joints (FFJ0, MFJ0, RFJ0, LFJ0).
#define HAND_DRIVER_0220_NB_ALL_JOINTS 28

using namespace sr_math_utils::filters;

namespace dexterous_hand_driver {

union CRCUnion {
  int16u word;
  int8u byte[2];
};

typedef CRCUnion union16;

struct BiotacData {
  int pac0;
  int pac1;
  int pdc;
  int tac;
  int tdc;
  std::array<int, HAND_DRIVER_0220_NB_BIOTAC_ELECTRODES> electrodes = {
      {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
       -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};
};

struct Hand0220Command {
  bool use_pwm;
  std::array<double, HAND_DRIVER_0220_NB_MOTORS> pwm_command = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<int, HAND_DRIVER_0220_NB_MOTORS> torque_command = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
};

struct Hand0220State {
  std::array<int, HAND_DRIVER_0220_NB_RAW_SENSORS> raw_position;
  std::array<double, HAND_DRIVER_0220_NB_ALL_JOINTS> calibrated_position;
  std::array<BiotacData, HAND_DRIVER_0220_NB_FINGERS> biotac_data;
  std::array<int, HAND_DRIVER_0220_NB_MOTORS> strain_gauges_data;
  double timestamp;  // In seconds.
};

class PositionPID {
 public:
  /*!
   * \brief Store gains in a struct to allow easier realtime buffer usage
   */
  struct Gains {
    Gains(const Gains &gains)
        : p_gain_(gains.p_gain_),
          i_gain_(gains.i_gain_),
          d_gain_(gains.d_gain_),
          i_max_(gains.i_max_),
          i_min_(gains.i_min_),
          antiwindup_(gains.antiwindup_) {}

    // Optional constructor for passing in values without antiwindup
    Gains(double p, double i, double d, double i_max, double i_min)
        : p_gain_(p),
          i_gain_(i),
          d_gain_(d),
          i_max_(i_max),
          i_min_(i_min),
          antiwindup_(false) {}
    // Optional constructor for passing in values
    Gains(double p, double i, double d, double i_max, double i_min,
          bool antiwindup)
        : p_gain_(p),
          i_gain_(i),
          d_gain_(d),
          i_max_(i_max),
          i_min_(i_min),
          antiwindup_(antiwindup) {}
    // Default constructor
    Gains()
        : p_gain_(0.0),
          i_gain_(0.0),
          d_gain_(0.0),
          i_max_(0.0),
          i_min_(0.0),
          antiwindup_(false) {}
    double p_gain_;   /**< Proportional gain. */
    double i_gain_;   /**< Integral gain. */
    double d_gain_;   /**< Derivative gain. */
    double i_max_;    /**< Maximum allowable integral term. */
    double i_min_;    /**< Minimum allowable integral term. */
    bool antiwindup_; /**< Antiwindup. */

    void DebugString() {
      printf("p_gain_ = %f\n", p_gain_);
      printf("i_gain_ = %f\n", i_gain_);
      printf("d_gain_ = %f\n", d_gain_);
      printf("i_max_ = %f\n", i_max_);
      printf("i_min_ = %f\n", i_min_);
    }
  };

  /*!
   * \brief Constructor, zeros out Pid values when created and
   *        initialize Pid-gains and integral term limits.
   *        Does not initialize dynamic reconfigure for PID gains
   *
   * \param p  The proportional gain.
   * \param i  The integral gain.
   * \param d  The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   */
  PositionPID(std::string joint_name, std::map<string, double> position_pid_map)
      : joint_name_(joint_name),
        gains_(position_pid_map["p"], position_pid_map["i"],
               position_pid_map["d"], position_pid_map["i_clamp"],
               -position_pid_map["i_clamp"]),
        max_force_(position_pid_map["max_force"]),
        deadband_(position_pid_map["deadband"]),
        position_deadband_(position_pid_map["position_deadband"]),
        friction_deadband_(position_pid_map["friction_deadband"]),
        p_error_last_(0.0),
        p_error_(0.0),
        i_error_(0.0),
        d_error_(0.0),
        cmd_(0.0){};

  PositionPID(const PositionPID &position_pid)
      : joint_name_(position_pid.joint_name_),
        gains_(position_pid.gains_),
        max_force_(position_pid.max_force_),
        deadband_(position_pid.deadband_),
        position_deadband_(position_pid.position_deadband_),
        friction_deadband_(position_pid.friction_deadband_),
        p_error_last_(position_pid.p_error_last_),
        p_error_(position_pid.p_error_),
        i_error_(position_pid.i_error_),
        d_error_(position_pid.d_error_),
        cmd_(position_pid.cmd_){};

  void DebugString() {
    printf("joint_name_ = %s\n", joint_name_.c_str());
    gains_.DebugString();
    printf("max_force = %f\n", max_force_);
    printf("deadband_ = %f\n", deadband_);
    printf("position_deaband_ = %f\n", position_deadband_);
    printf("friction_deaband_ = %f\n", friction_deadband_);
  }

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform
   * time step size. This also allows the user to pass in a precomputed
   * derivative error.
   *
   * \param error Error since last call (error = target - state)
   * \param error_dot d(Error)/dt since last call
   * \param dt Change in time in seconds since last call
   *
   * \returns PID command
   */
  double computeCommand(double error, double error_dot, double dt);

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform time
   * step size. The derivative error is computed from the change in the error
   * and the timestep \c dt.
   *
   * \param error  Error since last call (error = target - state)
   * \param dt Change in time in seconds since last call
   *
   * \returns PID command
   */
  double computeCommand(double error, double dt);

  bool in_deadband(double position, double command) {
    return abs(position - command) < position_deadband_;
  }

  bool in_deadband(double error) { return abs(error) < position_deadband_; }

 private:
  static double clampCommand(double command, double command_min = -1023.0,
                             double command_max = 1023.0) {
    return command < command_min
               ? command_min
               : (command > command_max ? command_max : command);
  }

  std::string joint_name_;
  Gains gains_;
  double max_force_;
  double deadband_;
  double position_deadband_;
  double friction_deadband_;
  double p_error_last_; /**< _Save position state for derivative state
                           calculation. */
  double p_error_;      /**< Position error. */
  double i_error_;      /**< Integral of position error. */
  double d_error_;      /**< Derivative of position error. */
  double cmd_;          /**< Command to send. */
};

struct JointState {
  JointState() : position_(0), velocity_(0) {}

  double position_;
  double velocity_;
};

struct PartialJointToSensor {
  PartialJointToSensor(int sensor_id, double coeff)
      : sensor_id_(sensor_id), coeff_(coeff) {}

  int sensor_id_;
  double coeff_;
};

struct JointToSensor {
  std::vector<std::string> sensor_names;
  std::vector<PartialJointToSensor> joint_to_sensor_vector;
  bool calibrate_after_combining_sensors;
};

// Data structure containing all information regarding a joint,
// including joint to motor ID mapping and joint PID config.
class MotorBoardEffortController {
 public:
  class PID {
   public:
    PID(const PID &pid)
        : d_(pid.d_),
          deadband_(pid.deadband_),
          f_(pid.f_),
          i_(pid.i_),
          imax_(pid.imax_),
          max_pwm_(pid.max_pwm_),
          p_(pid.p_),
          sg_left_(pid.sg_left_),
          sg_right_(pid.sg_right_),
          sgleftref_(pid.sgleftref_),
          sgrightref_(pid.sgrightref_),
          sign_(pid.sign_),
          torque_limit_(pid.torque_limit_),
          torque_limiter_gain_(pid.torque_limiter_gain_){};

    PID(std::map<std::string, int> &pid_map)
        : d_(pid_map["d"]),
          deadband_(pid_map["deadband"]),
          f_(pid_map["f"]),
          i_(pid_map["i"]),
          imax_(pid_map["imax"]),
          max_pwm_(pid_map["max_pwm"]),
          p_(pid_map["p"]),
          sg_left_(pid_map["sg_left"]),
          sg_right_(pid_map["sg_right"]),
          sgleftref_(pid_map["sgleftref"]),
          sgrightref_(pid_map["sgrightref"]),
          sign_(pid_map["sign"]),
          torque_limit_(pid_map["torque_limit"]),
          torque_limiter_gain_(pid_map["torque_limiter_gain"]){};

    PID(int d, int deadband, int f, int i, int imax, int max_pwm, int p,
        int sg_left, int sg_right, int sgleftref, int sgrightref, int sign,
        int torque_limit, int torque_limiter_gain)
        : d_(d),
          deadband_(deadband),
          f_(f),
          i_(i),
          imax_(imax),
          max_pwm_(max_pwm),
          p_(p),
          sg_left_(sg_left),
          sg_right_(sg_right),
          sgleftref_(sgleftref),
          sgrightref_(sgrightref),
          sign_(sign),
          torque_limit_(torque_limit),
          torque_limiter_gain_(torque_limiter_gain){};

    void DebugString() {
      printf("d_ = %d\n", d_);
      printf("f_ = %d\n", f_);
      printf("i_ = %d\n", i_);
      printf("imax_ = %d\n", imax_);
      printf("max_pwm_ = %d\n", max_pwm_);
      printf("p_ = %d\n", p_);
      printf("sg_left_ = %d\n", sg_left_);
      printf("sg_right_ = %d\n", sg_right_);
      printf("sgleftref_ = %d\n", sgleftref_);
      printf("sgrightref_ = %d\n", sgrightref_);
      printf("sign_ = %d\n", sign_);
      printf("torque_limit_ = %d\n", torque_limit_);
      printf("torque_limiter_gain_ = %d\n", torque_limiter_gain_);
    }

    int d_;
    int deadband_;
    int f_;
    int i_;
    int imax_;
    int max_pwm_;
    int p_;
    int sg_left_;
    int sg_right_;
    int sgleftref_;
    int sgrightref_;
    int sign_;
    int torque_limit_;
    int torque_limiter_gain_;
  };

  MotorBoardEffortController(int motor_id, std::string &joint_name,
                             bool backlash_compensation, PID pid,
                             LowPassFilter pos_filter,
                             const PositionPID &position_pid,
                             std::map<string, double> &sensors_map,
                             std::map<string, int> &sensor_name_to_id_map)
      : motor_id_(motor_id),
        joint_name_(joint_name),
        backlash_compensation_(backlash_compensation),
        pid_(pid),
        pos_filter_(pos_filter),
        position_pid_(std::make_shared<PositionPID>(position_pid)) {
    // Adds sensors corresponding to this joint to the data structure.
    for (auto &sensor : sensors_map) {
      joint_to_sensor_.sensor_names.push_back(sensor.first);
      int sensor_id = sensor_name_to_id_map[sensor.first.c_str()];
      joint_to_sensor_.joint_to_sensor_vector.push_back(
          PartialJointToSensor(sensor_id, sensor.second));
      if (sensor.second != 1.0) {
        joint_to_sensor_.calibrate_after_combining_sensors = true;
      } else {
        joint_to_sensor_.calibrate_after_combining_sensors = false;
      }
    }
  }

  void DebugString() {
    printf("motor_id_ = %d\n", motor_id_);
    printf("joint_name_ = %s\n", joint_name_.c_str());
    printf("backlash_compensation_ = %d\n", backlash_compensation_);
    pid_.DebugString();
    position_pid_->DebugString();
  }

  int motor_id_;
  std::string joint_name_;
  bool backlash_compensation_;
  PID pid_;
  LowPassFilter pos_filter_;
  JointToSensor joint_to_sensor_;
  JointState joint_state_;
  double command_;
  std::shared_ptr<PositionPID> position_pid_;
};

// From
// https://github.com/shadow-robot/sr-ros-interface-ethercat/sr_edc_launch/mappings/default_mappings/sensor_to_joint.yaml

static std::map<string, std::map<string, double>> joints_to_sensors_map = {
    {"FFJ0", {{"FFJ1", 1.0}, {"FFJ2", 1.0}}},
    {"FFJ1", {{"FFJ1", 1.0}}},
    {"FFJ2", {{"FFJ2", 1.0}}},
    {"FFJ3", {{"FFJ3", 1.0}}},
    {"FFJ4", {{"FFJ4", 1.0}}},
    {"MFJ0", {{"MFJ1", 1.0}, {"MFJ2", 1.0}}},
    {"MFJ1", {{"MFJ1", 1.0}}},
    {"MFJ2", {{"MFJ2", 1.0}}},
    {"MFJ3", {{"MFJ3", 1.0}}},
    {"MFJ4", {{"MFJ4", 1.0}}},
    {"RFJ0", {{"RFJ1", 1.0}, {"RFJ2", 1.0}}},
    {"RFJ1", {{"RFJ1", 1.0}}},
    {"RFJ2", {{"RFJ2", 1.0}}},
    {"RFJ3", {{"RFJ3", 1.0}}},
    {"RFJ4", {{"RFJ4", 1.0}}},
    {"LFJ0", {{"LFJ1", 1.0}, {"LFJ2", 1.0}}},
    {"LFJ1", {{"LFJ1", 1.0}}},
    {"LFJ2", {{"LFJ2", 1.0}}},
    {"LFJ3", {{"LFJ3", 1.0}}},
    {"LFJ4", {{"LFJ4", 1.0}}},
    {"LFJ5", {{"LFJ5", 1.0}}},
    {"THJ1", {{"THJ1", 1.0}}},
    {"THJ2", {{"THJ2", 1.0}}},
    {"THJ3", {{"THJ3", 1.0}}},
    {"THJ4", {{"THJ4", 1.0}}},
    {"THJ5", {{"THJ5A", 0.5}, {"THJ5B", 0.5}}},
    {"WRJ1", {{"WRJ1A", 0.5}, {"WRJ1B", 0.5}}},
    {"WRJ2", {{"WRJ2", 1.0}}},
};

// Enum for processing yaml file.
enum EXPECT_TYPE {
  EXPECT_KEY_TOKEN = 0,
  EXPECT_VALUE_TOKEN = 1,
  EXPECT_KEY_VALUE = 2,
  EXPECT_VALUE_VALUE = 3,
};

// Enum for transitioning from initialization state to normal operating
// state.
enum ACTION_STATE {
  SEND_RESET = 0,
  WAIT_FOR_RESET_COMPLETE = 1,
  SEND_PIDS = 2,
  SEND_BACKLASH = 3,
  SEND_NORMAL_COMMAND = 4,
};

class HandDriver0220 : public EthercatDevice {
 public:
  HandDriver0220() = default;

  // Initializes the internal data structures.
  void initializeStruct(const string &yaml_filename,
                        const string &position_controller_yaml_filename);

  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address,
                         unsigned int ethercat_command_data_size,
                         unsigned int ethercat_status_data_size,
                         unsigned int ethercat_can_bridge_data_size,
                         unsigned int ethercat_command_data_address,
                         unsigned int ethercat_status_data_address,
                         unsigned int ethercat_can_bridge_data_command_address,
                         unsigned int ethercat_can_bridge_data_status_address);

  virtual void packCommand(unsigned char *buffer);

  virtual bool unpackState(unsigned char *this_buffer,
                           unsigned char *prev_buffer);

  Hand0220Command *getCommandStruct();
  Hand0220State *getStateStruct();

  const std::map<string, int> &getJointsToMotorsMap();
  const std::map<int, string> &getMotorsToJointsMap();

  bool initialized();

  const std::map<string, int> &getJointToCalibratedIdMap() {
    return joint_name_to_calibrated_id_map_;
  }

  const std::map<int, string> &getCalibratedIdToJointMap() {
    return calibrated_id_to_joint_name_map_;
  }

  // Returns an array of indices in the calibrated positions for all the
  // controllable motors. This is to make it easy to correlate commanded
  // positions and observed states.
  std::array<int, HAND_DRIVER_0220_NB_MOTORS> *getCalibratedIds() {
    return &calibrated_ids_;
  }

 private:
  void buildMaps(std::map<string, int> &joints_to_motors_map,
                 std::map<int, string> &motors_to_joints_map);
  void buildJointCalibrationMap();

  void parseMotorBoardControllerYamlFile(std::string ymal_filename);
  void parsePositionControllerYamlFile(std::string ymal_filename);
  void buildFullConfigMap();
  void sendReset(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command);
  void sendPids(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command,
                int config_index, int motor_index);
  void sendSystemControls(
      ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command);
  void setFromMotorCommand(
      ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command,
      FROM_MOTOR_DATA_TYPE from_motor_data_type);
  void setTactileDataType(
      ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command,
      FROM_TACTILE_SENSOR_TYPE tactile_data_type);
  void setTactileDataTypes(
      ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command);
  void convertCommand(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command);
  int convertFromPositionToPwmCommand(MotorBoardEffortController &joint,
                                      double target_position, double period);

  bool convertState(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *state_data);
  double calibrateJoint(
      const MotorBoardEffortController &joint,
      ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *state_data,
      Hand0220State *high_level_state);
  void convertTactileData(
      ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *state_data,
      Hand0220State *high_level_state);
  void convertStrainGaugesData(
      ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *state_data,
      Hand0220State *high_level_state);
  // Command and state data pointers.
  int command_base_;
  int status_base_;
  std::string serial_number_;
  Hand0220Command high_level_command_;
  Hand0220State high_level_state_;
  bool initialized_ = false;

  // Various data structures for keeping track of joints, motors and sensors
  // mappings.
  std::map<string, int> joints_to_motors_map_;
  std::map<int, string> motors_to_joints_map_;
  std::map<string, int> sensor_name_to_id_map_;
  std::map<string, string> sensor_name_to_joint_name_map_;
  std::map<int, int> sensor_id_to_joint_id_map_;
  // Map from individual joint_name index map for outputing calibrated_positions
  // in high_level_state.
  std::map<string, int> joint_name_to_calibrated_id_map_;
  std::map<int, string> calibrated_id_to_joint_name_map_;
  std::array<int, HAND_DRIVER_0220_NB_MOTORS> calibrated_ids_;
  std::map<string, std::shared_ptr<JointCalibration>> joint_calibration_map_;

  // Vector containing all the programmable joints in the hand.
  std::vector<MotorBoardEffortController> joints_vector_;

  // Map from joint name to PositionPID.
  std::map<string, PositionPID> position_pid_map_;

  // Motor onfiguration map populated by parsing
  // motor_board_effort_controller.yaml.
  std::map<int, std::vector<union16>> full_config_map_;

  // State variable to controlling transitioning from initialization
  // state.
  enum ACTION_STATE action_state_ = SEND_RESET;

  // Internal states controlling what data to request.
  int which_motors_ = 0;
  int config_index_ = MOTOR_CONFIG_FIRST_VALUE;
  int motor_index_ = 0;
  int tactile_sensor_type_biotac_requested_ = TACTILE_SENSOR_TYPE_BIOTAC_PDC;

  struct timeval start_time_;
};
}  // namespace dexterous_hand_driver
#endif  // HAND_DRIVER_0220_H
