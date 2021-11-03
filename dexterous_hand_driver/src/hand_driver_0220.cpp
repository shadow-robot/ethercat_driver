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

#include <inttypes.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <yaml.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "dexterous_hand_driver/external/0220_palm_edc/0220_palm_edc_ethercat_protocol.h"
#include "dexterous_hand_driver/hand_driver_0220.h"
#include "dexterous_hand_driver/sr_math_utils.h"
#include "ethercat_hardware/log.h"

#define ETHERCAT_STATUS_DATA_SIZE \
  sizeof(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS)
#define ETHERCAT_COMMAND_DATA_SIZE \
  sizeof(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND)

#define ETHERCAT_CAN_BRIDGE_DATA_SIZE sizeof(ETHERCAT_CAN_BRIDGE_DATA)

#define ETHERCAT_COMMAND_DATA_ADDRESS PALM_0200_ETHERCAT_COMMAND_DATA_ADDRESS
#define ETHERCAT_STATUS_DATA_ADDRESS PALM_0200_ETHERCAT_STATUS_DATA_ADDRESS
#define ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS \
  PALM_0200_ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS
#define ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS \
  PALM_0200_ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS

using namespace sr_math_utils::filters;

namespace dexterous_hand_driver {

void HandDriver0220::initializeStruct(
    const string &yaml_filename,
    const string &position_controller_yaml_filename) {
  buildMaps(joints_to_motors_map_, motors_to_joints_map_);
  buildJointCalibrationMap();
  parsePositionControllerYamlFile(position_controller_yaml_filename);
  parseMotorBoardControllerYamlFile(yaml_filename);
  buildFullConfigMap();
}

void HandDriver0220::buildMaps(std::map<string, int> &joints_to_motors_map,
                               std::map<int, string> &motors_to_joints_map) {
  for (int i = 0; i < SENSORS_NUM_0X20; ++i) {
    sensor_name_to_id_map_[sensor_names[i]] = i;
  }
  for (int i = 0; i < JOINTS_NUM_0X20; ++i) {
    joints_to_motors_map[joint_names[i]] = motor_ids[i];
    motors_to_joints_map[motor_ids[i]] = joint_names[i];
    SHADOWHAND_DEBUG("motor_id[%d] = %s\n", motor_ids[i], joint_names[i]);
    for (auto sensor : joints_to_sensors_map[joint_names[i]]) {
      int sensor_id = sensor_name_to_id_map_[sensor.first.c_str()];
      if (motor_ids[i] != -1) {
        sensor_name_to_joint_name_map_.insert(
            std::pair<string, string>(sensor.first.c_str(), joint_names[i]));
        sensor_id_to_joint_id_map_.insert(std::pair<int, int>(sensor_id, i));
      }
    }
  }
  int i = 0;
  for (auto &jc : joints_to_sensors_map) {
    joint_name_to_calibrated_id_map_.insert(
        std::pair<string, int>(jc.first, i));
    int motor_id = joints_to_motors_map[jc.first];
    if (motor_id != -1) {
      calibrated_ids_[motor_id] = i;
    }
    calibrated_id_to_joint_name_map_.insert(
        std::pair<int, string>(i++, jc.first));
  }
}

void HandDriver0220::buildJointCalibrationMap() {
  for (auto &jc : joint_calibration_values_map) {
    joint_calibration_map_.insert(
        std::pair<string, std::shared_ptr<JointCalibration>>(
            jc.first, new JointCalibration(jc.second)));
  }
}

void HandDriver0220::parseMotorBoardControllerYamlFile(
    std::string yaml_filename) {
  FILE *fh = fopen(yaml_filename.c_str(), "r");
  yaml_parser_t parser;
  yaml_token_t token; /* new variable */
  enum EXPECT_TYPE expect_type = EXPECT_KEY_TOKEN;

  /* Initialize parser */
  if (!yaml_parser_initialize(&parser))
    fputs("Failed to initialize parser!\n", stderr);
  if (fh == NULL) fputs("Failed to open file!\n", stderr);

  /* Set input file */
  yaml_parser_set_input_file(&parser, fh);

  std::map<std::string, int> pid_map;
  std::string joint_name, l1_key, key, value;
  bool backlash_compensation;
  float tau;

  int key_depth = 0;
  do {
    yaml_parser_scan(&parser, &token);
    switch (token.type) {
      /* Stream start/end */
      case YAML_STREAM_START_TOKEN:
      case YAML_STREAM_END_TOKEN:
        break;
      case YAML_KEY_TOKEN:
        if (expect_type = EXPECT_KEY_TOKEN) {
          expect_type = EXPECT_KEY_VALUE;
        }
        break;
      case YAML_VALUE_TOKEN:
        if (expect_type == EXPECT_VALUE_TOKEN) {
          expect_type = EXPECT_VALUE_VALUE;
        }
        break;
      case YAML_BLOCK_SEQUENCE_START_TOKEN:
        // puts("<b>Start Block (Sequence)</b>"); break;
      case YAML_BLOCK_ENTRY_TOKEN:
        // puts("<b>Start Block (Entry)</b>");    break;
      case YAML_BLOCK_END_TOKEN:
        // puts("<b>End block</b>");
        key_depth--;
        if (key_depth == 0) {
          transform(joint_name.begin(), joint_name.end(), joint_name.begin(),
                    ::toupper);
          auto it = position_pid_map_.find(joint_name);
          if (it == position_pid_map_.end()) {
            SHADOWHAND_ERROR("Not position PID for %s\n", joint_name.c_str());
            break;
          }

          MotorBoardEffortController joint(
              joints_to_motors_map_[joint_name], joint_name,
              backlash_compensation, MotorBoardEffortController::PID(pid_map),
              LowPassFilter(tau), it->second, joints_to_sensors_map[joint_name],
              sensor_name_to_id_map_);
          joints_vector_.emplace_back(joint);
        }
        break;
      case YAML_BLOCK_MAPPING_START_TOKEN:
        // puts("[Block mapping]");
        if (expect_type == EXPECT_VALUE_VALUE) {
          l1_key = key;
          key_depth++;
          expect_type = EXPECT_KEY_TOKEN;
        }
        break;
      case YAML_SCALAR_TOKEN:
        if (key_depth == 0) {
          joint_name.assign(
              static_cast<const char *>((char *)(token.data.scalar.value)));
        }
        if (expect_type == EXPECT_VALUE_VALUE) {
          value.assign(
              static_cast<const char *>((char *)(token.data.scalar.value)));
          expect_type = EXPECT_KEY_TOKEN;
          if (key_depth == 1) {
            if (key == "backlash_compensation") {
              if (value == "true") {
                backlash_compensation = true;
              } else {
                backlash_compensation = false;
              }
            }
          } else {
            if (l1_key == "pid") {
              pid_map[key] = std::stoi(value);
            } else if (l1_key == "pos_filter" && key == "tau") {
              tau = stof(value);
            }
          }
        } else {
          key.assign(
              static_cast<const char *>((char *)(token.data.scalar.value)));
          expect_type = EXPECT_VALUE_TOKEN;
        }
        break;
      default:
        printf("Got token of type %d\n", token.type);
    }
    if (token.type != YAML_STREAM_END_TOKEN) yaml_token_delete(&token);
  } while (token.type != YAML_STREAM_END_TOKEN);

  /* Cleanup */
  yaml_parser_delete(&parser);
  fclose(fh);
}

void HandDriver0220::parsePositionControllerYamlFile(
    std::string yaml_filename) {
  FILE *fh = fopen(yaml_filename.c_str(), "r");
  yaml_parser_t parser;
  yaml_token_t token; /* new variable */
  enum EXPECT_TYPE expect_type = EXPECT_KEY_TOKEN;

  /* Initialize parser */
  if (!yaml_parser_initialize(&parser))
    fputs("Failed to initialize parser!\n", stderr);
  if (fh == NULL) fputs("Failed to open file!\n", stderr);

  /* Set input file */
  yaml_parser_set_input_file(&parser, fh);

  std::map<std::string, double> pid_map;
  std::string joint_name, l1_key, key, value;

  int key_depth = 0;
  do {
    yaml_parser_scan(&parser, &token);
    switch (token.type) {
      /* Stream start/end */
      case YAML_STREAM_START_TOKEN:
      case YAML_STREAM_END_TOKEN:
        break;
      case YAML_KEY_TOKEN:
        if (expect_type = EXPECT_KEY_TOKEN) {
          expect_type = EXPECT_KEY_VALUE;
        }
        break;
      case YAML_VALUE_TOKEN:
        if (expect_type == EXPECT_VALUE_TOKEN) {
          expect_type = EXPECT_VALUE_VALUE;
        }
        break;
      case YAML_BLOCK_SEQUENCE_START_TOKEN:
        // puts("<b>Start Block (Sequence)</b>"); break;
      case YAML_BLOCK_ENTRY_TOKEN:
        // puts("<b>Start Block (Entry)</b>");    break;
      case YAML_BLOCK_END_TOKEN:
        // puts("<b>End block</b>");
        key_depth--;
        if (key_depth == 0) {
          transform(joint_name.begin(), joint_name.end(), joint_name.begin(),
                    ::toupper);
          position_pid_map_.insert(std::pair<string, PositionPID>(
              joint_name.substr(6, 4),
              PositionPID(joint_name.substr(6, 4), pid_map)));
        }
        break;
      case YAML_BLOCK_MAPPING_START_TOKEN:
        // puts("[Block mapping]");
        if (expect_type == EXPECT_VALUE_VALUE) {
          l1_key = key;
          key_depth++;
          expect_type = EXPECT_KEY_TOKEN;
        }
        break;
      case YAML_SCALAR_TOKEN:
        if (key_depth == 0) {
          joint_name.assign(
              static_cast<const char *>((char *)(token.data.scalar.value)));
        }
        if (expect_type == EXPECT_VALUE_VALUE) {
          value.assign(
              static_cast<const char *>((char *)(token.data.scalar.value)));
          expect_type = EXPECT_KEY_TOKEN;
          if (key_depth != 1 && l1_key == "pid") {
            pid_map[key] = std::stof(value);
          }
        } else {
          key.assign(
              static_cast<const char *>((char *)(token.data.scalar.value)));
          expect_type = EXPECT_VALUE_TOKEN;
        }
        break;
      default:
        printf("Got token of type %d\n", token.type);
    }
    if (token.type != YAML_STREAM_END_TOKEN) yaml_token_delete(&token);
  } while (token.type != YAML_STREAM_END_TOKEN);

  /* Cleanup */
  yaml_parser_delete(&parser);
  fclose(fh);
}

void HandDriver0220::buildFullConfigMap() {
  for (auto &joint : joints_vector_) {
    // the vector is of the size of the TO_MOTOR_DATA_TYPE enum.
    // the value of the element at a given index is the value
    // for the given MOTOR_CONFIG.
    vector<union16> full_config(MOTOR_CONFIG_CRC + 1);
    union16 value;

    value.word = joint.pid_.max_pwm_;
    full_config.at(MOTOR_CONFIG_MAX_PWM) = value;

    value.byte[0] = joint.pid_.sg_left_;
    value.byte[1] = joint.pid_.sg_right_;
    full_config.at(MOTOR_CONFIG_SG_REFS) = value;

    value.word = joint.pid_.f_;
    full_config.at(MOTOR_CONFIG_F) = value;

    value.word = joint.pid_.p_;
    full_config.at(MOTOR_CONFIG_P) = value;

    value.word = joint.pid_.i_;
    full_config.at(MOTOR_CONFIG_I) = value;

    value.word = joint.pid_.d_;
    full_config.at(MOTOR_CONFIG_D) = value;

    value.word = joint.pid_.imax_;
    full_config.at(MOTOR_CONFIG_IMAX) = value;

    value.byte[0] = joint.pid_.deadband_;
    value.byte[1] = joint.pid_.sign_;
    full_config.at(MOTOR_CONFIG_DEADBAND_SIGN) = value;

    value.word = joint.pid_.torque_limit_;
    full_config.at(MOTOR_CONFIG_TORQUE_LIMIT) = value;

    value.word = joint.pid_.torque_limiter_gain_;
    full_config.at(MOTOR_CONFIG_TORQUE_LIMITER_GAIN) = value;

    // compute crc
    int16u crc_result = 0;
    int8u crc_byte;
    int8u crc_i;

    for (unsigned int i = MOTOR_CONFIG_FIRST_VALUE;
         i <= MOTOR_CONFIG_LAST_VALUE; ++i) {
      crc_byte = full_config.at(i).byte[0];
      INSERT_CRC_CALCULATION_HERE;

      crc_byte = full_config.at(i).byte[1];
      INSERT_CRC_CALCULATION_HERE;
    }

    // never send a CRC of 0, send 1 if the
    // computed CRC is 0 (0 is a code for
    // ignoring the config)
    if (crc_result == 0) {
      crc_result = 1;
    }
    value.word = crc_result;
    full_config.at(MOTOR_CONFIG_CRC) = value;
    full_config_map_[joint.motor_id_] = full_config;
  }
}

void HandDriver0220::construct(
    EtherCAT_SlaveHandler *sh, int &start_address,
    unsigned int ethercat_command_data_size,
    unsigned int ethercat_status_data_size,
    unsigned int ethercat_can_bridge_data_size,
    unsigned int ethercat_command_data_address,
    unsigned int ethercat_status_data_address,
    unsigned int ethercat_can_bridge_data_command_address,
    unsigned int ethercat_can_bridge_data_status_address) {
  sh_ = sh;

  command_base_ = start_address;
  command_size_ = ethercat_command_data_size + ethercat_can_bridge_data_size;

  start_address += command_size_;

  status_base_ = start_address;
  status_size_ = ethercat_status_data_size + ethercat_can_bridge_data_size;

  start_address += status_size_;

  // ETHERCAT_COMMAND_DATA
  //
  // This is for data going TO the palm
  //
  SHADOWHAND_INFO(
      "First FMMU (command) : start_address : 0x%08X ; size : %3d bytes ; phy "
      "addr : 0x%08X",
      command_base_, command_size_,
      static_cast<int>(ethercat_command_data_address));
  EC_FMMU *commandFMMU = new EC_FMMU(
      command_base_,  // Logical Start Address    (in ROS address space?)
      command_size_,
      0x00,                           // Logical Start Bit
      0x07,                           // Logical End Bit
      ethercat_command_data_address,  // Physical Start Address(in ET1200
                                      // address space?)
      0x00,                           // Physical Start Bit
      false,                          // Read Enable
      true,                           // Write Enable
      true);                          // Channel Enable

  // ETHERCAT_STATUS_DATA
  //
  // This is for data coming FROM the palm
  //
  SHADOWHAND_INFO(
      "Second FMMU (status) : start_address : 0x%08X ; size : %3d bytes ; phy "
      "addr : 0x%08X",
      status_base_, status_size_,
      static_cast<int>(ethercat_status_data_address));
  EC_FMMU *statusFMMU =
      new EC_FMMU(status_base_, status_size_, 0x00, 0x07,
                  ethercat_status_data_address, 0x00, true, false, true);

  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);

  (*fmmu)[0] = *commandFMMU;
  (*fmmu)[1] = *statusFMMU;

  sh->set_fmmu_config(fmmu);

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(4);

  (*pd)[0] =
      EC_SyncMan(ethercat_command_data_address, ethercat_command_data_size,
                 EC_QUEUED, EC_WRITTEN_FROM_MASTER);
  (*pd)[1] = EC_SyncMan(ethercat_can_bridge_data_command_address,
                        ethercat_can_bridge_data_size, EC_QUEUED,
                        EC_WRITTEN_FROM_MASTER);
  (*pd)[2] = EC_SyncMan(ethercat_status_data_address, ethercat_status_data_size,
                        EC_QUEUED);
  (*pd)[3] = EC_SyncMan(ethercat_can_bridge_data_status_address,
                        ethercat_can_bridge_data_size, EC_QUEUED);

  status_size_ = ethercat_status_data_size + ethercat_can_bridge_data_size;

  (*pd)[0].ChannelEnable = true;
  (*pd)[0].ALEventEnable = true;
  (*pd)[0].WriteEvent = true;

  (*pd)[1].ChannelEnable = true;
  (*pd)[1].ALEventEnable = true;
  (*pd)[1].WriteEvent = true;

  (*pd)[2].ChannelEnable = true;
  (*pd)[3].ChannelEnable = true;

  sh->set_pd_config(pd);

  SHADOWHAND_INFO("status_size_ : %d ; command_size_ : %d", status_size_,
                  command_size_);
}

void HandDriver0220::construct(EtherCAT_SlaveHandler *sh, int &start_address) {
  int i = PALM_0200_ETHERCAT_COMMAND_HEADER_SIZE;
  construct(sh, start_address, ETHERCAT_COMMAND_DATA_SIZE,
            ETHERCAT_STATUS_DATA_SIZE, ETHERCAT_CAN_BRIDGE_DATA_SIZE,
            ETHERCAT_COMMAND_DATA_ADDRESS, ETHERCAT_STATUS_DATA_ADDRESS,
            ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS,
            ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS);

  SHADOWHAND_INFO("Finished constructing the 0220 driver");
}

// Initializes the hand, including joint names and motor ids.
void HandDriver0220::sendReset(
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command) {
  // Sends reset.
  command->to_motor_data_type = MOTOR_SYSTEM_RESET;

  // Adds motor_id to the list of motors to reset.
  for (auto &joint : joints_vector_) {
    union16 to_send;
    to_send.byte[1] = MOTOR_SYSTEM_RESET_KEY >> 8;
    int motor_id = joint.motor_id_;
    SHADOWHAND_DEBUG("Sending reset to %d\n", motor_id);
    if (motor_id > 9) {
      to_send.byte[0] = motor_id - 10;
    } else {
      to_send.byte[0] = motor_id;
    }
    command->motor_data[motor_id] = to_send.word;
  }
}

// Resends PIDs.
void HandDriver0220::sendPids(
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command, int config_index,
    int motor_index) {
  setFromMotorCommand(command, MOTOR_DATA_TEMPERATURE);
  SHADOWHAND_DEBUG("Sending config %d of value %d to motor %d\n", config_index,
                   full_config_map_[motor_index][config_index].word,
                   motor_index);

  // Send command.
  command->to_motor_data_type = static_cast<TO_MOTOR_DATA_TYPE>(config_index);

  // set the data we want to send to the given motor
  command->motor_data[motor_index] =
      full_config_map_[motor_index][config_index].word;

  // We're now sending the CRC. We need to send the correct CRC to
  // the motor we updated, and CRC=0 to all the other motors in its
  // group (odd/even) to tell them to ignore the new
  // configuration.
  // Once the config has been transmitted, pop the element
  // and reset the config_index to the beginning of the
  // config values
  if (config_index == static_cast<int>(MOTOR_CONFIG_CRC)) {
    // loop on all the motors and send a CRC of 0
    // except for the motor we're reconfiguring
    for (int i = 0; i < joints_vector_.size(); ++i) {
      if (i != motor_index) {
        command->motor_data[i] = 0;
      }
    }
  }
}

void HandDriver0220::setFromMotorCommand(
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command,
    FROM_MOTOR_DATA_TYPE from_motor_data_type) {
  which_motors_++;
  which_motors_ %= 2;
  command->from_motor_data_type = from_motor_data_type;
  command->which_motors = which_motors_;
}

void HandDriver0220::setTactileDataTypes(
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command) {
  command->tactile_data_type = tactile_sensor_type_biotac_requested_;
  tactile_sensor_type_biotac_requested_++;
  if (tactile_sensor_type_biotac_requested_ ==
      FROM_TACTILE_SENSOR_TYPE_BIOTAC_NUM_VALUES) {
    tactile_sensor_type_biotac_requested_ = TACTILE_SENSOR_TYPE_BIOTAC_PDC;
  }
}

void HandDriver0220::setTactileDataType(
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command,
    FROM_TACTILE_SENSOR_TYPE tactile_data_type) {
  command->tactile_data_type = tactile_data_type;
}

void HandDriver0220::sendSystemControls(
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command) {
  Hand0220Command *high_level_command = getCommandStruct();
  command->EDC_command = EDC_COMMAND_SENSOR_DATA;
  command->to_motor_data_type = MOTOR_SYSTEM_CONTROLS;
  setFromMotorCommand(command, MOTOR_DATA_TEMPERATURE);

  for (auto &joint : joints_vector_) {
    command->motor_data[joint.motor_id_] = joint.backlash_compensation_;
  }
}

// Converts target position to PWM command.
int HandDriver0220::convertFromPositionToPwmCommand(
    MotorBoardEffortController &joint, double target_position, double period) {
  double command = target_position;
  double position = joint.joint_state_.position_;
  double velocity = joint.joint_state_.velocity_;
  double error_position = 0.0;
  double commanded_effort = 0.0;

  // command = clamp_command(command);

  // TODO(sherrym): Add hysteresis_deadband.
  // bool in_deadband = hysteresis_deadband.is_in_deadband(
  //     command, error_position, position_deadband);

  if (joint.position_pid_->in_deadband(position, command)) return 0.0;

  error_position = sr_math_utils::to_rad(position - command);
  commanded_effort =
      joint.position_pid_->computeCommand(-error_position, period);

#if 0
  if (!in_deadband) {
    commanded_effort += friction_compensator->friction_compensation(
        position, velocity, static_cast<int>(commanded_effort),
        friction_deadband);
  }
#endif

  SHADOWHAND_DEBUG("joint: %s, target_position = %f commanded_effort = %f\n",
                   joint.joint_name_.c_str(), target_position,
                   commanded_effort);
  return static_cast<int>(commanded_effort);
}

// Converts high_level_command to hand command.
void HandDriver0220::convertCommand(
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command) {
  double period = 1.0;
  Hand0220Command *high_level_command = getCommandStruct();
  command->EDC_command = EDC_COMMAND_SENSOR_DATA;
  setFromMotorCommand(command, MOTOR_DATA_PWM);
  setTactileDataTypes(command);
  if (high_level_command->use_pwm) {
    command->to_motor_data_type = MOTOR_DEMAND_PWM;
    for (auto &joint : joints_vector_) {
      command->motor_data[joint.motor_id_] = high_level_command->pwm_command[joint.motor_id_]; //convertFromPositionToPwmCommand(
          //joint, high_level_command->pwm_command[joint.motor_id_], period);
      SHADOWHAND_DEBUG("Setting pwm[%d] to %d\n", joint.motor_id_,
                       command->motor_data[joint.motor_id_]);
    }
  } else {
    command->to_motor_data_type = MOTOR_DEMAND_TORQUE;
    for (auto &joint : joints_vector_) {
      command->motor_data[joint.motor_id_] =
          high_level_command->torque_command[joint.motor_id_];
      // SHADOWHAND_DEBUG("Setting torque[%d] to %d\n",
      //        joint.motor_id_, command->motor_data[joint.motor_id_]);
    }
  }
}

void HandDriver0220::packCommand(unsigned char *buffer) {
  // We cast the buffer to the low level command structure so that we can fill
  // its fields directly in the transmission buffer
  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command =
      reinterpret_cast<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *>(buffer);

  // TODO(sherrym): Do we need to build CAN message?
  ETHERCAT_CAN_BRIDGE_DATA *message =
      reinterpret_cast<ETHERCAT_CAN_BRIDGE_DATA *>(buffer +
                                                   ETHERCAT_COMMAND_DATA_SIZE);

  command->EDC_command = EDC_COMMAND_SENSOR_DATA;

  switch (action_state_) {
    case SEND_RESET:
      action_state_ = WAIT_FOR_RESET_COMPLETE;
      sendReset(command);
      gettimeofday(&start_time_, NULL);
      break;
    case WAIT_FOR_RESET_COMPLETE:
      struct timeval cur_time;
      gettimeofday(&cur_time, NULL);
      if (cur_time.tv_sec - start_time_.tv_sec > 3) {
        action_state_ = SEND_PIDS;
      } else {
        sendSystemControls(command);
      }
      break;
    case SEND_PIDS:
      sendPids(command, config_index_, motor_index_);
      config_index_++;
      if (config_index_ > MOTOR_CONFIG_CRC) {
        config_index_ = MOTOR_CONFIG_FIRST_VALUE;
        motor_index_++;
        if (motor_index_ >= joints_vector_.size()) {
          action_state_ = SEND_BACKLASH;
          motor_index_ = 0;
        }
      }
      break;
    case SEND_BACKLASH:
      sendSystemControls(command);
      action_state_ = SEND_NORMAL_COMMAND;
      break;
    case SEND_NORMAL_COMMAND:
      convertCommand(command);
      initialized_ = true;
      break;
  }
}

bool HandDriver0220::unpackState(unsigned char *this_buffer,
                                 unsigned char *prev_buffer) {
  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *state_data =
      reinterpret_cast<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *>(
          this_buffer + command_size_);
  if (!initialized_) {
    // We don't set the values to the high_level_state_ structure during
    // initialization
  } else {
    // TODO here we should

    // Take the values in the structure "state_data" and write them to the
    // structure "high_level_state_"
    convertState(state_data);
  }
  return true;
}

double HandDriver0220::calibrateJoint(
    const MotorBoardEffortController &joint,
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *state_data,
    Hand0220State *high_level_state) {
  auto &joint_to_sensor = joint.joint_to_sensor_;
  double calibrated_position = 0.0;
  if (joint_to_sensor.calibrate_after_combining_sensors) {
    double raw_position = 0.0;
    for (auto &sensor : joint_to_sensor.joint_to_sensor_vector) {
      raw_position += state_data->sensors[sensor.sensor_id_] * sensor.coeff_;
    }

    auto it = joint_calibration_map_.find(joint.joint_name_);
    if (it == joint_calibration_map_.end()) {
      SHADOWHAND_ERROR("Cannot find %s in calibration map\n",
                       joint.joint_name_.c_str());
    } else {
      int joint_id = joint_name_to_calibrated_id_map_[joint.joint_name_];
      high_level_state->calibrated_position[joint_id] =
          it->second->compute(raw_position);
      calibrated_position = high_level_state->calibrated_position[joint_id];
    }
  } else {
    for (int i = 0; i < joint_to_sensor.joint_to_sensor_vector.size(); ++i) {
      int sensor_id = joint_to_sensor.joint_to_sensor_vector[i].sensor_id_;
      int joint_id =
          joint_name_to_calibrated_id_map_[joint_to_sensor.sensor_names[i]];
      auto it = joint_calibration_map_.find(joint_to_sensor.sensor_names[i]);
      if (it == joint_calibration_map_.end()) {
        SHADOWHAND_ERROR("Cannot find %s in calibration map\n",
                         joint_to_sensor.sensor_names[i].c_str());
      } else {
        double raw_position = state_data->sensors[sensor_id];
        high_level_state->calibrated_position[joint_id] =
            it->second->compute(raw_position);
        calibrated_position += high_level_state->calibrated_position[joint_id];
      }
    }
    // Add calibrated position for combined joint (such as FFJ0, MFJ0, RFJ0 and
    // LFJ0 as well.
    int joint_id = joint_name_to_calibrated_id_map_[joint.joint_name_];
    high_level_state->calibrated_position[joint_id] = calibrated_position;
  }

  return calibrated_position;
}

void HandDriver0220::convertTactileData(
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *state_data,
    Hand0220State *high_level_state) {
  SHADOWHAND_DEBUG("tactile type = %d\n", state_data->tactile_data_type);
  for (int i = 0; i < HAND_DRIVER_0220_NB_FINGERS; ++i) {
    if (state_data->tactile_data_valid & (1 << i)) {
      TACTILE_SENSOR_BIOTAC_DATA_CONTENTS *tactile_data =
          reinterpret_cast<TACTILE_SENSOR_BIOTAC_DATA_CONTENTS *>(
              &(state_data->tactile[i]));
      // Pac data is available for all tactile sensor data type.
      if (tactile_data->data_valid.Pac0 && tactile_data->data_valid.Pac1) {
        high_level_state->biotac_data[i].pac0 =
            static_cast<int>(tactile_data->Pac[0]);
        high_level_state->biotac_data[i].pac1 =
            static_cast<int>(tactile_data->Pac[1]);
      }
      // "other_sensor_0" corresponds to state_data->tactile_data_type.
      // "other_sensor_1" corresponds to state_data->tactile_data_type + 1.
      // As a result, we can skip updating if we already have the data from
      // the previous request.
      if ((state_data->tactile_data_type <
               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_24 &&
           state_data->tactile_data_type % 2) ||
          (state_data->tactile_data_type ==
           TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_23)) {
        if (tactile_data->data_valid.other_sensor_0 &&
            tactile_data->data_valid.other_sensor_1) {
          *((int *)(&high_level_state->biotac_data[i]) +
            state_data->tactile_data_type + 2) =
              static_cast<int>(tactile_data->other_sensor_0);
          *((int *)(&high_level_state->biotac_data[i]) +
            state_data->tactile_data_type + 3) =
              static_cast<int>(tactile_data->other_sensor_1);
        }
      }
    } else {
      SHADOWHAND_DEBUG("Tactile data for data type %d on finger %d is invalid.",
                       state_data->tactile_data_type, i);
    }
  }
}

void HandDriver0220::convertStrainGaugesData(
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *state_data,
    Hand0220State *high_level_state)
{
  // fill up high level state with even motors
  if (state_data->which_motors == 0)
  {
    // even motors
    for (int i = 0; i < 10; ++i) 
    {
      high_level_state->strain_gauges_data[i*2] = state_data->motor_data_packet[i].torque;
    }
  }
  else if (state_data->which_motors == 1)   // fill with odd motors
  {
    for (int i = 0; i < 10; ++i)
    {
      int index = i + (i+1);
      high_level_state->strain_gauges_data[index] = state_data->motor_data_packet[i].torque;
    }
  }
}

bool HandDriver0220::convertState(
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *state_data) {
  Hand0220State *high_level_state = getStateStruct();
  struct timeval cur_time;
  gettimeofday(&cur_time, NULL);
  high_level_state->timestamp = static_cast<double>(cur_time.tv_sec) +
                                static_cast<double>(cur_time.tv_usec) * 1.0e-6;

  SHADOWHAND_DEBUG("EDC_command = %d\n", state_data->EDC_command);
  for (int i = 0; i < HAND_DRIVER_0220_NB_RAW_SENSORS; ++i) {
    SHADOWHAND_DEBUG("sensor %s: sensors[%d] = %d\n", sensor_names[i], i,
                     state_data->sensors[i]);
    high_level_state->raw_position[i] = state_data->sensors[i];
    if (sensor_id_to_joint_id_map_.find(i) ==
        sensor_id_to_joint_id_map_.end()) {
      SHADOWHAND_DEBUG("No corresponding joint for sensor %d\n", i);
      continue;
    }
  }

  // calibrateJoint() updates calibrated_position and position_ in the joint
  // state.
  for (int i = 0; i < HAND_DRIVER_0220_NB_MOTORS; ++i) {
    auto &joint = joints_vector_[i];
    double calibrated_position =
        calibrateJoint(joint, state_data, high_level_state);
    auto mutable_joint = &joints_vector_[i];
    // TODO(sherrym): Need to verify that having pos_filter is indeed better.
    pair<double, double> pos_and_velocity = mutable_joint->pos_filter_.compute(
      calibrated_position, high_level_state->timestamp);
    mutable_joint->joint_state_.position_ = pos_and_velocity.first;
    mutable_joint->joint_state_.velocity_ = pos_and_velocity.second;
  }

  // Converts tactile data to high_level_state->biotac_data.
  convertTactileData(state_data, high_level_state);

  SHADOWHAND_DEBUG("motor_data_type = %d\n", state_data->motor_data_type);
  SHADOWHAND_DEBUG("which_motors = %d\n", state_data->which_motors);
  SHADOWHAND_DEBUG("which_motor_data_arrived = 0x%x\n",
                   state_data->which_motor_data_arrived);
  SHADOWHAND_DEBUG("which_motor_data_had_errors = %d\n",
                   state_data->which_motor_data_had_errors);
  for (int i = 0; i < 10; ++i) 
  {
    SHADOWHAND_DEBUG("motor_data_packet[%d].torque = %d\n", i, state_data->motor_data_packet[i].torque);
    SHADOWHAND_DEBUG("motor_data_packet[%d].misc = %d\n", i, state_data->motor_data_packet[i].misc);
  }

  convertStrainGaugesData(state_data, high_level_state);

  return true;
}

Hand0220Command *HandDriver0220::getCommandStruct() {
  return &high_level_command_;
}

Hand0220State *HandDriver0220::getStateStruct() { return &high_level_state_; }

bool HandDriver0220::initialized() { return initialized_; }

const std::map<string, int> &HandDriver0220::getJointsToMotorsMap() {
  return joints_to_motors_map_;
}

const std::map<int, string> &HandDriver0220::getMotorsToJointsMap() {
  return motors_to_joints_map_;
}

double PositionPID::computeCommand(double error, double dt) {
  if (dt == 0 || std::isnan(error) || std::isinf(error)) return 0.0;

  double error_dot = d_error_;

  // Calculate the derivative error
  if (dt > 0) {
    error_dot = (error - p_error_last_) / dt;
    p_error_last_ = error;
  }

  return computeCommand(error, error_dot, dt);
}

double PositionPID::computeCommand(double error, double error_dot, double dt) {
  double p_term, d_term, i_term;
  p_error_ = error;  // this is error = target - state
  d_error_ = error_dot;

  if (dt == 0 || std::isnan(error) || std::isinf(error) ||
      std::isnan(error_dot) || std::isinf(error_dot))
    return 0.0;

  // Calculate proportional contribution to command
  p_term = gains_.p_gain_ * p_error_;

  // Calculate the integral of the position error
  i_error_ += dt * p_error_;

#if 0
  if(gains_.antiwindup_)
  {
    // Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
    i_error_ = boost::algorithm::clamp(i_error_,
                                       gains_.i_min_ / std::abs(gains_.i_gain_),
                                       gains_.i_max_ / std::abs(gains_.i_gain_));
  }
#endif

  // Calculate integral contribution to command
  i_term = gains_.i_gain_ * i_error_;

#if 0
  if(!gains_.antiwindup_)
  {
    // Limit i_term so that the limit is meaningful in the output
    i_term = boost::algorithm::clamp(i_term, gains_.i_min_, gains_.i_max_);
  }
#endif

  // Calculate derivative contribution to command
  d_term = gains_.d_gain_ * d_error_;
  SHADOWHAND_DEBUG("p_term = %f, d_term = %f, i_term = %f\n", p_term, d_term,
                   i_term);

  // Computes the commanded force.
  cmd_ = p_term + i_term + d_term;

  // Clamps the result to max_force.
  cmd_ = clampCommand(cmd_, -max_force_, max_force_);

  return cmd_;
}

}  // namespace dexterous_hand_driver
