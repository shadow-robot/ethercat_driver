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

#include <getopt.h>
#include <stdio.h>
#include <unistd.h>

#include "dexterous_hand_driver/ethercat_hardware_hand_0220.h"

#define NB_IDLE_FRAMES 9

int main(int argc, char* argv[]) {
  int flags, opt;
  int motor_index = 0;
  double computed_command = 0;
  int loop_counter = 0;
  bool use_pwm = true;
  bool demo = true;
  string ether_interface = "enp5s0";
  string motor_board_effort_controller_file =
      "../src/motor_board_effort_controllers.yaml";
  string position_controller_file =
      "../src/sr_edc_joint_position_controllers_PWM.yaml";

  while ((opt = getopt(argc, argv, "dpe:i:v:f:")) != -1) {
    switch (opt) {
      case 'd':
        demo = true;
        break;
      case 'p':
        use_pwm = true;
        break;
      case 'e':
        ether_interface.assign(optarg);
        break;
      case 'i':
        motor_index = atoi(optarg);
        break;
      case 'f':
        motor_board_effort_controller_file.assign(optarg);
        break;
      case 'v':
        computed_command = atof(optarg);
        break;
      default:
        fprintf(stderr,
                "Usage: %s [-dp] [-i index] [-v value] [-e interface]\n",
                argv[0]);
        exit(EXIT_FAILURE);
    }
  }

  dexterous_hand_driver::EthercatHardwareHand0220 hand(ether_interface);

  // initializeHand0220 will do the etherCAT initialization of all the slaves
  // and then carry on with the initialization of their parameters (e.g all hand
  // params)
  hand.initializeHand0220(motor_board_effort_controller_file,
                          position_controller_file);

  // Get pointers to the command and state structures
  dexterous_hand_driver::EthercatHand0220Command* command =
      hand.getCommandStruct();
  dexterous_hand_driver::EthercatHand0220State* state = hand.getStateStruct();

  while (1) {
    loop_counter++;
    // Here "command" structure will contain the last command that was written
    // this should be OK, as it is equivalent to not sending a new etherCAT
    // frame (by default the motor controller board will keep applying the
    // latest received motor command if the communication is interrupted, until
    // a 50ms timeout is triggered, that will make the motor controller set the
    // command value to zero as a protection mechanism)
    hand.sendAndReceiveFromHand0220();
    // Here "state" structure contains the latest sensor data received from the
    // hand

    // Do something here with "state" data to compute the next command data
    // int first_raw_sensor = state->hand_1->raw_position[0]; // Whatever. e.g
    // read the value from the first raw sensor

    // Write your command data to structure "command"
    if (demo) {
      command->hand_1->use_pwm = true;
      // for (int i = 0; i < HAND_DRIVER_0220_NB_MOTORS; ++i)
      // {
      //   std::cout<<"Motor: "<<i<<" command: "<<command->hand_1->pwm_command[i]<<std::endl;
      // }
      //command->hand_1->pwm_command[0] = 0.0;
      command->hand_1->pwm_command[1] = -200.0;
    } //else {
    //   command->hand_1->use_pwm = use_pwm;
    //   if (motor_index == -1) {
    //     if (use_pwm) {
    //       for (int i = 0; i < HAND_DRIVER_0220_NB_MOTORS; ++i) {
    //         command->hand_1->pwm_command[i] = computed_command;
    //       }
    //     } else {
    //       for (int i = 0; i < HAND_DRIVER_0220_NB_MOTORS; ++i) {
    //         command->hand_1->torque_command[i] = computed_command;
    //       }
    //     }
    //   } else {
    //     if (use_pwm) {
    //       for (int i = 0; i < HAND_DRIVER_0220_NB_MOTORS; ++i) {
    //         if (i == motor_index) {
    //           command->hand_1->pwm_command[motor_index] = computed_command;
    //         } else {
    //           auto& mj_map = hand.getMotorsToJointsMap();
    //           auto mj = mj_map.find(i);
    //           int joint_id =
    //               hand.getJointToCalibratedIdMap().find(mj->second)->second;
    //           command->hand_1->pwm_command[i] = state->hand_1->calibrated_position[joint_id];
    //         }
    //       }
    //     } else {
    //       command->hand_1->torque_command[motor_index] = computed_command;
    //     }
    //   }
    // }

    // wait until 1ms has passed since last call to sendAndReceiveFromHand0220()
    usleep(1000);  // Don't do it like this (take the time of the system to
                   // account for last sending time instead)
    for (int i = 0; i < HAND_DRIVER_0220_NB_RAW_SENSORS; ++i) {
      printf("sensor[%d] %s = %d\n", i, sensor_names[i],
             state->hand_1->raw_position[i]);
    }
    for (int i = 0; i < HAND_DRIVER_0220_NB_ALL_JOINTS; ++i) {
      printf("joint[%d] %s calibrated_position = %f\n", i,
             hand.getCalibratedIdToJointMap().find(i)->second.c_str(),
             state->hand_1->calibrated_position[i]);
    }

    hand.sendAndReceiveFromHand0220();

    for (int i = 0; i < HAND_DRIVER_0220_NB_MOTORS; ++i)
    {
      std::cout<<"Motor: "<<i<<" command: "<<command->hand_1->pwm_command[i]<<std::endl;
    }
    // The following section is recommended to guarantee that the slow sensor
    // data fields are as fresh as they can be e.g. most of the biotac data
    // fields are polled only once every 14 frames so we should prefer not to
    // skip any frame and send one every 1ms even if we don't recompute the
    // motor command. Sending the latest motor command computed will have the
    // same effect as not sending a frame (see above)
    for (int i; i < NB_IDLE_FRAMES; i++) {
      usleep(1000);
      hand.sendAndReceiveFromHand0220();
    }
    usleep(1000);
  }
}
