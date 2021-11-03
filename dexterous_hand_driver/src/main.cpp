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
#include <chrono>
#include "dexterous_hand_driver/ethercat_hardware_hand_0220.h"

#define NB_IDLE_FRAMES 9
#define YELLOW  "\033[33m"      /* Yellow */

static const int SEC_2_NSEC = 1e+9;


static void timespecInc(struct timespec &tick, int nsec)
{
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= SEC_2_NSEC)
  {
    tick.tv_nsec -= SEC_2_NSEC;
    ++tick.tv_sec;
  }
}

int main(int argc, char* argv[]) {
  int flags, opt;
  int motor_index = 0;
  bool use_pwm = true;
  bool execute_commands = false;
  int pwm_command = 0;
  bool read_hand_data = false;
  bool run_all = false;
  int counter = 0;
  double old_timestamp = 0;
  string ether_interface = "enp5s0";
  string motor_board_effort_controller_file =
      "../src/motor_board_effort_controllers.yaml";
  string position_controller_file =
      "../src/sr_edc_joint_position_controllers_PWM.yaml";

  while ((opt = getopt(argc, argv, "c:r:a:i:v:f:")) != -1) {
    switch (opt) {
      case 'c':
        execute_commands = true;
        break;
      case 'r':
        read_hand_data = true;
        break;
      case 'a':
        run_all = true;
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
      default:
        fprintf(stderr,
                "Usage: %s [-cra] [-i index] [-v value] [-e interface]\n",
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

  int policy;

  // Set to realtime scheduler for this thread
  struct sched_param thread_param;
  policy = SCHED_FIFO;
  thread_param.sched_priority = sched_get_priority_max(policy);
  pthread_setschedparam(pthread_self(), policy, &thread_param);
  struct timespec tick;

  while (1) {
    if (execute_commands == true)
    {
      cout << "Insert index of motor you want to control:\n";
      cout << "[FFJ0 - 0   | FFJ3 - 1  | FFJ4 - 2 ]\n";
      cout << "[MFJ0 - 3   | MFJ3 - 11 | MFJ4 - 13]\n";
      cout << "[RFJ0 - 15  | RFJ3 - 16 | RFJ4 - 17]\n";
      cout << "[LFJ0 - 12  | LFJ3 - 10 | LFJ4 - 14 | LFJ5 - 4]\n";
      cout << "[THJ1 - 6   | THJ2 - 5  | THJ3 - 7  | THJ4 - 9 | THJ5 - 19]\n";
      cout << "[WRJ1 - 18  | WRJ2 - 8]\n";
      cin >> motor_index;
      cout << "Insert pwm command to send to the motor (+ or - for direction, max 500): ";
      cin >> pwm_command;
      auto start = std::chrono::steady_clock::now();
      bool executing_pwm_command = true;

      while (executing_pwm_command == true)
      {
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(5))
        {
          executing_pwm_command = false;
          break;
        }
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
        cout<<"Sending: "<<pwm_command<<" to motor: "<<motor_index<<"...\n";
        command->hand_1->use_pwm = true;
        command->hand_1->pwm_command[motor_index] = pwm_command;

        hand.sendAndReceiveFromHand0220();
  
        usleep(1000);
      }
    }
    else if (read_hand_data == true)
    {
      std::string data_type;
      int read_duration;
      cout << "Select data you want to read\n";
      cout << "- RP for raw position\n";
      cout << "- I for imu\n";
      cout << "- CP for calibrated position\n";
      cout << "- SG for strain gauges\n";
      cin >> data_type;
      cout << "Insert data read duration in seconds: (0 for infinite read)\n";
      cin >> read_duration;
      auto start = std::chrono::steady_clock::now();
      bool reading_data = true;

      while (reading_data == true)
      {
        counter++;
        if (read_duration > 0)
        {
          if (std::chrono::steady_clock::now() - start > std::chrono::seconds(read_duration))
          {
            reading_data = false;
            std::cout<<"Counter: "<<counter<<"\n";
            break;
          }
        }
        // wait until 1ms has passed since last call to sendAndReceiveFromHand0220()
        // usleep(1000);  // Don't do it like this (take the time of the system to
                      // account for last sending time instead)
        hand.sendAndReceiveFromHand0220();
        if (data_type == "RP")
        {
          for (int i = 0; i < HAND_DRIVER_0220_NB_RAW_SENSORS - HAND_DRIVER_0220_IMU_FIELDS; ++i) {
            printf("sensor[%d] %s = %d\n", i, sensor_names[i],
                  state->hand_1->raw_position[i]);
          }
        }
        else if (data_type == "I")
        {
            for (int i = HAND_DRIVER_0220_NB_RAW_SENSORS - HAND_DRIVER_0220_IMU_FIELDS; i < HAND_DRIVER_0220_NB_RAW_SENSORS; ++i) {
            printf("sensor[%d] %s = %d\n", i, sensor_names[i],
                  state->hand_1->raw_position[i]);
          }
        }
        else if (data_type == "CP")
        {
          for (int i = 0; i < HAND_DRIVER_0220_NB_ALL_JOINTS; ++i) {
            printf("joint[%d] %s calibrated_position = %f\n", i,
                  hand.getCalibratedIdToJointMap().find(i)->second.c_str(),
                  state->hand_1->calibrated_position[i]);
          }
        }
        else if (data_type == "SG")
        {
          for (int i = 0; i < HAND_DRIVER_0220_NB_MOTORS; ++i) {
              printf("motor_data_packet[%d].torque = %d\n", i,
                      state->hand_1->strain_gauges_data[i]);
            }
        }
        else
        {
          std:cout<<"Type of data not recognized, insert a correct flag to read the data\n";
          read_hand_data = false;
          break;
        }

        hand.sendAndReceiveFromHand0220();

        // // The following section is recommended to guarantee that the slow sensor
        // // data fields are as fresh as they can be e.g. most of the biotac data
        // // fields are polled only once every 14 frames so we should prefer not to
        // // skip any frame and send one every 1ms even if we don't recompute the
        // // motor command. Sending the latest motor command computed will have the
        // // same effect as not sending a frame (see above)
        for (int i; i < NB_IDLE_FRAMES; i++) {
          usleep(1000);
          hand.sendAndReceiveFromHand0220();
        }
        usleep(1000);
      } 
    }
    else if (run_all == true)
    {
      clock_gettime(CLOCK_REALTIME, &tick);
      tick.tv_nsec = (tick.tv_nsec / 1000000 + 1) * 1000000;

      auto start_test = std::chrono::steady_clock::now();
      int counter = 0;

      while (std::chrono::steady_clock::now() - start_test <= std::chrono::milliseconds(1))
      {
        counter++;
        
        command->hand_1->use_pwm = true;
        command->hand_1->pwm_command[0] = 1;
        command->hand_1->pwm_command[1] = 1;
        command->hand_1->pwm_command[2] = 1;
        command->hand_1->pwm_command[3] = 1;
        command->hand_1->pwm_command[4] = 1;
        command->hand_1->pwm_command[5] = 1;
        command->hand_1->pwm_command[6] = 1;
        command->hand_1->pwm_command[7] = 1;
        command->hand_1->pwm_command[8] = 1;
        command->hand_1->pwm_command[9] = 1;
        command->hand_1->pwm_command[10] = 1;
        command->hand_1->pwm_command[11] = 1;
        command->hand_1->pwm_command[12] = 1;
        command->hand_1->pwm_command[13] = 1;
        command->hand_1->pwm_command[14] = 1;
        command->hand_1->pwm_command[15] = 1;
        command->hand_1->pwm_command[16] = 1;
        command->hand_1->pwm_command[17] = 1;
        command->hand_1->pwm_command[18] = 1;
        command->hand_1->pwm_command[19] = 1;
        command->hand_1->pwm_command[20] = 1;

        hand.sendAndReceiveFromHand0220();

        for (int i = 0; i < HAND_DRIVER_0220_NB_RAW_SENSORS - HAND_DRIVER_0220_IMU_FIELDS; ++i) {
            printf("sensor[%d] %s = %d\n", i, sensor_names[i],
                  state->hand_1->raw_position[i]);
        }
        for (int i = 0; i < HAND_DRIVER_0220_NB_MOTORS; ++i) {
              printf("motor_data_packet[%d].torque = %d\n", i,
                      state->hand_1->strain_gauges_data[i]);
        }
        for (int i = HAND_DRIVER_0220_NB_RAW_SENSORS - HAND_DRIVER_0220_IMU_FIELDS; i < HAND_DRIVER_0220_NB_RAW_SENSORS; ++i) {
            printf("sensor[%d] %s = %d\n", i, sensor_names[i],
                  state->hand_1->raw_position[i]);
          }
        timespecInc(tick, 1000000);

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
     }
     std::cout<<"\033[1;31mCounter\033[0m\n"<<counter<<"\n";
     if (counter > 1)
      break;
    }
  }
}
