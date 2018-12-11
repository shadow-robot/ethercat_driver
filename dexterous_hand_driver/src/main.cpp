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

#include "dexterous_hand_driver/ethercat_hardware_hand_0220.h"

#define NB_IDLE_FRAMES 9

int main(int argc, char *argv[])
{
  dexterous_hand_driver::EthercatHardwareHand0220 hand;

  // initializeHand0220 will do the etherCAT initialization of all the slaves and then carry on
  // with the initialization of their parameters (e.g all hand params)
  hand.initializeHand0220();

  // Get pointers to the command and state structures
  dexterous_hand_driver::EthercatHand0220Command* command = hand.getCommandStruct();
  dexterous_hand_driver::EthercatHand0220State* state = hand.getStateStruct();

  while(1)
  {
    // Here "command" structure will contain the last command that was written
    // this should be OK, as it is equivalent to not sending a new etherCAT frame
    // (by default the motor controller board will keep applying the latest received motor command
    // if the communication is interrupted, until a 50ms timeout is triggered, that will make
    // the motor controller set the command value to zero as a protection mechanism)
    hand.sendAndReceiveFromHand0220();
    // Here "state" structure contains the latest sensor data received from the hand

    // Do something here with "state" data to compute the next command data
    int first_raw_sensor = state->hand_1->raw_position[0]; // Whatever. e.g read the value from the first raw sensor

    int computed_command = 0;
    // Write your command data to structure "command"
    command->hand_1->use_pwm = true;
    command->hand_1->pwm_command[0] = computed_command; //Whatever e.g. set a pwm command for the first motor
    // wait until 1ms has passed since last call to sendAndReceiveFromHand0220()
    usleep(1000); // Don't do it like this (take the time of the system to account for last sending time instead)

    hand.sendAndReceiveFromHand0220();

    // The following section is recommended to guarantee that the slow sensor data fields
    // are as fresh as they can be
    // e.g. most of the biotac data fields are polled only once every 14 frames
    // so we should prefer not to skip any frame and send one every 1ms
    // even if we don't recompute the motor command.
    // Sending the latest motor command computed will have the same effect as not sending a frame (see above)
    for (int i; i<NB_IDLE_FRAMES; i++)
    {
      usleep(1000);
      hand.sendAndReceiveFromHand0220();
    }
    usleep(1000);
  }
}
