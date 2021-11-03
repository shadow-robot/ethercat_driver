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

#ifndef ETHERCAT_HARDWARE_HAND_0220_H
#define ETHERCAT_HARDWARE_HAND_0220_H

#include <ethercat_hardware/ethercat_hardware.h>
#include "dexterous_hand_driver/ethercat_bridge_driver.h"
#include "dexterous_hand_driver/hand_driver_0220.h"

namespace dexterous_hand_driver {

struct EthercatHand0220Command {
  Hand0220Command* hand_1;
  // More members would have to be declared here if more ethercat slaves are
  // present in the bus (bridge doesn't have data so it is not here) e.g. if a
  // second dexterous hand is connected to the same etherCAT bus we would need
  // to declare a hand_2 field
};

struct EthercatHand0220State {
  Hand0220State* hand_1;
  // More members would have to be declared here if more ethercat slaves are
  // present in the bus (bridge doesn't have data so it is not here) e.g. if a
  // second dexterous hand is connected to the same etherCAT bus we would need
  // to declare a hand_2 field
};

class EthercatHardwareHand0220 : public EthercatHardware {
 public:
  EthercatHardwareHand0220(const string& interface = "enx00e04c706b17");
  virtual ~EthercatHardwareHand0220();

  // Initializes the hand. This will perform etherCAT initialization of all the
  // slaves and their hand parameters.
  bool initializeHand0220(
      const string& yaml_filename = "motor_board_effort_controllers.yaml",
      const string& position_controller_yaml_filename =
          "sr_edc_joint_position_controllers_PWM.yaml");

  // Sends commands and receives statuses from the hand.
  void sendAndReceiveFromHand0220();

  void setCommandForHand0220(
      unsigned char* command_buffer /* this will be a struct */);

  void getStatusFromHand0220(
      unsigned char* status_buffer /* this will be a struct */);

  // Returns pointer to EthercatHand0220Command structure.
  EthercatHand0220Command* getCommandStruct();

  // Returns pointer to EthercatHand0220State structure.
  EthercatHand0220State* getStateStruct();

  std::array<int, HAND_DRIVER_0220_NB_MOTORS>* getCalibratedIds();

  const std::map<string, int>& getJointToCalibratedIdMap();
  const std::map<int, string>& getCalibratedIdToJointMap();
  const std::map<string, int>& getJointsToMotorsMap();
  const std::map<int, string>& getMotorsToJointsMap();

 private:
  virtual boost::shared_ptr<EthercatDevice> configSlave(
      EtherCAT_SlaveHandler* sh);
  boost::shared_ptr<EthercatDevice> findHand();

  boost::shared_ptr<HandDriver0220> hand_driver_;
  boost::shared_ptr<EthercatBridgeDriver> ethercat_bridge_driver_;
  EthercatHand0220Command command_;
  EthercatHand0220State state_;
};
}  // namespace dexterous_hand_driver

#endif  // ETHERCAT_HARDWARE_HAND_0220_H
