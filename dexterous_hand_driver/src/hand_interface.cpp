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

#include "dexterous_hand_driver/hand_interface.h"
#include "dexterous_hand_driver/ethercat_hardware_hand_0220.h"

dexterous_hand_driver::EthercatHardwareHand0220* hand = nullptr;

void hand_init(char* port) {
  if (hand != nullptr) {
    delete hand;
  }
  hand = new dexterous_hand_driver::EthercatHardwareHand0220(port);
  hand->initializeHand0220();
}

void send_command(dexterous_hand_driver::Hand0220Command* high_level_command) {
  if (hand && high_level_command) {
    dexterous_hand_driver::EthercatHand0220Command* command =
        hand->getCommandStruct();
    memcpy(command->hand_1, high_level_command,
           sizeof(dexterous_hand_driver::Hand0220Command));
  }
}

void receive_state(dexterous_hand_driver::Hand0220State* high_level_state) {
  if (hand && high_level_state) {
    dexterous_hand_driver::EthercatHand0220State* state =
        hand->getStateStruct();
    memcpy(high_level_state, state->hand_1,
           sizeof(dexterous_hand_driver::Hand0220State));
  }
}

void send_and_receive_from_hand_0220() {
  if (hand) {
    hand->sendAndReceiveFromHand0220();
  }
}

void get_ordered_joint_names(
    std::array<char[5], HAND_DRIVER_0220_NB_ALL_JOINTS>* ordered_joint_names) {
  if (hand) {
    const std::map<int, string>& cj_map = hand->getCalibratedIdToJointMap();
    for (int i = 0; i < HAND_DRIVER_0220_NB_ALL_JOINTS; ++i) {
      memcpy((*ordered_joint_names)[i], cj_map.find(i)->second.c_str(), 5);
    }
  }
}

void hand_shutdown() {
  delete hand;
  hand = nullptr;
}
