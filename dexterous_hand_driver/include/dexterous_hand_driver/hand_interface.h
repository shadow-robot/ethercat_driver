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
#ifndef HAND_INTERFACE_H
#define HAND_INTERFACE_H

#include "dexterous_hand_driver/hand_driver_0220.h"

#if defined(__cplusplus)
extern "C" {
#endif

void hand_init(char* port);
void send_command(dexterous_hand_driver::Hand0220Command* high_level_command);
void receive_state(dexterous_hand_driver::Hand0220State* high_level_state);
void send_and_receive_from_hand_0220();
void get_ordered_joint_names(
    std::array<char[5], HAND_DRIVER_0220_NB_ALL_JOINTS>* ordered_joint_names);
void hand_shutdown();

#endif

#if defined(__cplusplus)
}

#endif  // HAND_INTERFACE_H
