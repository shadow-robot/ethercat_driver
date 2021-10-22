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

#include "dexterous_hand_driver/ethercat_hardware_hand_0220.h"
#include "ethercat_hardware/log.h"

#include <boost/pointer_cast.hpp>

#define ETHERCAT_HARDWARE_HAND_0220_PRODUCT_CODE 9
#define ETHERCAT_HARDWARE_HAND_0220_BRIDGE_PRODUCT_CODE 0

namespace dexterous_hand_driver {
EthercatHardwareHand0220::EthercatHardwareHand0220(const string& interface)
    : EthercatHardware::EthercatHardware(interface, false) {}

EthercatHardwareHand0220::~EthercatHardwareHand0220() {}

bool EthercatHardwareHand0220::initializeHand0220(
    const string& yaml_filename,
    const string& position_controller_yaml_filename) {
  // TODO Read relevant configuration (from file for the moment)

  if (!interface_.empty()) {
    init();
  } else {
    SHADOWHAND_DEBUG(
        "No ethercat interface given. EthercatHardware will not be "
        "initialized");
  }

  hand_driver_ = boost::dynamic_pointer_cast<HandDriver0220>(findHand());

  // Initializes the hand driver's internal data structure.
  hand_driver_->initializeStruct(yaml_filename,
                                 position_controller_yaml_filename);

  command_.hand_1 = hand_driver_->getCommandStruct();
  state_.hand_1 = hand_driver_->getStateStruct();

  bool hand_initialized = false;
  while (!hand_driver_->initialized()) {
    sendAndReceiveFromHand0220();
    usleep(1000);
  }

  return true;  // Maybe there should be a timeout
}

boost::shared_ptr<EthercatDevice> EthercatHardwareHand0220::configSlave(
    EtherCAT_SlaveHandler* sh) {
  boost::shared_ptr<EthercatDevice> p;
  unsigned product_code = sh->get_product_code();
  std::cout<<"Product code: "<<product_code<<std::endl;
  unsigned serial = sh->get_serial();
  std::cout<<"Serial: "<<serial<<std::endl;
  uint32_t revision = sh->get_revision();
  unsigned slave = sh->get_station_address() - 1;

  if (product_code == ETHERCAT_HARDWARE_HAND_0220_PRODUCT_CODE) {
    printf("Initializing ETHERCAT_HARDWARE_HAND_0220_PRODUCT_CODE\n");
    p.reset(new HandDriver0220());
  } else if (product_code == 0) {
    printf("Initializing EthercatBridgeDriver\n");
    p.reset(new EthercatBridgeDriver());
  }

  if (p != NULL) {
    p->construct(sh, start_address_);
  }

  return p;
}

boost::shared_ptr<EthercatDevice> EthercatHardwareHand0220::findHand() {
  for (unsigned int s = 0; s < slaves_.size(); ++s) {
    if (slaves_[s]->sh_->get_product_code() ==
        ETHERCAT_HARDWARE_HAND_0220_PRODUCT_CODE)  // TODO currently this can
                                                   // only deal with one hand
                                                   // per ethercat bus
    {
      return slaves_[s];
    }
  }
}

bool EthercatHardwareHand0220::sendAndReceiveFromHand0220() { update(); }

EthercatHand0220Command* EthercatHardwareHand0220::getCommandStruct() {
  return &command_;
}

EthercatHand0220State* EthercatHardwareHand0220::getStateStruct() {
  return &state_;
}

std::array<int, HAND_DRIVER_0220_NB_MOTORS>*
EthercatHardwareHand0220::getCalibratedIds() {
  return hand_driver_.get()->getCalibratedIds();
}

const std::map<string, int>&
EthercatHardwareHand0220::getJointToCalibratedIdMap() {
  return hand_driver_.get()->getJointToCalibratedIdMap();
}

const std::map<int, string>&
EthercatHardwareHand0220::getCalibratedIdToJointMap() {
  return hand_driver_.get()->getCalibratedIdToJointMap();
}

const std::map<string, int>& EthercatHardwareHand0220::getJointsToMotorsMap() {
  return hand_driver_.get()->getJointsToMotorsMap();
}

const std::map<int, string>& EthercatHardwareHand0220::getMotorsToJointsMap() {
  return hand_driver_.get()->getMotorsToJointsMap();
}

}  // namespace dexterous_hand_driver
