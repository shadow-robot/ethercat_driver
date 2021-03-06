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

#ifndef ETHERCAT_HARDWARE_HAND_0220_H
#define ETHERCAT_HARDWARE_HAND_0220_H

#include <ethercat_hardware/ethercat_hardware.h>
#include "dexterous_hand_driver/hand_driver_0220.h"
#include "dexterous_hand_driver/ethercat_bridge_driver.h"

namespace dexterous_hand_driver
{

struct EthercatHand0220Command
{
  Hand0220Command* hand_1;
  // More members would have to be declared here if more ethercat slaves are present in the bus (bridge doesn't have data so it is not here)
  // e.g. if a second dexterous hand is connected to the same etherCAT bus we would need to declare a hand_2 field
};

struct EthercatHand0220State
{
  Hand0220State* hand_1;
  // More members would have to be declared here if more ethercat slaves are present in the bus (bridge doesn't have data so it is not here)
  // e.g. if a second dexterous hand is connected to the same etherCAT bus we would need to declare a hand_2 field
};

class EthercatHardwareHand0220 : public EthercatHardware
{
public:
  EthercatHardwareHand0220();
  virtual ~EthercatHardwareHand0220();
  bool initializeHand0220();
  bool sendAndReceiveFromHand0220();
  void setCommandForHand0220(unsigned char *command_buffer /* this will be a struct */);
  void getStatusFromHand0220(unsigned char *status_buffer /* this will be a struct */);
  EthercatHand0220Command* getCommandStruct();
  EthercatHand0220State* getStateStruct();

private:
  virtual boost::shared_ptr<EthercatDevice> configSlave(EtherCAT_SlaveHandler *sh);
  boost::shared_ptr<EthercatDevice> findHand();

  boost::shared_ptr<HandDriver0220> hand_driver_;
  boost::shared_ptr<EthercatBridgeDriver> ethercat_bridge_driver_;
  EthercatHand0220Command command_;
  EthercatHand0220State state_;
};
}
#endif  // ETHERCAT_HARDWARE_HAND_0220_H