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
#include "ethercat_hardware/log.h"

namespace dexterous_hand_driver
{
  EthercatHardwareHand0220::EthercatHardwareHand0220()
  : EthercatHardware::EthercatHardware("enp0s25", false) // eth port name will be read in initialize, not set in constructor
  {

  }

  EthercatHardwareHand0220::~EthercatHardwareHand0220()
  {

  }

  bool EthercatHardwareHand0220::initializeHand0220()
  {
    // Read relevant configuration (from file for the moment)

    if (!interface_.empty())
    {
      init();
    }
    else
    {
      ROS_DEBUG("No ethercat interface given. EthercatHardware will not be initialized");
    }
  }

  boost::shared_ptr<EthercatDevice> EthercatHardwareHand0220::configSlave(EtherCAT_SlaveHandler *sh)
  {
    boost::shared_ptr<EthercatDevice> p;
    unsigned product_code = sh->get_product_code();
    unsigned serial = sh->get_serial();
    uint32_t revision = sh->get_revision();
    unsigned slave = sh->get_station_address() - 1;

    if (product_code == 6)
    {
      p.reset(new HandDriver0220());
    }
    else if (product_code == 0)
    {
      p.reset(new EthercatBridgeDriver());
    }

    if (p != NULL)
    {
      p->construct(sh, start_address_);
    }

    return p;
  }

  bool EthercatHardwareHand0220::sendAndReceiveFromHand0220(unsigned char *command_buffer /* this will be a struct */, unsigned char *status_buffer /* this will be a struct */)
  {
    update();
  }
}
