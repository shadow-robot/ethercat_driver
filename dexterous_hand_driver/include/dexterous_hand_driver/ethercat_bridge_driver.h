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

#ifndef ETHERCAT_BRIDGE_DRIVER_H
#define ETHERCAT_BRIDGE_DRIVER_H

#include <ethercat_hardware/ethercat_device.h>

namespace dexterous_hand_driver {
class EthercatBridgeDriver : public EthercatDevice {
 public:
  EthercatBridgeDriver();

  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);

  virtual void packCommand(unsigned char *buffer);

  virtual bool unpackState(unsigned char *this_buffer,
                           unsigned char *prev_buffer);

 private:
  int command_base_;
  int status_base_;
  std::string serial_number_;
};
}  // namespace dexterous_hand_driver
#endif  // ETHERCAT_BRIDGE_DRIVER_H