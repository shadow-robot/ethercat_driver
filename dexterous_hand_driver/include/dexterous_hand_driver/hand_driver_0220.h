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

#ifndef HAND_DRIVER_0220_H
#define HAND_DRIVER_0220_H

#include <ethercat_hardware/ethercat_device.h>

#include <array>

#define HAND_DRIVER_0220_NB_JOINTS              24
#define HAND_DRIVER_0220_NB_RAW_SENSORS         37
#define HAND_DRIVER_0220_NB_MOTORS              20
#define HAND_DRIVER_0220_NB_BIOTAC_ELECTRODES   24
#define HAND_DRIVER_0220_NB_FINGERS             5

namespace dexterous_hand_driver
{

struct BiotacData
{
  int pac0;
  int pac1;
  int pdc;
  int tac;
  int tdc;
  std::array<int, HAND_DRIVER_0220_NB_BIOTAC_ELECTRODES> electrodes = {{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}};
};

struct Hand0220Command
{
  bool use_pwm;
  std::array<int, HAND_DRIVER_0220_NB_MOTORS> pwm_command = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
  std::array<int, HAND_DRIVER_0220_NB_MOTORS> torque_command = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
};

struct Hand0220State
{
  std::array<int, HAND_DRIVER_0220_NB_RAW_SENSORS>  raw_position;
  std::array<int, HAND_DRIVER_0220_NB_JOINTS>  calibrated_position;
  std::array<BiotacData, HAND_DRIVER_0220_NB_FINGERS> biotac_data;
};

class HandDriver0220 : public EthercatDevice
{
public:
  HandDriver0220();

  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address, unsigned int ethercat_command_data_size,
                         unsigned int ethercat_status_data_size, unsigned int ethercat_can_bridge_data_size,
                         unsigned int ethercat_command_data_address, unsigned int ethercat_status_data_address,
                         unsigned int ethercat_can_bridge_data_command_address,
                         unsigned int ethercat_can_bridge_data_status_address);

  virtual void packCommand(unsigned char *buffer);

  virtual bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

  Hand0220Command* getCommandStruct();
  Hand0220State* getStateStruct();
  bool initialized();

private:
  int command_base_;
  int status_base_;
  std::string serial_number_;
  Hand0220Command high_level_command_;
  Hand0220State high_level_state_;
  bool initialized_;
};
}
#endif  // HAND_DRIVER_0220_H