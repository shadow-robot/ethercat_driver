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

#include "dexterous_hand_driver/hand_driver_0220.h"

#include <sstream>
#include <iomanip>

#include "dexterous_hand_driver/external/0220_palm_edc/0220_palm_edc_ethercat_protocol.h"
#include "ethercat_hardware/log.h"

#define ETHERCAT_STATUS_DATA_SIZE sizeof(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS)
#define ETHERCAT_COMMAND_DATA_SIZE sizeof(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND)

#define ETHERCAT_CAN_BRIDGE_DATA_SIZE sizeof(ETHERCAT_CAN_BRIDGE_DATA)

#define ETHERCAT_COMMAND_DATA_ADDRESS                   PALM_0200_ETHERCAT_COMMAND_DATA_ADDRESS
#define ETHERCAT_STATUS_DATA_ADDRESS                    PALM_0200_ETHERCAT_STATUS_DATA_ADDRESS
#define ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS        PALM_0200_ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS
#define ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS         PALM_0200_ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS

namespace dexterous_hand_driver
{
  HandDriver0220::HandDriver0220()
    :initialized_(false)
  {
  }

  void HandDriver0220::construct(EtherCAT_SlaveHandler *sh, int &start_address, unsigned int ethercat_command_data_size,
                         unsigned int ethercat_status_data_size, unsigned int ethercat_can_bridge_data_size,
                         unsigned int ethercat_command_data_address, unsigned int ethercat_status_data_address,
                         unsigned int ethercat_can_bridge_data_command_address,
                         unsigned int ethercat_can_bridge_data_status_address)
  {
    sh_ = sh;

    command_base_ = start_address;
    command_size_ = ethercat_command_data_size + ethercat_can_bridge_data_size;

    start_address += command_size_;

    status_base_ = start_address;
    status_size_ = ethercat_status_data_size + ethercat_can_bridge_data_size;

    start_address += status_size_;

    // ETHERCAT_COMMAND_DATA
    //
    // This is for data going TO the palm
    //
    ROS_INFO("First FMMU (command) : start_address : 0x%08X ; size : %3d bytes ; phy addr : 0x%08X", command_base_,
            command_size_,
            static_cast<int> (ethercat_command_data_address));
    EC_FMMU *commandFMMU = new EC_FMMU(command_base_,  // Logical Start Address    (in ROS address space?)
                                      command_size_,
                                      0x00,  // Logical Start Bit
                                      0x07,  // Logical End Bit
                                      ethercat_command_data_address,  // Physical Start Address(in ET1200 address space?)
                                      0x00,  // Physical Start Bit
                                      false,  // Read Enable
                                      true,  // Write Enable
                                      true);  // Channel Enable




    // ETHERCAT_STATUS_DATA
    //
    // This is for data coming FROM the palm
    //
    ROS_INFO("Second FMMU (status) : start_address : 0x%08X ; size : %3d bytes ; phy addr : 0x%08X", status_base_,
            status_size_,
            static_cast<int> (ethercat_status_data_address));
    EC_FMMU *statusFMMU = new EC_FMMU(status_base_,
                                      status_size_,
                                      0x00,
                                      0x07,
                                      ethercat_status_data_address,
                                      0x00,
                                      true,
                                      false,
                                      true);


    EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);

    (*fmmu)[0] = *commandFMMU;
    (*fmmu)[1] = *statusFMMU;

    sh->set_fmmu_config(fmmu);

    EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(4);

    (*pd)[0] = EC_SyncMan(ethercat_command_data_address, ethercat_command_data_size, EC_QUEUED, EC_WRITTEN_FROM_MASTER);
    (*pd)[1] = EC_SyncMan(ethercat_can_bridge_data_command_address, ethercat_can_bridge_data_size, EC_QUEUED,
                          EC_WRITTEN_FROM_MASTER);
    (*pd)[2] = EC_SyncMan(ethercat_status_data_address, ethercat_status_data_size, EC_QUEUED);
    (*pd)[3] = EC_SyncMan(ethercat_can_bridge_data_status_address, ethercat_can_bridge_data_size, EC_QUEUED);

    status_size_ = ethercat_status_data_size + ethercat_can_bridge_data_size;

    (*pd)[0].ChannelEnable = true;
    (*pd)[0].ALEventEnable = true;
    (*pd)[0].WriteEvent = true;

    (*pd)[1].ChannelEnable = true;
    (*pd)[1].ALEventEnable = true;
    (*pd)[1].WriteEvent = true;

    (*pd)[2].ChannelEnable = true;
    (*pd)[3].ChannelEnable = true;

    sh->set_pd_config(pd);

    ROS_INFO("status_size_ : %d ; command_size_ : %d", status_size_, command_size_);
  }

  void HandDriver0220::construct(EtherCAT_SlaveHandler *sh, int &start_address)
  {
    int i = PALM_0200_ETHERCAT_COMMAND_HEADER_SIZE;
    construct(sh, start_address, ETHERCAT_COMMAND_DATA_SIZE, ETHERCAT_STATUS_DATA_SIZE,
              ETHERCAT_CAN_BRIDGE_DATA_SIZE,
              ETHERCAT_COMMAND_DATA_ADDRESS, ETHERCAT_STATUS_DATA_ADDRESS,
              ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS, ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS);

    ROS_INFO("Finished constructing the 0220 driver");
  }

  void HandDriver0220::packCommand(unsigned char *buffer)
  {
    // We cast the buffer to the low level command structure so that we can fill its fields 
    // directly in the transmission buffer
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *command =
          reinterpret_cast<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *>(buffer);

    command->EDC_command = EDC_COMMAND_SENSOR_DATA;

    if (!initialized_)
    {
      // TODO here we should transmit the necessary parameters to the hand
      // it will take several consecutive frames to do so
      // When all parameters have bee sent and correctly received (this is checked in unpackState)
      // unpackState will set the variable initialized_=true

      // The values in the "high_level_command_" structure are ignored during initialization

      // All the initialization stuff that needs to be added here happens in this function in the ROS driver 
      // https://github.com/shadow-robot/sr-ros-interface-ethercat/blob/kinetic-devel/sr_robot_lib/src/sr_motor_robot_lib.cpp#L197


      // The FROM_MOTOR commands can currently be ignored for the moment, as they are not necessary to operate the hand
      
      // The so called initialization of the tactiles can also be omitted, as it basically checks the type of tactile and version
      // and we assume for the moment that these are biotacs with the latest protocol

      // We need to send a reset signal and then the initialization parameters to the motor boards.
      // Reset is necessary to make the motor jiggle the joint to zero the strain gauges.
      // The parameters we want to send (referred as pid commands for the motor) are contained in this file:
      // https://github.com/shadow-robot/sr-config/blob/shadowrobot_180504/sr_ethercat_hand_config/controls/motors/rh/motor_board_effort_controllers.yaml
      // The parameter pos_filter is not actually one of the ones we send to the motor. It is a parameter for the position filter in the ROS driver.
      // It might not be necessary to filter the position.

      // In the ROS driver these parameters are enqueued here:
      // - The reset command is enqueued here:
      // https://github.com/shadow-robot/sr-ros-interface-ethercat/blob/kinetic-devel/sr_robot_lib/src/sr_motor_hand_lib.cpp#L209
      // Then a timer is set that will enqueue the pid command for the moror here:
      // https://github.com/shadow-robot/sr-ros-interface-ethercat/blob/kinetic-devel/sr_robot_lib/src/sr_motor_hand_lib.cpp#L223
      // Actually two calls, one for the pids
      // https://github.com/shadow-robot/sr-ros-interface-ethercat/blob/kinetic-devel/sr_robot_lib/src/sr_motor_hand_lib.cpp#L284
      // one for the backlash compensation:
      // https://github.com/shadow-robot/sr-ros-interface-ethercat/blob/kinetic-devel/sr_robot_lib/src/sr_motor_hand_lib.cpp#L301


      // The queue is dealt with here. This is where the parameters are actually written to the structure "command", and sent to the 
      // hand. The queued parameters will be sent sequentially in consecutive ethercat frames to the hand.
      // https://github.com/shadow-robot/sr-ros-interface-ethercat/blob/kinetic-devel/sr_robot_lib/src/sr_motor_robot_lib.cpp#L312-L467

      // When the queue is empty we can set
      // initialized_ = true;
    }
    else
    {
      //TODO here we should

      // Take the values in the structure "high_level_command_" and write them to the structure "command"

      // Fill the tactile polling parameters to sequentially read all the values from the biotac sensors
    }
  }

  bool HandDriver0220::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
  {
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *state_data =
          reinterpret_cast<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *>(this_buffer + command_size_);
    if (!initialized_)
    {
      // We don't set the values to the high_level_state_ structure during initialization
    }
    else
    {
      //TODO here we should

      // Take the values in the structure "state_data" and write them to the structure "high_level_command_"
    }
  }

  Hand0220Command* HandDriver0220::getCommandStruct()
  {
    return &high_level_command_;
  }

  Hand0220State* HandDriver0220::getStateStruct()
  {
    return &high_level_state_;
  }

  bool HandDriver0220::initialized()
  {
    return initialized_;
  }
}
