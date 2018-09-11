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

// #include "dexterous_hand_driver/hand_h/RAMCIP_Protocol_Version_2_0x02000104_00.h"
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

  }

  bool HandDriver0220::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
  {

  }

  // void HandDriver0220::construct(EtherCAT_SlaveHandler *sh, int &start_address)
  // {
  //   sh_ = sh;

  //   std::stringstream serial_number_string_stream;
  //   serial_number_string_stream << std::setfill('0') << std::setw(9) << sh->get_serial();
  //   serial_number_ = serial_number_string_stream.str();

  //   // Set up ethercat device
  //   command_base_  = start_address;
  //   command_size_  = COMMAND_ARRAY_SIZE_BYTES;

  //   start_address += command_size_;
  //   status_base_   = start_address;
  //   status_size_   = STATUS_ARRAY_SIZE_BYTES;

  //   start_address += status_size_;

  //   // ETHERCAT_COMMAND_DATA
  //   //
  //   // This is for data going TO the board
  //   //

  //   if ( (PROTOCOL_TYPE) == (EC_BUFFERED) )
  //   {
  //     ROS_INFO("Using EC_BUFFERED");
  //   }
  //   else if ( (PROTOCOL_TYPE) == (EC_QUEUED) )
  //   {
  //     ROS_INFO("Using EC_QUEUED");
  //   }

  //   ROS_INFO("First FMMU (command) : Logical address: 0x%08X ; size: %3d bytes ; ET1200 address: 0x%08X", command_base_,
  //           command_size_, static_cast<int>(COMMAND_ADDRESS) );
  //   EC_FMMU *commandFMMU = new EC_FMMU(command_base_,            // Logical Start Address    (in ROS address space?)
  //                                     command_size_,
  //                                     0x00,                     // Logical Start Bit
  //                                     0x07,                     // Logical End Bit
  //                                     COMMAND_ADDRESS,          // Physical Start Address  (in ET1200 address space?)
  //                                     0x00,                     // Physical Start Bit
  //                                     false,                    // Read Enable
  //                                     true,                     // Write Enable
  //                                     true);                    // Channel Enable

  //   // WARNING!!!
  //   // We are leaving (command_size_ * 4) bytes in the physical memory of the device, but strictly we only need to
  //   // leave (command_size_ * 3). This change should be done in the firmware as well, otherwise it won't work.
  //   // This triple buffer is needed in the ethercat devices to work in EC_BUFFERED mode (in opposition to the other mode
  //   // EC_QUEUED, the so called mailbox mode)

  //   // ETHERCAT_STATUS_DATA
  //   //
  //   // This is for data coming FROM the board
  //   //
  //   ROS_INFO("Second FMMU (status) : Logical address: 0x%08X ; size: %3d bytes ; ET1200 address: 0x%08X", status_base_,
  //           status_size_, static_cast<int>(STATUS_ADDRESS) );
  //   EC_FMMU *statusFMMU = new EC_FMMU(status_base_,
  //                                     status_size_,
  //                                     0x00,
  //                                     0x07,
  //                                     STATUS_ADDRESS,
  //                                     0x00,
  //                                     true,
  //                                     false,
  //                                     true);


  //   EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);

  //   (*fmmu)[0] = *commandFMMU;
  //   (*fmmu)[1] = *statusFMMU;

  //   sh->set_fmmu_config(fmmu);

  //   EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(2);

  // // SyncMan takes the physical address
  //   (*pd)[0] = EC_SyncMan(COMMAND_ADDRESS,              command_size_,    PROTOCOL_TYPE, EC_WRITTEN_FROM_MASTER);
  //   (*pd)[1] = EC_SyncMan(STATUS_ADDRESS,               status_size_,     PROTOCOL_TYPE);


  //   (*pd)[0].ChannelEnable = true;
  //   (*pd)[0].ALEventEnable = true;
  //   (*pd)[0].WriteEvent    = true;

  //   (*pd)[1].ChannelEnable = true;

  //   sh->set_pd_config(pd);

  //   ROS_INFO("Finished constructing the FhThreeJointsFinger driver");
  // }

}
