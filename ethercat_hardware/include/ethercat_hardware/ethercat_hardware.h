/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#ifndef ETHERCAT_HARDWARE_H
#define ETHERCAT_HARDWARE_H

#include <eml/ethercat_AL.h>
#include <eml/ethercat_master.h>
#include <eml/ethercat_slave_handler.h>
#include <eml/ethercat_dll.h>
#include <eml/ethercat_device_addressed_telegram.h>

#include "ethercat_hardware/ethercat_device.h"
#include "ethercat_hardware/ethercat_com.h"
#include "ethercat_hardware/ethernet_interface_info.h"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <boost/regex.hpp>

using namespace boost::accumulators;

struct EthercatHardwareDiagnostics
{
  EthercatHardwareDiagnostics();
  void resetMaxTiming();
  accumulator_set<double, stats<tag::max, tag::mean> > pack_command_acc_; //!< time taken by all devices packCommand functions
  accumulator_set<double, stats<tag::max, tag::mean> > txandrx_acc_; //!< time taken by to transmit and recieve process data
  accumulator_set<double, stats<tag::max, tag::mean> > unpack_state_acc_; //!< time taken by all devices updateState functions
  accumulator_set<double, stats<tag::max, tag::mean> > publish_acc_; //!< time taken by any publishing step in main loop
  double max_pack_command_;
  double max_txandrx_;
  double max_unpack_state_;
  double max_publish_;
  int txandrx_errors_;
  unsigned device_count_;
  bool pd_error_;
  bool halt_after_reset_; //!< True if motor halt soon after motor reset
  unsigned reset_motors_service_count_; //!< Number of times reset_motor service has been used
  unsigned halt_motors_service_count_; //!< Number of time halt_motor service call is used
  unsigned halt_motors_error_count_; //!< Number of transitions into halt state due to device error
  struct netif_counters counters_;
  bool input_thread_is_stopped_;
  bool motors_halted_; //!< True if motors are halted
  const char* motors_halted_reason_; //!< reason that motors first halted

  static const bool collect_extra_timing_ = true;
};

class EthercatHardware
{
public:
  /*!
   * \brief Scans the network and gives a list of detected devices on a given ethercat port
   * \param eth is the thernet port to be scanned
   */
  static std::vector<EtherCAT_SlaveHandler> scanPort(const std::string& eth);


  /*!
   * \brief Constructor
   */
  EthercatHardware(const std::string& name, const string &eth, bool allow_unprogrammed);

  /*!
   * \brief Destructor
   */
  ~EthercatHardware();

  /*!
   * \brief Send most recent motor commands and retrieve updates. This command must be run at a sufficient rate or else the motors will be disabled.
   * \returns A boolean indicating success of the transmission
   */
  bool update();

  /*!
   * \brief Initialize the EtherCAT Master Library.
   */
  void init();

  /*!
   * \brief Collects diagnostics from all devices.
   */
  void collectDiagnostics();

  void printCounters(std::ostream &os = std::cout);

  /*!
   * \brief Send process data
   */
  bool txandrx_PD(unsigned buffer_size, unsigned char* buffer, unsigned tries);

  const std::vector<boost::shared_ptr<const EthercatDevice> > getSlaves() const
  {
    return std::vector<boost::shared_ptr<const EthercatDevice> >(slaves_.begin(), slaves_.end());
  }

private:
  static void changeState(EtherCAT_SlaveHandler *sh, EC_State new_state);

  virtual boost::shared_ptr<EthercatDevice> configSlave(EtherCAT_SlaveHandler *sh);
  bool setRouterToSlaveHandlers();

  struct netif *ni_;

  string interface_; //!< The socket interface that is connected to the EtherCAT devices (e.g., eth0)

  EtherCAT_DataLinkLayer m_dll_instance_;
  EC_Logic m_logic_instance_;
  EtherCAT_PD_Buffer pd_buffer_;
  EtherCAT_AL *application_layer_;
  EtherCAT_Router *m_router_;
  EtherCAT_Master *ethercat_master_;

  std::vector<boost::shared_ptr<EthercatDevice> > slaves_;
  unsigned int num_ethercat_devices_;

  unsigned char *this_buffer_;
  unsigned char *prev_buffer_;
  unsigned char *buffers_;
  unsigned int buffer_size_;

  bool halt_motors_;
  unsigned int reset_state_;

  unsigned timeout_; //!< Timeout (in microseconds) to used for sending/recieving packets once in realtime mode.
  unsigned max_pd_retries_; //!< Max number of times to retry sending process data before halting motors

  EthercatHardwareDiagnostics diagnostics_;
  struct timespec last_published_;
  struct timespec last_reset_;

  EthercatOobCom *oob_com_;

  bool allow_unprogrammed_; //!< if the driver should treat the discovery of unprogrammed boards as a fatal error. Set to 'true' during board configuration, and set to 'false' otherwise.

  int start_address_;
};

#endif /* ETHERCAT_HARDWARE_H */
