# Copyright 2018 Google LLC. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Python interface for ShadowHand."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import ctypes
import time

# TODO(sherrym): Get these from the library.
HAND_DRIVER_0220_NB_BIOTAC_ELECTRODES = 24
HAND_DRIVER_0220_NB_FINGERS = 5
HAND_DRIVER_0220_NB_JOINTS = 24
HAND_DRIVER_0220_NB_MOTORS = 20
HAND_DRIVER_0220_NB_RAW_SENSORS = 37
# All joints including the combined ones FFJ0, MFJ0, RFJ0 and LFJ0.
HAND_DRIVER_0220_NB_ALL_JOINTS = 28


class Hand0220Command(ctypes.Structure):
  _fields_ = [
      ("use_pwm", ctypes.c_bool),
      ("pwm_command", ctypes.c_double * HAND_DRIVER_0220_NB_MOTORS),
      ("torque_command", ctypes.c_int * HAND_DRIVER_0220_NB_MOTORS),
  ]


class BiotacData(ctypes.Structure):
  _fields_ = [
      ("pac0", ctypes.c_int),
      ("pac1", ctypes.c_int),
      ("pdc", ctypes.c_int),
      ("tac", ctypes.c_int),
      ("tdc", ctypes.c_int),
      ("electrodes", ctypes.c_int * HAND_DRIVER_0220_NB_BIOTAC_ELECTRODES),
  ]


class Hand0220State(ctypes.Structure):
  _fields_ = [
      ("raw_position", ctypes.c_int * HAND_DRIVER_0220_NB_RAW_SENSORS),
      ("calibrated_position", ctypes.c_double * HAND_DRIVER_0220_NB_ALL_JOINTS),
      ("biotac_data", BiotacData * HAND_DRIVER_0220_NB_FINGERS),
      ("timestamp", ctypes.c_double), # in seconds
  ]


class ShadowHandRobot():
  # Joint name to hardware motor ID map. A value of -1 means the joint cannot be
  # independently controlled.
  _JOINTS_TO_MOTORS_MAP = {
      "FFJ0" : 0,
      "FFJ1" : -1,
      "FFJ2" : -1,
      "FFJ3" : 1,
      "FFJ4" : 2,
      "MFJ0" : 3,
      "MFJ1" : -1,
      "MFJ2" : -1,
      "MFJ3" : 11,
      "MFJ4" : 13,
      "RFJ0" : 15,
      "RFJ1" : -1,
      "RFJ2" : -1,
      "RFJ3" : 16,
      "RFJ4" : 17,
      "LFJ0" : 12,
      "LFJ1" : -1,
      "LFJ2" : -1,
      "LFJ3" : 10,
      "LFJ4" : 14,
      "LFJ5" : 4,
      "THJ1" : 6,
      "THJ2" : 5,
      "THJ3" : 7,
      "THJ4" : 9,
      "THJ5" : 19,
      "WRJ1" : 18,
      "WRJ2" : 8,
  }

  # The Mujoco view of the joint order when sending commands Vector size is
  # HAND_DRIVER_0220_NB_JOINTS.
  # TODO(sherrym): This will map positions set in Mujoco for J2 to motor J0.
  # This map is different from _MUJOCO_STATE_JOINT_ORDER. Positions set in
  # Mujoco for J1 are ignored.
  _MUJOCO_JOINT_COMMAND_ORDER = [
      "WRJ2", "WRJ1",
      "FFJ4", "FFJ3", "FFJ0", "FFJ1",
      "MFJ4", "MFJ3", "MFJ0", "MFJ1",
      "RFJ4", "RFJ3", "RFJ0", "RFJ1",
      "LFJ5", "LFJ4", "LFJ3", "LFJ0", "LFJ1",
      "THJ5", "THJ4", "THJ3", "THJ2", "THJ1",
  ]

  # The Mujoco view of the joint order when receiving positions.  Vector size is
  # HAND_DRIVER_0220_NB_JOINTS.
  MUJOCO_JOINT_STATE_ORDER = [
      "WRJ2", "WRJ1", "FFJ4", "FFJ3", "FFJ2", "FFJ1", "MFJ4", "MFJ3",
      "MFJ2", "MFJ1", "RFJ4", "RFJ3", "RFJ2", "RFJ1", "LFJ5", "LFJ4",
      "LFJ3", "LFJ2", "LFJ1", "THJ5", "THJ4", "THJ3", "THJ2", "THJ1",
  ]

  # TODO(sherrym): These joints are flipped. We explicitly check for them
  # in this file.  Remove once the correct scale are set in adroit_config.xml.
  MUJOCO_NEED_CALIBRATION = ["THJ1", "THJ2", "FFJ4", "MFJ4"]

  def __init__(self):

    # Path to the library.
    full_path_to_libhand = "libdexterous_hand_driver_interface.so"
    ctypes.cdll.LoadLibrary(full_path_to_libhand)
    libhand = ctypes.CDLL(full_path_to_libhand)

    # hand_init()
    self._hand_init = libhand.hand_init
    self._hand_init.argtypes = (ctypes.c_char_p,)
    self._hand_init.restype = None

    # hand_shutdown()
    self._hand_shutdown = libhand.hand_shutdown

    # send_and_receive_from_hand_0220()
    self._send_and_receive_from_hand_0220 = libhand.send_and_receive_from_hand_0220
    self._send_and_receive_from_hand_0220.argtypes = None
    self._send_and_receive_from_hand_0220.restype = None

    # send_command(Hand0220Command* command)
    self._send_command = libhand.send_command
    self._send_command.argtypes = (ctypes.POINTER(Hand0220Command),)
    self._send_command.restype = None

    # receive_state(Hand0220State* state)
    self._receive_state = libhand.receive_state
    self._receive_state.argtypes = (ctypes.POINTER(Hand0220State),)
    self._receive_state.restype = ctypes.c_bool

    # get_ordered_joint_names(
    # std::array<char[5], HAND_DRIVER_0220_NB_ALL_JOINTS>* ordered_joint_names)
    self._get_ordered_joint_names = libhand.get_ordered_joint_names
    c_char_array_5_type = ctypes.c_char * 5
    self._get_ordered_joint_names.argtypes = (
        c_char_array_5_type * HAND_DRIVER_0220_NB_ALL_JOINTS,)
    self._get_ordered_joint_names.restype = None

    # Vector of motor names based on hardware motor id order.
    sorted_motor_names = sorted(
        self._JOINTS_TO_MOTORS_MAP.items(), key=lambda kv: kv[1])
    self._ordered_motor_names = [j[0] for j in sorted_motor_names
                                 if j[1] != -1]

  def hand_reset(self, reset_pose=None):
    """Resets the hand to 0 position
    """
    self.run_loop(commands=reset_pose.tolist(), use_position=True, duration=2.0)

  def hand_init(self, initial_positions=None, port="enx00e04c68c394"):
    """Initializes the hand.

    Args:
      initial_position: Initial positions.
      port: Interface that the hand is connected to.
    """
    self._hand_init(port.encode('utf-8'))
    if initial_positions:
      commands = self.convert_commands(
        initial_positions, defaults_to_current_value=False)
      self.run_loop(commands=commands, use_position=True)

    # Vector of joint names in the returned calibrated states order.
    self._ordered_joint_names = self.get_ordered_joint_names()

  def hand_shutdown(self):
    """Shuts down the hand.

    TODO(vikashplus): Per vikashplus@ the way we shut down the hand is to shut
    down, initialize, thne shut down again.
    """
    self._hand_shutdown()
    self.hand_init()
    self._hand_shutdown()

  def get_ordered_joint_names(self):
    """Get a vector of joint names as ordered by the calibrated positions.

    Returns:
      A list of HAND_DRIVER_0220_NB_ALL_JOINTS joint names as strings.
    """
    c_char_array_5_type = ctypes.c_char * 5
    OrderedJointNames = c_char_array_5_type * HAND_DRIVER_0220_NB_ALL_JOINTS
    ordered_joint_names = OrderedJointNames()
    for i in range(HAND_DRIVER_0220_NB_ALL_JOINTS):
      ordered_joint_names[i] = ctypes.create_string_buffer(5)
    self._get_ordered_joint_names(ordered_joint_names)
    return [joint_name.raw[:4].decode('unicode_escape')
            for joint_name in ordered_joint_names]

  def send_and_receive_from_hand_0220(self):
    """Calls the driver's update() to send commands and receive states.

    The driver's update() code calls packCommand() to pack the command in the
    command buffer and sends it to the hardware, and calls unpackState() to
    unpack the state from hardware into the state buffer.
    """
    self._send_and_receive_from_hand_0220()

  def send_motor_command(self, commands, use_position=True):
    """Copies commands in degrees to the command buffer for the hardware.

    Args:
      commands: A list of HAND_DRIVER_0220_NB_MOTORS integers or doubles.
        If use_position is True, the commands are positions in degrees;
        if use_position is False, the commands are torques.
      use_position: Bool. Whether to use position mode or not.

    Returns:
      None.
    """
    assert len(commands) == HAND_DRIVER_0220_NB_MOTORS
    command = Hand0220Command(
        use_position,
        (ctypes.c_double * len(commands))(*commands),
        (ctypes.c_int * len(commands))(*commands))
    self._send_command(command)

  def receive_state(self):
    """Copies states from the state buffer.

    Returns:
      Hand0220State structure.
    """
    state = Hand0220State()
    self._receive_state(state)
    return state

  def get_mujoco_state(self, state):
    """Returns calibrated positions in radians.

    Args:
      state: Hand0220State.

    Returns:
      A list of HAND_DRIVER_0220_NB_JOINTS (24) calibrated positions as defined
      in MUJOCO_JOINT_STATE_ORDER, in radians.
    """
    return get_mujoco_state(state, self._ordered_joint_names)

  def get_motor_calibrated_positions(self, state):
    """Returns calibrated_positions for controllable motors only.

    Args:
      state: Hand0220State.

    Returns:
      A list of HAND_DRIVER_0220_NB_MOTORS calibrated positions for controllable
      motors only as integers.
    """
    return [int(state.calibrated_position[self._ordered_joint_names.index(jn)])
            for jn in self._ordered_motor_names]


  def convert_commands(self, commands=None, defaults_to_current_value=True,
                       default_value=0):
    """Converts to position or torque commands to be sent to motors.

    Args:
      commands: A map of joint name to commanded position or torque.
        For example: {'FFJ3': 10, 'FFJ0': 20}
      defaults_to_current_value: For unspecified joints, whether to default
        to current value or default_value.
      default_value: If defaults_to_current_value is False, set unspecified
        joint position or torque to this value.

    Returns:
      A list of HAND_DRIVER_0220_NB_MOTORS commanded positions or torques.
      None if no commands are given.
    """
    if commands is None:
      return None

    if defaults_to_current_value:
      state = self.receive_state()
      motor_commands = self.get_motor_calibrated_positions(state)
    else:
      motor_commands = [default_value] * HAND_DRIVER_0220_NB_MOTORS
    for joint_name, command in commands.items():
      if joint_name not in self._JOINTS_TO_MOTORS_MAP:
        print("Joint %s doesn't exist. Ignoring." % joint_name)
      else:
        motor_id = self._JOINTS_TO_MOTORS_MAP[joint_name]
        motor_commands[motor_id] = command
    return motor_commands

  def run_loop(self, commands=None, use_position=True, duration=1.0):
    """Runs send and receive in a loop.

    Args:
      commands: A list of torque or position commands of dimension
        HAND_DRIVER_0220_NB_MOTORS. If no commands are provided,
        just show the observed positions.
      use_position: Whether to use position or torque. Defaults to True.
      duration: Time in seconds for how long to run. Defaults to 1 second.
    """
    start_time = time.time()
    while time.time() - start_time < duration:
      self.send_and_receive_from_hand_0220()
      time.sleep(0.001)
      if commands is not None:
        self.send_command(commands, use_position=use_position)
        time.sleep(0.001)
      state = self.receive_state()
    return state

  def send_mujoco_command(self, commands, use_position=True, in_radians=True):
    """Sends mujoco ordered commands in radians to hardware in degrees.

    Args:
      commands: A list of HAND_DRIVER_0220_NB_JOINTS positions in radians in the
        mujoco order as defined in _MUJOCO_JOINT_COMMAND_ORDER.
      use_position: Bool.  Whether to use position mode or not.
      in_radians: Whether the commands are in radians. If they are, they
        need to be converted to degrees before sending to the motors.

    Returns:
      None.
    """
    # Converts from mujoco ordered commands to motor ordered commands.
    assert len(commands) == HAND_DRIVER_0220_NB_JOINTS
    motor_commands = {}
    for command_index in range(len(commands)):
      joint_name = self._MUJOCO_JOINT_COMMAND_ORDER[command_index]
      motor_index = self._JOINTS_TO_MOTORS_MAP[joint_name]

      # Ignore joints we can't control independently.
      if motor_index != -1:
        motor_commands[motor_index] = commands[command_index]
      if use_position and joint_name in self.MUJOCO_NEED_CALIBRATION:
        motor_commands[motor_index] *= -1

    # Converts from dict to list
    motor_commands_sorted = []
    for key in sorted(motor_commands): 
      motor_commands_sorted.append(motor_commands[key])

    # Converts radians to degrees.
    if in_radians:
      motor_commands_sorted = [int(to_degrees(command)) for command in motor_commands_sorted]

    assert len(motor_commands_sorted) == HAND_DRIVER_0220_NB_MOTORS
    self.send_motor_command(motor_commands_sorted, use_position=use_position)

  def send_command(self, commands, use_position=True):
    """Sends commands to hardware or simulator.


    If commands is of size HAND_DRIVER_0220_NB_MOTORS, we assume that commands
    are in degrees.
    If commands is of size HAND_DRIVER_0220_NB_JOINTS, we assume that commands
    are in radians, and are the same commands one would send to mujoco.

    Args:
      commands: A list of HAND_DRIVER_0220_NB_MOTORS or HAND_DRIVER_0220_NB_JOINTS
        positions.
      use_position: Bool. Whether to use position mode or not.

    Returns:
      None.
    """
    if len(commands) == HAND_DRIVER_0220_NB_MOTORS:
      self.send_motor_command(commands, use_position=use_position)
    else:
      assert len(commands) == HAND_DRIVER_0220_NB_JOINTS
      self.send_mujoco_command(commands, use_position=use_position)



def to_radians(degrees):
  """Converts degress to radians."""
  return degrees * 0.017453292519943295


def to_degrees(radians):
  """Converts radians to degress."""
  return radians * 57.295779513082323


def process_positions(positions_str=None):
  """Converts a joint name to position map string to a python map.

  Args:
    positions: A joint name to position map string, like "FFJ0:90,FFJ3:45".

  Returns:
    A joint name to position map or None if positions is empty.
  """
  if positions_str:
    positions = {}
    for i in positions_str.split(","):
      joint_name, position = i.split(":")
      positions[joint_name] = int(position)
  else:
    positions = None

  return positions


def print_state(state):
  """Prints states.

  Args:
    state: Hand0220State.
  """
  print("Timestamp: ", int(state.timestamp))
  print("Raw Positions: ", state.raw_position[:])
  print("Calibrated Positions: ", state.calibrated_position[:])
  for i in range(len(state.biotac_data)):
    print("BiotacData", i)
    print("\tpac0: \t", state.biotac_data[i].pac0)
    print("\tpac1: \t", state.biotac_data[i].pac1)
    print("\tpdc: \t", state.biotac_data[i].pdc)
    print("\ttac: \t", state.biotac_data[i].tac)
    print("\ttdc: \t", state.biotac_data[i].tdc)
    print("\telectrodes: ", state.biotac_data[i].electrodes[:])


def get_raw_positions(state):
  """Returns raw_positions from state.

  Args:
    state: Hand0220State.

  Returns:
    A list of HAND_DRIVER_0220_NB_RAW_SENSORS raw positions.
  """
  return [state.raw_position[i]
          for i in range(HAND_DRIVER_0220_NB_RAW_SENSORS)]


def get_calibrated_positions(state):
  """Returns calibrated_positions from state.

  Args:
    state: Hand0220State.

  Returns:
    A list of HAND_DRIVER_0220_NB_ALL_JOINTS calibrated positions as doubles.
  """
  return [state.calibrated_position[i]
          for i in range(HAND_DRIVER_0220_NB_ALL_JOINTS)]


def get_mujoco_state(state, ordered_joint_names):
  """Returns calibrated positions in radians.

  Args:
    state: Hand0220State.
    ordered_joint_names: Vector of joint names in the returned calibrated states
      order.

  Returns:
    A list of HAND_DRIVER_0220_NB_JOINTS (24) calibrated positions as defined
    in MUJOCO_JOINT_STATE_ORDER, in radians.
  """
  mujoco_positons = []
  for joint_name in ShadowHandRobot.MUJOCO_JOINT_STATE_ORDER:
    val = to_radians(state.calibrated_position[
        ordered_joint_names.index(joint_name)])
    if joint_name in ShadowHandRobot.MUJOCO_NEED_CALIBRATION:
      val = val * (-1)
    mujoco_positons.append(val)
  return mujoco_positons


def get_timestamp(state):
  """Returns timestamp in seconds.

  Args:
    state: Hand0220State.

  Returns:
    Timestamp in seconds as integer.
  """
  return int(state.timestamp)
