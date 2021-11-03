"""Example for using the ShadowHandRobot class.

export LD_LIBRARY_PATH=path-to-libdexterous_hand_driver_interface.so
export PYTHONPATH=path-to-hand-interface
python3 hand_example.py

"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import hand_interface


def run_demo(robot, use_position=True, duration=1.0):
  if use_position:
    FF_close = [90,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    MF_close = [0,0,0,90,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0]
    RF_close = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,90,0,0,0]
    LF_close = [0,0,0,0,0,0,0,0,0,0,90,0,90,0,0,0,0,0,0,0]
    TH_close = [0,0,0,0,0,90,90,0,0,0,0,0,0,0,0,0,0,0,0,0]
    ALL_open = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
  else:
    FF_close = [200,200,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    MF_close = [0,0,0,200,0,0,0,0,0,0,0,200,0,0,0,0,0,0,0,0]
    RF_close = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,200,200,0,0,0]
    LF_close = [0,0,0,0,0,0,0,0,0,0,200,0,200,0,0,0,0,0,0,0]
    TH_close = [0,0,0,0,0,-150,-150,0,0,0,0,0,0,0,0,0,0,0,0,0]

    FF_open = [-200,-150,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    MF_open = [0,0,0,-200,0,0,0,0,0,0,0,-150,0,0,0,0,0,0,0,0]
    RF_open = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-200,-200,0,0,0]
    LF_open = [0,0,0,0,0,0,0,0,0,0,-200,0,-200,0,0,0,0,0,0,0]
    TH_open = [0,0,0,0,0,150,150,0,0,0,0,0,0,0,0,0,0,0,0,0]

  close_positions = [FF_close, MF_close, RF_close, LF_close, TH_close]
  if use_position:
    print("Closing hand")
    cur_position = close_positions[0]
    for p in close_positions[1:]:
      robot.run_loop(cur_position, use_position=use_position, duration=duration)
      cur_position = [x + y for x, y in zip(cur_position, p)]
    robot.run_loop(cur_position, use_position=use_position, duration=duration)
    print("Open hand")
    robot.run_loop(ALL_open, use_position=use_position, duration=duration)
  else:
    open_positions = [FF_open, MF_open, RF_open, LF_open, TH_open]
    print("Closing hand")
    for p in close_positions:
      robot.run_loop(p, use_position=use_position, duration=duration)
    print("Open hand")
    open_positions.reverse()
    for p in open_positions:
      robot.run_loop(p, use_position=use_position, duration=duration)

def main():
  parser = argparse.ArgumentParser(
      description="Python example for using the hand_interface")
  parser.add_argument("--demo", action="store_true",
                      default=False,
                      help="Whether to command to demo positions")
  parser.add_argument("--use_torque", action="store_true", default=False,
                      help="Use pwm or torque")
  parser.add_argument("--commands", type=str,
                      help="Comma separated map from joint_name to command")
  parser.add_argument("--torque_duration", type=float, default=5.0,
                      help="How long to run a torque command")
  parser.add_argument("--pwm_duration", type=float, default=5.0,
                      help="How long to run a position command")
  parser.add_argument("--run_forever", action="store_true", default=False,
                      help="Whether or not to run forever")
  parser.add_argument("--initial_positions", type=str,
                      help="Comma separated map from joint_name to "
                      "initial positions. Unspecified joints will be set to 0")
  args = parser.parse_args()
  use_position = not args.use_torque
  pwm_duration = args.pwm_duration
  torque_duration = args.torque_duration
  run_forever = args.run_forever

  initial_positions = hand_interface.process_positions(args.initial_positions)
  commands = hand_interface.process_positions(args.commands)

  robot = hand_interface.ShadowHandRobot()

  print("Initializing hand")
  robot.hand_init(initial_positions=initial_positions)

  ordered_joint_names = robot.get_ordered_joint_names()
  print("Ordered joint names", ordered_joint_names)

  try:
    if args.demo:
      duration = pwm_duration if use_position else torque_duration
      run_demo(robot, use_position=use_position, duration=duration)
    else:
      first_run = True
      commands = robot.convert_commands(commands, defaults_to_current_value=use_position)
      while run_forever or first_run:
        robot.run_loop(commands=commands, use_position=use_position,
                       duration=pwm_duration)
        first_run = False
  finally:
    robot.hand_shutdown()

if __name__ == '__main__':
  main()
