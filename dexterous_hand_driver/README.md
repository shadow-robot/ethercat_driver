# dexterous_hand_driver
A driver class for Hand E, protocol version 0220

To build and install:

First build and install `ethercat_hardware` following instructions.

Then:

```
# mkdir build
# cd build
# cmake -DCMAKE_INSTALL_PREFIX:PATH=${HOME} ..
# make
# make install
```

To run the driver

```
sudo -s
export LD_LIBRARY_PATH=${HOME}/lib:$LD_LIBRARY_PATH
dexterous_hand_driver_0220
```

## What needs to be adapted to run the driver on a new hand unit

* Ethernet port parameter (where the hand is plugged) should be specified via the python or the C++ interface.
* In the C++ main, use the path to the correct yaml files (motor boards and position controllers) for your hand (take them from https://github.com/shadow-robot/sr_hand_config/tree/master/sr_hand_config. You will need to know your hand serial number). E.g. this is the right hand template: https://github.com/shadow-robot/sr_hand_config/tree/master/sr_hand_config/Template_ER/controls

```c++
  string ether_interface = "enx00e04c68c394";
  string motor_board_effort_controller_file =
      "../src/motor_board_effort_controllers.yaml";
  string position_controller_file =
      "../src/sr_edc_joint_position_controllers_PWM.yaml";
```
In the python interface this is still not exposed, so in that case you will have to place the files in the place where the code expects it by default.

* Make sure that the correct motor mapping for your type of hand is hardcoded in common_edc_ethercat_protocol.h. You can check which one by looking in the general_info.yaml for you hand. See the URL for the right hand template here https://github.com/shadow-robot/sr_hand_config/blob/master/sr_hand_config/Template_ER/general_info.yaml#L26. You will need to check the URL for you hand based on its serial number.

* Joint calibration. Make sure that you hardcode the correct calibration values for your hand in the file `calibration.h`. You can read these values from your hand config on github. See the URL for the right hand template here https://github.com/shadow-robot/sr_hand_config/blob/master/sr_hand_config/Template_ER/calibrations/calibration.yaml. You will need to check the URL for you hand based on its serial number.
