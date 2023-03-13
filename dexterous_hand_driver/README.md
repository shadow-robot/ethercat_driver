# dexterous_hand_driver
A driver class for Hand E, protocol version 0220

To build and install:

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
export LD_LIBRARY_PATH=/home/user/lib:$LD_LIBRARY_PATH
dexterous_hand_driver_0220
```
