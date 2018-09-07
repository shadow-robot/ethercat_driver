/*
 * Copyright (C) 2017 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential. 
 * Unauthorized copying of the content in this file, via any medium is strictly prohibited.
 */ 
//! EtherCAT protocol for RoNeX General I/O stacker, 01.
//! Works on Node revision 01

#ifndef RAMCIP_PROTOCOL_VERSION2_0x02000104_H_INCLUDED
#define RAMCIP_PROTOCOL_VERSION2_0x02000104_H_INCLUDED

#include "typedefs_shadow.h"

#if defined(__GNUC__)

#else
    #define __attribute__(x)
#endif

// Look in this_node.h for the #defines to turn these on or off
#ifdef FINGER_VERSION_2
    #define PRODUCT_NAME                 "ramcip_finger_v2"
    #define PRODUCT_ID                          0x02000104
#endif

#ifdef WRIST_VERSION_2
    #define PRODUCT_NAME                  "ramcip_wrist_v2"
    #define PRODUCT_ID                          0x02000106
#endif

// PSoC Hardware Definitions
                                                            // -------------------------
#define COMMAND_TYPE_BITMASK                    0xFF00      //!< Used internally

#define COMMAND_TYPE_INVALID                    0x0000      //!< COMMAND_TYPE values are sent by the host to tell the node
#define COMMAND_TYPE_MOTOR_PWM                  0x0100      //!< For direct control of the motors
#define COMMAND_TYPE_TORQUE_DEMAND              0x0200      //!< To use the internal torque control loop
#define COMMAND_TYPE_SET_PARAMETERS             0x0300      //!< To set the PID values of said loop
#define COMMAND_TYPE_READ_EEPROM_CRC            0x0400
#define COMMAND_TYPE_WRITE_VALUES_TO_EEPROM     0x2000
#define COMMAND_TYPE_READ_STRING                0x4000      //!< reserved[0] = current fragment number (n). Each fragment is 4 bytes long
                                                            //!< reserved[1] = fragment n
                                                            //!< reserved[2] = fragment n+1

#define COMMAND_TYPE_READ_STRING_LENGTH         0x4001      //!< reserved[0] = string length, including zero termination
                                                            //!< reserved[1] = num 4-byte fragments
                                                            //!< reserved[2] = num messages required to read entire string

#define COMMAND_TYPE_READ_MASK                  0x1000
#define COMMAND_TYPE_UNKNOWN_PARAMETER_MASK     0x8000
#define COMMAND_TYPE_PARAMETER_ID_MASK          0x0FFF


#define COMMAND_TYPE_CONFIGURE_PID0_P           0x0301      //!< Proportional gain
#define COMMAND_TYPE_CONFIGURE_PID0_I           0x0302      //!< Integral gain
#define COMMAND_TYPE_CONFIGURE_PID0_D           0x0303      //!< Derivative gain
#define COMMAND_TYPE_CONFIGURE_PID0_F           0x0304      //!< Feed forward gain
#define COMMAND_TYPE_CONFIGURE_PID0_DB          0x0305      //!< Deadband
#define COMMAND_TYPE_CONFIGURE_PID0_FLAGS       0x0306      //!< See PID_SETTINGS_FLAGS_*
#define COMMAND_TYPE_CONFIGURE_PID0_SEN_SEP     0x0307      //!< Separation between sensors
#define COMMAND_TYPE_CONFIGURE_PID0_SEN_ZERO_0  0x0308      //!< Zero position of sensor 0
#define COMMAND_TYPE_CONFIGURE_PID0_SEN_ZERO_1  0x0309      //!< Zero position of sensor 1
#define COMMAND_TYPE_CONFIGURE_PID0_RATE        0x030A      //!< Spring rate in uNm / sensor LSB
#define COMMAND_TYPE_CONFIGURE_PID0_BASELINE    0x030B      //!< Minimum allowable motor on time
#define COMMAND_TYPE_CONFIGURE_PID0_MAX_PWM     0x030C      //!< Maximum allowable motor PWM

#define COMMAND_TYPE_CONFIGURE_PID1_P           0x0311      //!< Proportional gain
#define COMMAND_TYPE_CONFIGURE_PID1_I           0x0312      //!< Integral gain
#define COMMAND_TYPE_CONFIGURE_PID1_D           0x0313      //!< Derivative gain
#define COMMAND_TYPE_CONFIGURE_PID1_F           0x0314      //!< Feed forward gain
#define COMMAND_TYPE_CONFIGURE_PID1_DB          0x0315      //!< Deadband
#define COMMAND_TYPE_CONFIGURE_PID1_FLAGS       0x0316      //!< See PID_SETTINGS_FLAGS_*
#define COMMAND_TYPE_CONFIGURE_PID1_SEN_SEP     0x0317      //!<
#define COMMAND_TYPE_CONFIGURE_PID1_SEN_ZERO_0  0x0318      //!< Zero position of sensor 0
#define COMMAND_TYPE_CONFIGURE_PID1_SEN_ZERO_1  0x0319      //!< Zero position of sensor 1
#define COMMAND_TYPE_CONFIGURE_PID1_RATE        0x031A      //!< Spring rate in uNm / sensor LSB
#define COMMAND_TYPE_CONFIGURE_PID1_BASELINE    0x031B      //!< Minimum allowable motor on time
#define COMMAND_TYPE_CONFIGURE_PID1_MAX_PWM     0x031C      //!< Maximum allowable motor PWM

#define COMMAND_TYPE_CONFIGURE_PID2_P           0x0321      //!< Proportional gain
#define COMMAND_TYPE_CONFIGURE_PID2_I           0x0322      //!< Integral gain
#define COMMAND_TYPE_CONFIGURE_PID2_D           0x0323      //!< Derivative gain
#define COMMAND_TYPE_CONFIGURE_PID2_F           0x0324      //!< Feed forward gain
#define COMMAND_TYPE_CONFIGURE_PID2_DB          0x0325      //!< Deadband
#define COMMAND_TYPE_CONFIGURE_PID2_FLAGS       0x0326      //!< See PID_SETTINGS_FLAGS_*
#define COMMAND_TYPE_CONFIGURE_PID2_SEN_SEP     0x0327      //!<
#define COMMAND_TYPE_CONFIGURE_PID2_SEN_ZERO_0  0x0328      //!< Zero position of sensor 0
#define COMMAND_TYPE_CONFIGURE_PID2_SEN_ZERO_1  0x0329      //!< Zero position of sensor 1
#define COMMAND_TYPE_CONFIGURE_PID2_RATE        0x032A      //!< Spring rate in uNm / sensor LSB
#define COMMAND_TYPE_CONFIGURE_PID2_BASELINE    0x032B      //!< Minimum allowable motor on time
#define COMMAND_TYPE_CONFIGURE_PID2_MAX_PWM     0x032C      //!< Maximum allowable motor PWM

#define COMMAND_TYPE_CONFIGURE_MOTOR_TEMP       0x0380
#define COMMAND_TYPE_CONFIGURE_MOTOR_TEMP_LOW   0x0380      //!< Set Low  temperature for throttling ramp in ºC in 8.8 fixed point format
#define COMMAND_TYPE_CONFIGURE_MOTOR_TEMP_HIGH  0x0381      //!< Set High temperature for throttling ramp in ºC in 8.8 fixed point format
                                                            //!< E.G.  Low  = 60*256
                                                            //!<       High = 80*256
                                                            //!< would allow the motor to be at full power up to 60deg, then throttle
                                                            //!< down until it was at zero power at 80deg.



#define STATUS_TYPE_ERROR                       0xFF00      //!< This is returned in case of error

#define STATUS_FLAG_MOTOR_0_NFAULT              0x0001
#define STATUS_FLAG_MOTOR_1_NFAULT              0x0002
#define STATUS_FLAG_MOTOR_2_NFAULT              0x0004
#define STATUS_FLAG_MOTOR_0_CURRENT_LIMIT       0x0008
#define STATUS_FLAG_MOTOR_1_CURRENT_LIMIT       0x0010
#define STATUS_FLAG_MOTOR_2_CURRENT_LIMIT       0x0020
#define STATUS_FLAG_MOTOR_0_DISABLED            0x0040
#define STATUS_FLAG_MOTOR_1_DISABLED            0x0080
#define STATUS_FLAG_MOTOR_2_DISABLED            0x0100
#define STATUS_FLAG_ALL_MOTORS_DISABLED         0x0200
#define STATUS_FLAG_UNASIGNED_FAULT1            0x0400      
#define STATUS_FLAG_UNASIGNED_FAULT2            0x0800
#define STATUS_FLAG_EEPROM_CRC_FAULT            0x1000
#define STATUS_FLAG_JOINT_0_PARITY_FAULT        0x2000
#define STATUS_FLAG_JOINT_1_PARITY_FAULT        0x4000
#define STATUS_FLAG_JOINT_2_PARITY_FAULT        0x8000

static const int FLAGS_ARRAY[] = {STATUS_FLAG_MOTOR_0_NFAULT,
                                  STATUS_FLAG_MOTOR_1_NFAULT,
                                  STATUS_FLAG_MOTOR_2_NFAULT,
                                  STATUS_FLAG_MOTOR_0_CURRENT_LIMIT,
                                  STATUS_FLAG_MOTOR_1_CURRENT_LIMIT,
                                  STATUS_FLAG_MOTOR_2_CURRENT_LIMIT,
                                  STATUS_FLAG_MOTOR_0_DISABLED,
                                  STATUS_FLAG_MOTOR_1_DISABLED,
                                  STATUS_FLAG_MOTOR_2_DISABLED,
                                  STATUS_FLAG_ALL_MOTORS_DISABLED,
                                  STATUS_FLAG_UNASIGNED_FAULT1,
                                  STATUS_FLAG_UNASIGNED_FAULT2,
                                  STATUS_FLAG_EEPROM_CRC_FAULT,
                                  STATUS_FLAG_JOINT_0_PARITY_FAULT,
                                  STATUS_FLAG_JOINT_1_PARITY_FAULT,
                                  STATUS_FLAG_JOINT_2_PARITY_FAULT};

static const char* FLAG_NAMES[] = {"Motor 0 H-Bridge fault",
                                   "Motor 1 H-Bridge fault",
                                   "Motor 2 H-Bridge fault",
                                   "Motor 0 current limit",
                                   "Motor 1 current limit",
                                   "Motor 2 current limit",
                                   "Motor 0 disabled",
                                   "Motor 1 disabled",
                                   "Motor 2 disabled",
                                   "All motors disabled",
                                   "",
                                   "",
                                   "EEPROM CRC fault",
                                   "Joint 0 sensor fault",
                                   "Joint 1 sensor fault",
                                   "Joint 2 sensor fault"};

static const char *FLAG_DESCRIPTIONS[] = {"Flag turned on when the H-bridge reports a fault."
                                          "A fault can be either a motor fault, undervoltage"
                                          "condition or high temperature condition."
                                          "A motor fault is when the load is shorted to either"
                                          "supply or ground."
                                          "An undervoltage condition is when there is a low"
                                          "voltage on VCP or VREG which are both required for"
                                          "correct function of the H-Bridge."
                                          "A high temperature condition is when the H-Bridge"
                                          "exceeds 160 degrees celsius. If the temperature"
                                          "continues to rise to 170 degrees then the outputs"
                                          "will be disabled until the temperature drops.",

                                          "Flag turned on when the H-bridge reports a fault."
                                          "A fault can be either a motor fault, undervoltage"
                                          "condition or high temperature condition."
                                          "A motor fault is when the load is shorted to either"
                                          "supply or ground."
                                          "An undervoltage condition is when there is a low"
                                          "voltage on VCP or VREG which are both required for"
                                          "correct function of the H-Bridge."
                                          "A high temperature condition is when the H-Bridge"
                                          "exceeds 160 degrees celsius. If the temperature"
                                          "continues to rise to 170 degrees then the outputs"
                                          "will be disabled until the temperature drops.",

                                          "Flag turned on when the H-bridge reports a fault."
                                          "A fault can be either a motor fault, undervoltage"
                                          "condition or high temperature condition."
                                          "A motor fault is when the load is shorted to either"
                                          "supply or ground."
                                          "An undervoltage condition is when there is a low"
                                          "voltage on VCP or VREG which are both required for"
                                          "correct function of the H-Bridge."
                                          "A high temperature condition is when the H-Bridge"
                                          "exceeds 160 degrees celsius. If the temperature"
                                          "continues to rise to 170 degrees then the outputs"
                                          "will be disabled until the temperature drops.",

                                          "Flag turned on when the current in the motor"
                                          "exceeds 1A"
                                          "When this occurs the PWM output is driven low for"
                                          "the current cycle thus limiting the current to 1A.",

                                          "Flag turned on when the current in the motor"
                                          "exceeds 1A"
                                          "When this occurs the PWM output is driven low for"
                                          "the current cycle thus limiting the current to 1A.",

                                          "Flag turned on when the current in the motor"
                                          "exceeds 1A"
                                          "When this occurs the PWM output is driven low for"
                                          "the current cycle thus limiting the current to 1A.",

                                          "Flag turned on if the motor has not had callibration"
                                          "set up or if the encoders for the motor are faulty.",

                                          "Flag turned on if the motor has not had callibration"
                                          "set up or if the encoders for the motor are faulty.",

                                          "Flag turned on if the motor has not had callibration"
                                          "set up or if the encoders for the motor are faulty.",

                                          "Flag turned on in the event of a communication"
                                          "timeout with the host."
                                          "When this occurs the PID controllers are reset and"
                                          "the motors are stopped.",

                                          "Flag unassiged to a fault",

                                          "Flag unassiged to a fault",

                                          "Flag turned on when the calculated eeprom crc value"
                                          "does not match the crc value stored in the eeprom."
                                          "If there is a mismatch then power to the motors will"
                                          "be disabled."
                                          "This flag must be cleared by a succesful parameter"
                                          "setup before the finger will begin to function."

                                          "Flag turned on if either encoder for each joint has"
                                          "a fault"
                                          "Check if sensor wires are connected/damaged etc.",

                                          "Flag turned on if either encoder for each joint has"
                                          "a fault"
                                          "Check if sensor wires are connected/damaged etc.",

                                          "Flag turned on if either encoder for each joint has"
                                          "a fault"
                                          "Check if sensor wires are connected/damaged etc."};

#define PID_SETTINGS_FLAGS_SIGN                 0x0001      //!< Set/clear this bit to change the motor polarity
#define PID_SETTINGS_FLAGS_INITIALISED          0x0002      //!< Set this bit when you have finally initialised the PID controller

#define MOTOR_PWM_PERIOD                          1023

                                                                                    // Queued (Mailbox)
                                                                                    // Syncmanager Definitions
                                                                                    // -----------------------
#define PROTOCOL_TYPE   EC_QUEUED                                                   //!< Synchronous communication
#define COMMAND_ADDRESS 0x1000                                                      //!< ET1200 address containing the Command Structure
#define STATUS_ADDRESS  (COMMAND_ADDRESS+sizeof(RAMCIP_VERSION2_COMMAND_00000104) *4) //!< ET1200 address containing the Status  Structure

#define COMMAND_ARRAY_SIZE_BYTES    (sizeof(RAMCIP_VERSION2_COMMAND_00000104))
#define COMMAND_ARRAY_SIZE_WORDS    (sizeof(RAMCIP_VERSION2_COMMAND_00000104)/2)
#define STATUS_ARRAY_SIZE_BYTES     (sizeof(RAMCIP_VERSION2_STATUS_00000104 ))
#define STATUS_ARRAY_SIZE_WORDS     (sizeof(RAMCIP_VERSION2_STATUS_00000104 )/2)


typedef struct                                  //!< Status Structure - Returned to host
{                                               //   -----------------------------------
    int16u      command_type;                   //!<  2: Copy of command_type from COMMAND struct
    int16u      frame_number;                   //!<  2: Increments every 100uS

    int16s      encoder[6];                     //!< 12: From the AMS sensors
    int32s      torque_uNm[3];                  //!< 12: Calculated
    int32s      velocity[3];                    //!< 12: Measured/Calculated by linear regression

    int16u      incoming_voltage;               //!<  2: Incoming motor power voltage in Volts in 8:8 format
    int16u      current[3];                     //!<  6: Motor current     in Amps in 8:8 format
    int16u      temperature[3];                 //!<  6: Motor temperature in ºC   in 8:8 format

    int32s      reserved[3];                    //!< 12:

    int16u      flags;                          //!<  2:

}__attribute__((packed)) RAMCIP_VERSION2_STATUS_00000104;   //<! Total Size = 56 bytes


typedef struct                                  //!< Command structure - Sent by host
{                                               //   --------------------------------
    int16u      command_type;                   //!<  2: Will be a copy of the value sent in the Command structure

    union
    {
        int32s  demands[3];                     //!< 12: Motor Power: 0..1599   Torque: -16384..16384
        int32s  settings[3];                    //!< 12: Use this when writing PID settings
    }payload;

}__attribute__((packed)) RAMCIP_VERSION2_COMMAND_00000104;


#define RONEX_COMMAND_STRUCT        RAMCIP_VERSION2_COMMAND_00000104
#define RONEX_STATUS_STRUCT         RAMCIP_VERSION2_STATUS_00000104

#define COMMAND_ARRAY_EXPECTED_SIZE_BYTES       14
#define STATUS_ARRAY_EXPECTED_SIZE_BYTES        68

#define EEPROM_SETTINGS_ADDRESS     0x020

#endif