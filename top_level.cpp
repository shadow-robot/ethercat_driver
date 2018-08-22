#include hand_XXX_driver.h

// No memory allocation outside init 

main{
    /// This function will do all the necessary work to set up the hand:
    /// - Scan bus for slaves
    /// - If there's a hand of type XXX then
    
    /// - initialize hand [p0-m1]
    
    /// - configure hand
    /// Configurations for the hand will be read from a file, these will include:
    /// - Ethernet port to use
    /// - internal tuning and over-tension potection parameters
    /// - desired operation mode (PWM or tendon tension)
    int success = initialize_hand_XXX();

    /// buffer where the commands (PWM or torque) to the robot motors will be written
    unsigned char *command_buffer;

    /// buffer where the status (sensor data) from the robot can be read from
    unsigned char *status_buffer;

    /// From here 
    while(true)
    {
        /// This will use the command_buffer data to build an etherCAT frame and send it to the hand
        /// The etherCAT frame comes back with sensor data from the hand that can be read from the status_buffer
        send_and_receive_from_hand_XXX(command_buffer, status_buffer);

        /// Do stuff with the status data (compute next commands)
        
        /// Wait until time for next frame

        send_and_receive_from_hand_XXX(command_buffer, status_buffer);
        /// Waiting until next iteration

    }
}