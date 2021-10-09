/**************************************************************************************************
 * @file        i2c1_board_temperature.cpp
 * @author      Thomas
 * @brief       Source file for I2C board temperature sensor
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/i2c_dev_driver/i2c1_board_temperature.hpp"
// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// ---------------
// None

// Project Libraries
// -----------------
#include "tasks/0_hardware_drivers/i2c_dev_driver/i2c1_dev_driver_parameters.hpp"


#include "FileIndex.h"
//~~~~~~~~~~~~~~~~~~~~
#include FilInd_I2CPe__HD               // Include the I2C class handler
#include FilInd_AD741x_HD               // Include the device AD741x handler

//=================================================================================================

namespace _i2c1_dev::_int_temp {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
inline constexpr uint8_t   form_depth           = 16;   // Queue size for the AD741x device
inline constexpr uint8_t   fault_count          = 6;    // Maximum number of no data, before faults

static AD741x::Form        top_temperature_array[form_depth]    = { 0 };
static AD741x::Form        bottom_temperature_array[form_depth] = { 0 };

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

//=================================================================================================

/**
  * @brief:  Create the Top Internal Temperature Sensor class as per attached device
  * @param:  void const, not used
  * @retval: AD741x class type
  */
AD741x  generateInternalTopTemperatureSensor(void) {
    AD741x  external_device(AD741x::DevPart::kAD7415_1,
                            AD741x::AddrBit::kVdd,
                            &top_temperature_array[0],
                            form_depth);

    return (  external_device  );
}

/**
  * @brief:  Create the Bottom Internal Temperature Sensor class as per attached device
  * @param:  void const, not used
  * @retval: AD741x class type
  */
AD741x  generateInternalBottomTemperatureSensor(void) {
    AD741x  external_device(AD741x::DevPart::kAD7415_1,
                            AD741x::AddrBit::kGnd,
                            &bottom_temperature_array[0],
                            form_depth);

    return (  external_device  );
}

/**
  * @brief:  Will request multiple communication session with temperature sensor, and will be
  *          queued into the I2C1 device
  * @param:  I2C Peripheral pointer
  * @param:  AD741x device pointer
  * @param:  GenBuffer pointer to the generic write buffer
  * @param:  GenBuffer pointer to the AD741x specific read buffer
  * @retval: None (void output)
  */
static void temperatureSensorPacketSetup(I2CPeriph *hi2c, AD741x *device,
                                         uint8_t *wrBuff, uint8_t *rdBuff)
{
    // Clear communication count fault flags
    device->wrte_cmp_target     = 0;        // Clear the communication write target count
    device->read_cmp_target     = 0;        // Clear the communication  read target count

    device->wrte_cmp_flg        = 0;        // Clear the communication write flag (actual
                                            // count)
    device->read_cmp_flag       = 0;        // Clear the communication  read flag (actual
                                            // count)

    device->intConfigWrite(hi2c,                            // Construct a write request to AD7415
                           AD741x::PwrState::kStand_By,     // device, putting it into 'Standby'
                                                            // mode
                           AD741x::FiltState::kEnabled,     // Filter is enabled
                           AD741x::OneShot::kTrigConv,       // and trigger a conversion request
                           // (in 'Standby' conversion will only occur upon request)
                           wrBuff);   // Provide pointer to array/buffer to
                                                            // request data from

    device->intConfigRead(hi2c, rdBuff, wrBuff);        // Read back the status of AD7415
                                                        // configuration
    device->intTempRead(hi2c, rdBuff, wrBuff);          // Read back temperature
}

/**
  * @brief:  Determines whether there is new data available to be decoded from the temperature
  *          sensor. Will also determine fault status of device communication.
  * @param:  I2C Peripheral pointer
  * @param:  AD741x device pointer
  * @param:  GenBuffer pointer to the generic write buffer
  * @param:  GenBuffer pointer to the AD741x specific read buffer
  * @param:  Pointer to the DevComFlt specific for the AD741x device, so includes the AD741x
  *          fault type, and I2C Peripheral fault type
  * @retval: None (void output)
  */
void internalTemperatureSensorManagement(I2CPeriph *hi2c, AD741x *device,
                        uint8_t *wrBuff, uint8_t *rdBuff,
                        _ihal::DevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt> *CommState)
{
    /* AD7415_1 I2C communication interrupt management steps:
     * ***********
     *  1 =     If there is no fault with the I2C interrupt communication, and the correct number
     *          of bytes/packets have been transmitted, then decode data and bring contents into
     *          the AD741x class type. No faults triggered, and all communication counts cleared.
     *          Next communication is organised, and requested of the hardware.
     *
     *  2 =     If there is no fault with the I2C interrupt communication, and the correct number
     *          of bytes/packets have been transmitted, then de-code data and bring contents into
     *          the AD741x class type. If the class determines a fault with the data, then
     *          indicate that the external device is faulty. Re-initialise the AD741x class type
     *          internals, and organised a new communication - and sent request to hardware.
     *
     *  3 =     If there is no fault with the I2C interrupt communication, and the correct number
     *          of bytes/packets have NOT been transmitted. Then increment a count of "blackout"
     *          iterations.
     *          If this count exceeds the threshold 'I2C1_FaultCount', then indicate a device no
     *          communication fault.
     *
     *  4 =     If there is a I2C interrupt communication fault, then indicate a I2C communication
     *          fault, setup device fault as parent comm fault.
     *          Re-initialise the AD741x class type internals, clear the I2C interrupt
     *          communication fault, and send a new transmit request to hardware - in effort to
     *          re-establish communication.
     *********************************************************************************************/
    if ( (device->i2c_wrte_flt == I2CPeriph::DevFlt::kNone) &&
         (device->i2c_read_flt == I2CPeriph::DevFlt::kNone) ) { // If there is no fault with the
                                                                // I2C communication then...
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        if ( (device->read_cmp_target == device->read_cmp_flag) &&
             (device->wrte_cmp_target == device->wrte_cmp_flg) ) {  // If the expected number of
                                                                    // bytes been written/read then
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            device->intCheckCommStatus(&rdBuff[0], device->read_cmp_target);
                // Read all available from selected AD741x device

            // Set device communication state to None for I2C communication fault, and device
            // fault to be equal to output of AD741x class
            CommState->ComFlt   = I2CPeriph::DevFlt::kNone;
            CommState->DevFlt   = device->flt;

            if (device->flt != AD741x::DevFlt::kNone)   // If there is a detected fault with the
                                                        // device, then
                device->reInitialise();                 // Re-initialise the class internals

            CommState->IdleCount    = 0;                // Clear communication "blackout" count

            temperatureSensorPacketSetup(hi2c, device, &wrBuff[0], &rdBuff[0]);
            // Setup a new set of packets to transmit.
        }
        else {  // If the number of bytes have not been written/read yet, then...
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            if (CommState->IdleCount >= fault_count) {      // If the count exceeds the limit,
                                                            // then
                CommState->DevFlt   = AD741x::DevFlt::kNo_Communication;
                // Indicate that there is no communication with device
            }
            else                            // Only if the counter is less then threshold
                CommState->IdleCount++;     // Increment "blackout" counter
        }
    }
    else {  // If there is a detected bus (I2C) fault during communication then
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        if (device->i2c_read_flt != I2CPeriph::DevFlt::kNone)   // If there is a read fault
            CommState->ComFlt = device->i2c_read_flt;           // then capture
        else                                                    // If the read is fault free
            CommState->ComFlt = device->i2c_wrte_flt;           // then captured write fault

        CommState->DevFlt   = AD741x::DevFlt::kParent_Communication_Fault;
                    // Indicate parent comm fault

        device->reInitialise();                 // Reinitialise the internal class

        device->i2c_read_flt    = I2CPeriph::DevFlt::kNone;     // Clear communication faults
        device->i2c_wrte_flt    = I2CPeriph::DevFlt::kNone;     // Clear communication faults

        temperatureSensorPacketSetup(hi2c, device, &wrBuff[0], &rdBuff[0]);
        // Setup a new set of packets to transmit.
        // This is to allow for any temporary hardware fault to be cleared, and communication to
        // be re-established
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
// None
}
