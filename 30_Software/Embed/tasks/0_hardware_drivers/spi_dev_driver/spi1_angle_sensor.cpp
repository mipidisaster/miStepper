/**************************************************************************************************
 * @file        spi1_angle_sensor.cpp
 * @author      Thomas
 * @brief       Source file for SPI angle sensor
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/spi_dev_driver/spi1_angle_sensor.hpp"
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
#include "tasks/0_hardware_drivers/spi_dev_driver/spi1_dev_driver_parameters.hpp"

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "FileIndex.h"
//~~~~~~~~~~~~~~~~~~~~
#include FilInd_SPIPe__HD               // Include the SPI class handler
#include FilInd_AS5x4x_HD               // Include the device AS5x4 handler

//=================================================================================================

namespace _spi1_dev::_ext_angle {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
inline constexpr uint8_t   form_depth           = 16;   // Queue size for the AS5047 device
inline constexpr uint8_t   fault_count          = 6;    // Maximum number of no data, before faults

static uint16_t sensor_array[2][form_depth] = { 0 };    // Form Array for angle sensor

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

//=================================================================================================

/**
  * @brief:  Create the Angle Sensor class as per attached device
  * @param:  void const, not used
  * @retval: AS5x4x class type
  */
AS5x4x  generateAngleSensor(void) {
    AS5x4x  external_device(AS5x4x::DevPart::kAS5047D,
                            &sensor_array[_param::kwrte_loc][0],
                            &sensor_array[_param::kread_loc][0],
                            form_depth);                              // State form size

    return (  external_device  );
}

/**
  * @brief:  Will request multiple communication sessions with Angle Sensor, and will be queued
  *          into the SPI1 device
  * @param:  SPI Peripheral pointer
  * @param:  AS5x4x device pointer
  * @param:  Daisy chain structure pointer
  * @param:  Chip Select GPIO pointer
  * @param:  Array pointer to where data is read from and written to, expects this is an array
  *          with 'SPI1_CommBuff' columns
  * @retval: None (void output)
  */
static void angleSensorPacketSetup(SPIPeriph *hspi, AS5x4x *device, AS5x4x::Daisy *daisy, GPIO *CS,
                                 uint8_t *wtBuff, uint8_t *rdBuff)
{
    device->constructCEF();         // Construct the Clear Error Flags request
    device->constructAGC();         // Construct the Automatic Gain Control request
    device->constructMag();         // Construct the CORDIC Magnitude
    device->constructAng();         // Construct the Angular position
    device->constructNOP();         // Construct NOP

    // Clear communication count fault flags
    daisy->Trgt     = 0;                    // Clear the communication target count
    daisy->Cmplt    = 0;                    // Clear the communication actual count

    AS5x4x::intSingleTransmit(hspi,                 // Add data requests to target SPI peripheral
                              CS,                   // Chip Select to use
                              daisy,                // Daisy Chain setup
                              rdBuff,               // Pointer to where read back data is to be
                                                    // stored
                              wtBuff);              // Pointer to where data to be written to
                                                    // AS5048 is to be taken from.
}

/**
  * @brief:  Determines whether there is new data available to be decoded from the Angular sensor
  *          device. Will also determine fault status of device communication.
  * @param:  SPI Peripheral pointer
  * @param:  AS5x4x device pointer
  * @param:  Daisy chain structure pointer
  * @param:  Chip Select GPIO pointer
  * @param:  Array pointer to where data is read from and written to, expects this is an array
  *          with 'SPI1_CommBuff' columns
  * @param:  Pointer to the DevComFlt specific for the AS5x4x device, so includes the AS5x4x
  *          fault type, and SPI Peripheral fault type
  * @retval: None (void output)
  */
void angleSensorMangement(SPIPeriph *hspi, AS5x4x *device, AS5x4x::Daisy *daisy, GPIO *CS,
                          uint8_t *wtBuff, uint8_t *rdBuff,
                          _ihal::DevComFlt<AS5x4x::DevFlt, SPIPeriph::DevFlt> *CommState)
{
    /* AS504x SPI communication interrupt management steps:
     * ***********
     *  1 =     If there is no fault with the SPI interrupt communication, and the correct number
     *          of bytes/packets have been transmitted, then decode data and bring contents into
     *          the AS5x4x class type. No faults triggered, and all communication counts cleared.
     *          Next communication is organised, and requested of the hardware.
     *
     *  2 =     If there is no fault with the SPI interrupt communication, and the correct number
     *          of bytes/packets have been transmitted, then de-code data and bring contents into
     *          the AS5x4x class type. If the class determines a fault with the data, then
     *          indicate that the external device is faulty. Re-initialise the AS5x4x class type
     *          internals, and organised a new communication - and sent request to hardware.
     *
     *  3 =     If there is no fault with the SPI interrupt communication, and the correct number
     *          of bytes/packets have NOT been transmitted. Then increment a count of "blackout"
     *          iterations.
     *          If this count exceeds the threshold 'SPI1_FaultCount', then indicate a device no
     *          communication fault.
     *
     *  4 =     If there is a SPI interrupt communication fault, then indicate a SPI communication
     *          fault, setup device fault as parent comm fault.
     *          Re-initialise the AS5x4x class type internals, clear the SPI interrupt
     *          communication fault, and send a new transmit request to hardware - in effort to
     *          re-establish communication.
     *********************************************************************************************/
    if (daisy->Flt == SPIPeriph::DevFlt::kNone) {   // If there is no fault with the SPI
                                                    // communication then...
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        if ( daisy->Trgt == daisy->Cmplt ) {            // If the expected number of bytes have
                                                        // been transmitted then
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            AS5x4x::readSPIChain(daisy->Devices,        // Read all available data from every
                                 daisy->numDevices,     // AS5x4x supported device within daisy
                                 &rdBuff[0],            // chain.
                                 daisy->Trgt);
            //####
            // Currently only supports use of 1 connected AS5x4x device.
            //####
            AS5x4x::readDaisyPackets(daisy);            // Deconstruct all data, and integrate
                                                        // into each AS5x4x class instance.

            // Set device communication state to None for SPI communication fault, and device
            // fault to be equal to output of AS5x4x class
            CommState->ComFlt   = SPIPeriph::DevFlt::kNone;
            CommState->DevFlt   = device->flt;

            if (device->flt != AS5x4x::DevFlt::kNone)   // If there is a detected fault with the
                                                        // device, then
                device->reInitialise();                 // Re-initialise the class internals

            CommState->IdleCount    = 0;                // Clear communication "blackout" count

            angleSensorPacketSetup(hspi, device, daisy, CS, wtBuff, rdBuff);
            // Setup a new set of packets to transmit.
        }
        else {  // If the number of bytes have not been transmitted yet, then...
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            if (CommState->IdleCount >= fault_count) {      // If the count exceeds the limit,
                                                            // then
                CommState->DevFlt   = AS5x4x::DevFlt::kNo_Communication;
                // Indicate that there is no communication with device
            }
            else                            // Only if the counter is less then threshold
                CommState->IdleCount++;     // Increment "blackout" counter
        }
    }
    else {  // If there is a detected bus (SPI) fault during communication then
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        CommState->ComFlt   = daisy->Flt;       // Copy across the communication fault
        CommState->DevFlt   = AS5x4x::DevFlt::kParent_Comm_Flt; // Indicate parent comm fault

        device->reInitialise();                 // Reinitialise the internal class

        daisy->Flt          = SPIPeriph::DevFlt::kNone;         // Clear the daisy chain
                                                                // communication fault
        angleSensorPacketSetup(hspi, device, daisy, CS, wtBuff, rdBuff);
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
