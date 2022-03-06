/**************************************************************************************************
 * @file        i2c1_motor_temperature.cpp
 * @author      Thomas
 * @brief       Source file for I2C motor temperature sensor
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/i2c_dev_driver/i2c1_motor_temperature.hpp"
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

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "tasks/1_hardware_arbitration_layer/ii2c_hal.hpp"

#include "FileIndex.h"
//~~~~~~~~~~~~~~~~~~~~
#include FilInd_I2CPe__HD               // Include the I2C class handler
#include FilInd_BME280_HD               // Header for BME280 Device
#include FilInd_BMEI2C_HD               // Header for the BME280 I2C interface

//=================================================================================================

namespace _i2c1_dev::_motor_temp {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
static BME280::Form         right_temperature_array[_param::kmotor_form_depth]  = { 0 };
static uint8_t              right_buff[2][_param::kmotor_buff_size]             = { 0 };

static BME280::Form         left_temperature_array[_param::kmotor_form_depth]   = { 0 };
static uint8_t              left_buff[2][_param::kmotor_buff_size]              = { 0 };

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

//=================================================================================================

/**
  * @brief:  Create the Right Motor Temperature Sensor class as per attached device
  * @param:  void const, not used
  * @retval: BME280I2C class type
  */
BME280I2C   generateMotorRightTemperatureSensor(void) {
    BME280I2C  external_device(BME280I2C::AddrBit::kGnd,
                               &right_temperature_array[0],
                               _param::kmotor_form_depth);

    return (  external_device  );
}

/**
  * @brief:  Create the Left Motor Temperature Sensor class as per attached device
  * @param:  void const, not used
  * @retval: BME280I2C class type
  */
BME280I2C   generateMotorLeftTemperatureSensor(void) {
    BME280I2C  external_device(BME280I2C::AddrBit::kVdd,
                               &left_temperature_array[0],
                               _param::kmotor_form_depth);

    return (  external_device  );
}

/**
  * @brief:  Will request multiple communication session with temperature sensor, and will be
  *          queued into the I2C1 device
  * @param:  I2C Peripheral pointer
  * @param:  BME280I2C device pointer
  * @param:  GenBuffer pointer to the generic write buffer
  * @param:  GenBuffer pointer to the BME280I2C specific read buffer
  * @retval: None (void output)
  */
static void temperatureSensorPacketSetup(I2CPeriph *hi2c, BME280I2C *device,
                                         uint8_t *wrBuff, uint8_t *rdBuff,
                                         uint8_t *state_controller)
{
    // Clear communication count fault flags
    device->clearCommunicationCount();


    if (device->deviceCalibrated() != 1) {                  // If device calibration data has not
        device->intCalibrateRead(hi2c, rdBuff, wrBuff);     // been read, then read it
        *state_controller   = 0;
    }
    else {
        if (*state_controller == 0) {
            device->intRecommendedConfigIndoor(hi2c, wrBuff);
            device->intConfigRead(hi2c, rdBuff, wrBuff);
            *state_controller = 1;
        }
        else {
            device->intSensorRead(hi2c, rdBuff, wrBuff);    // Otherwise read sensor data
            *state_controller = 2;
        }
    }
}

/**
  * @brief:  Determines whether there is new data available to be decoded from the temperature
  *          sensor. Will also determine fault status of device communication.
  * @param:  I2C Peripheral pointer
  * @param:  BME280I2C device pointer
  * @param:  GenBuffer pointer to the generic write buffer
  * @param:  GenBuffer pointer to the BME280I2C specific read buffer
  * @param:  Pointer to the DevComFlt specific for the BME280I2C device, so includes the BME280I2C
  *          fault type, and I2C Peripheral fault type
  * @retval: None (void output)
  */
void internalTemperatureSensorManagement(I2CPeriph *hi2c, BME280I2C *device,
                        uint8_t *wrBuff, uint8_t *rdBuff,
                        _ihal::DevComFlt<BME280::DevFlt, I2CPeriph::DevFlt> *CommState,
                        uint8_t *state_controller)
{
    /* BME280 I2C communication interrupt management steps:
     * ***********
     *  1 =     If there is no fault with the I2C interrupt communication, and the correct number
     *          of bytes/packets have been transmitted, then decode data and bring contents into
     *          the BME280 class type. No faults triggered, and all communication counts cleared.
     *          Next communication is organised, and requested of the hardware.
     *
     *  2 =     If there is no fault with the I2C interrupt communication, and the correct number
     *          of bytes/packets have been transmitted, then de-code data and bring contents into
     *          the BME280 class type. If the class determines a fault with the data, then
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
     *          Re-initialise the BME280 class type internals, clear the I2C interrupt
     *          communication fault, and send a new transmit request to hardware - in effort to
     *          re-establish communication.
     *********************************************************************************************/
    if ( (device->i2c_wrte_flt == I2CPeriph::DevFlt::kNone) &&
         (device->i2c_read_flt == I2CPeriph::DevFlt::kNone) ) { // If there is no fault with the
                                                                // I2C communication then...
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        if ( device->intCheckCommStatus(&rdBuff[0]) ) {         // If the expected number of bytes
                                                                // been written/read then
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Check to see what the state of the data read is...
            if (*state_controller  ==  2) {
                device->calculateSensorReadings();
            }


            // Set device communication state to None for I2C communication fault, and device
            // fault to be equal to output of AD741x class
            CommState->ComFlt   = I2CPeriph::DevFlt::kNone;
            CommState->DevFlt   = device->flt;

            if (device->flt != BME280::DevFlt::kNone)// If there is a detected fault with the
                                                        // device, then
                device->reInitialise();                 // Re-initialise the class internals

            CommState->IdleCount    = 0;                // Clear communication "blackout" count

            temperatureSensorPacketSetup(hi2c, device, &wrBuff[0], &rdBuff[0], state_controller);
            // Setup a new set of packets to transmit.
        }
        else {  // If the number of bytes have not been written/read yet, then...
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            if (CommState->IdleCount >= _param::kfault_count) { // If the count exceeds the limit,
                                                                // then
                CommState->DevFlt   = BME280::DevFlt::kNo_Communication;
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

        CommState->DevFlt   = BME280::DevFlt::kParent_Communication_Fault;
                    // Indicate parent comm fault

        device->reInitialise();                 // Reinitialise the internal class

        device->i2c_read_flt    = I2CPeriph::DevFlt::kNone;     // Clear communication faults
        device->i2c_wrte_flt    = I2CPeriph::DevFlt::kNone;     // Clear communication faults

        temperatureSensorPacketSetup(hi2c, device, &wrBuff[0], &rdBuff[0], state_controller);
        // Setup a new set of packets to transmit.
        // This is to allow for any temporary hardware fault to be cleared, and communication to
        // be re-established
    }
}

/**
  * @brief:  Determines whether there is new data available to be decoded from the temperature
  *          (right) sensor. Will also determine fault status of device communication.
  * @param:  I2C Peripheral pointer
  * @param:  BME280I2C device pointer
  * @param:  Pointer to the DevComFlt specific for the BME280 device, so includes the BME280
  *          fault type, and I2C Peripheral fault type
  * @param:  first_pass of the I2C task handler
  * @retval: None (void output)
  */
void    manageMotorRightTemperatureSensor(I2CPeriph *hi2c, BME280I2C *right_device,
        _ihal::DevComFlt<BME280::DevFlt, I2CPeriph::DevFlt> *CommState, uint8_t *first_pass)
{
    static uint8_t state_controller = 0;

    internalTemperatureSensorManagement(
            hi2c,
            right_device,
            &right_buff[_param::kwrte_loc][0],
            &right_buff[_param::kread_loc][0],
            CommState,
            &state_controller);

    if ((CommState->DevFlt  != BME280::DevFlt::kNone) || (*first_pass == 1)) {
        // If the BME280 device/communication is faulty, OR it has been the first pass through
        // of task then
        _ihal::setFault(&_ihal::_ii2c1::right_motor_temp_raw);
        _ihal::setFault(&_ihal::_ii2c1::right_motor_pres_raw);
        _ihal::setFault(&_ihal::_ii2c1::right_motor_humd_raw);
    }
    else
    {   // Otherwise data is good, and this is not the first pass
        _ihal::pushValue(&_ihal::_ii2c1::right_motor_temp_raw,
                         right_device->temperature);
        _ihal::pushValue(&_ihal::_ii2c1::right_motor_pres_raw,
                         right_device->pressure);
        _ihal::pushValue(&_ihal::_ii2c1::right_motor_humd_raw,
                         right_device->humidity);

        _ihal::clearFault(&_ihal::_ii2c1::right_motor_temp_raw);
        _ihal::clearFault(&_ihal::_ii2c1::right_motor_pres_raw);
        _ihal::clearFault(&_ihal::_ii2c1::right_motor_humd_raw);
    }
}

/**
  * @brief:  Determines whether there is new data available to be decoded from the temperature
  *          (left) sensor. Will also determine fault status of device communication.
  * @param:  I2C Peripheral pointer
  * @param:  BME280I2C device pointer
  * @param:  Pointer to the DevComFlt specific for the BME280 device, so includes the BME280
  *          fault type, and I2C Peripheral fault type
  * @param:  first_pass of the I2C task handler
  * @retval: None (void output)
  */
void    manageMotorLeftTemperatureSensor(I2CPeriph *hi2c, BME280I2C *left_device,
        _ihal::DevComFlt<BME280::DevFlt, I2CPeriph::DevFlt> *CommState, uint8_t *first_pass)
{
    static uint8_t state_controller = 0;

    internalTemperatureSensorManagement(
            hi2c,
            left_device,
            &left_buff[_param::kwrte_loc][0],
            &left_buff[_param::kread_loc][0],
            CommState,
            &state_controller);

    if ((CommState->DevFlt  != BME280::DevFlt::kNone) || (*first_pass == 1)) {
        // If the BME280 device/communication is faulty, OR it has been the first pass through
        // of task then
        _ihal::setFault(&_ihal::_ii2c1::left_motor_temp_raw);
        _ihal::setFault(&_ihal::_ii2c1::left_motor_pres_raw);
        _ihal::setFault(&_ihal::_ii2c1::left_motor_humd_raw);
    }
    else
    {   // Otherwise data is good, and this is not the first pass
        _ihal::pushValue(&_ihal::_ii2c1::left_motor_temp_raw,
                         left_device->temperature);
        _ihal::pushValue(&_ihal::_ii2c1::left_motor_pres_raw,
                         left_device->pressure);
        _ihal::pushValue(&_ihal::_ii2c1::left_motor_humd_raw,
                         left_device->humidity);

        _ihal::clearFault(&_ihal::_ii2c1::left_motor_temp_raw);
        _ihal::clearFault(&_ihal::_ii2c1::left_motor_pres_raw);
        _ihal::clearFault(&_ihal::_ii2c1::left_motor_humd_raw);
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
