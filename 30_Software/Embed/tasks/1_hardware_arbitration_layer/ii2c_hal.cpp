/**************************************************************************************************
 * @file        ii2c_hal.cpp
 * @author      Thomas
 * @brief       HAL interface layer for the I2C task (source)
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/1_hardware_arbitration_layer/ii2c_hal.hpp"
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
#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "FileIndex.h"
//~~~~~~~~~~~~~~~~~~~~
#include FilInd_I2CPe__HD               // Include the I2C class handler
#include FilInd_AD741x_HD               // Include the device AD741x handler

//=================================================================================================

namespace _ihal::_ii2c1 {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
Semaphore<  Param<float>  > internal_top_temp_raw = {
        .content = { 0 },
        .lock    = LockState::kLocked
};

Semaphore<  Param<float>  > internal_bottom_temp_raw = {
        .content = { 0 },
        .lock    = LockState::kLocked
};

Semaphore< I2CPeriph::DevFlt > comm_flt = {
        .content = I2CPeriph::DevFlt::kInitialised,
        .lock    = LockState::kLocked
};

Semaphore< DevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt> > top_temp_sensor_flt = {
        .content = { 0 },
        .lock    = LockState::kLocked
};

Semaphore< DevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt> > bottom_temp_sensor_flt = {
        .content = { 0 },
        .lock    = LockState::kLocked
};

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

//=================================================================================================

/**
  * @brief:  Initialises all the I2C interface hal parameters.
  * @param:  void
  * @retval: void
  */
void interfaceInitialise(void) {
    internal_top_temp_raw.content.data      = -999.0f;
    internal_top_temp_raw.content.flt       = FltState::kFaulty;
    internal_top_temp_raw.lock              = LockState::kUnlocked;

    internal_bottom_temp_raw.content.data   = -999.0f;
    internal_bottom_temp_raw.content.flt    = FltState::kFaulty;
    internal_bottom_temp_raw.lock           = LockState::kUnlocked;

    comm_flt.content            = I2CPeriph::DevFlt::kInitialised;
    comm_flt.lock               = LockState::kUnlocked;

    top_temp_sensor_flt.content.IdleCount   = 0;
    top_temp_sensor_flt.content.DevFlt      = AD741x::DevFlt::kInitialised;
    top_temp_sensor_flt.content.ComFlt      = I2CPeriph::DevFlt::kInitialised;
    top_temp_sensor_flt.lock                = LockState::kUnlocked;

    bottom_temp_sensor_flt.content.IdleCount= 0;
    bottom_temp_sensor_flt.content.DevFlt   = AD741x::DevFlt::kInitialised;
    bottom_temp_sensor_flt.content.ComFlt   = I2CPeriph::DevFlt::kInitialised;
    bottom_temp_sensor_flt.lock             = LockState::kUnlocked;
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
