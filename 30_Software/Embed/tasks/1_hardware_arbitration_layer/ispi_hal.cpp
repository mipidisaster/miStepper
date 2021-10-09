/**************************************************************************************************
 * @file        ispi_hal.cpp
 * @author      Thomas
 * @brief       HAL interface layer for the SPI task (source)
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/1_hardware_arbitration_layer/ispi_hal.hpp"
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
#include FilInd_SPIPe__HD               // Include the SPI class handler
#include FilInd_AS5x4x_HD               // Include the device AS5x4 handler

//=================================================================================================

namespace _ihal::_ispi1 {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
Semaphore<  Param<float>  > ang_pos_raw = {
        .content = { 0 },
        .lock    = LockState::kLocked
};

Semaphore< SPIPeriph::DevFlt > comm_flt = {
        .content = SPIPeriph::DevFlt::kInitialised,
        .lock    = LockState::kLocked
};

Semaphore< DevComFlt<AS5x4x::DevFlt, SPIPeriph::DevFlt> > angle_sensor_flt = {
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
  * @brief:  Initialises all the SPI interface hal parameters.
  * @param:  void
  * @retval: void
  */
void interfaceInitialise(void) {
    ang_pos_raw.content.data    = -999.0f;
    ang_pos_raw.content.flt     = FltState::kFaulty;
    ang_pos_raw.lock            = LockState::kUnlocked;

    comm_flt.content            = SPIPeriph::DevFlt::kInitialised;
    comm_flt.lock               = LockState::kUnlocked;

    angle_sensor_flt.content.IdleCount  = 0;
    angle_sensor_flt.content.DevFlt     = AS5x4x::DevFlt::kInitialised;
    angle_sensor_flt.content.ComFlt     = SPIPeriph::DevFlt::kInitialised;
    angle_sensor_flt.lock               = LockState::kUnlocked;
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
