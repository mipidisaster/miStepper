/**************************************************************************************************
 * @file        istepper_hal.cpp
 * @author      Thomas
 * @brief       HAL interface layer for the Stepper task (source)
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/1_hardware_arbitration_layer/istepper_hal.hpp"
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

//=================================================================================================

namespace _ihal::_istepper {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
Semaphore< uint8_t > enable = {
        .content = { 0 },
        .lock    = LockState::kLocked
};

Semaphore< uint8_t > microstep = {
        .content = { 0 },
        .lock    = LockState::kLocked
};

Semaphore< uint8_t > direction = {
        .content = { 0 },
        .lock    = LockState::kLocked
};

Semaphore<uint16_t > frequency = {
        .content = { 0 },
        .lock    = LockState::kLocked
};

Semaphore<uint32_t > calc_position = {
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
  * @brief:  Initialises all the Stepper hal parameters.
  * @param:  void
  * @retval: void
  */
void interfaceInitialise(void) {
    enable.content          = 0;
    enable.lock             = LockState::kUnlocked;

    microstep.content       = 0;
    microstep.lock          = LockState::kUnlocked;

    direction.content       = 0;
    direction.lock          = LockState::kUnlocked;

    frequency.content       = 0;
    frequency.lock          = LockState::kUnlocked;

    calc_position.content   = 0;
    calc_position.lock      = LockState::kUnlocked;
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
