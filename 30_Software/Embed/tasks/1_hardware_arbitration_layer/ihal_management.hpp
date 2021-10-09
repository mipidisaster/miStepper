/**************************************************************************************************
 * @file        ihal_mangement.h
 * @author      Thomas
 * @brief       Hardware Arbitration Layer Interface header
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This Header contains the template types and function calls to handle the parameters which will
 * be stored within the interface -> HAL.
 * All parameters within the HAL, will be contained within the namespace - "_ihal".
 *
 * All parameters within the HAL, will also be captured within a Semaphore, such that any writing
 * to this interface via multiple threads at the same time, won't result in any errors.
 *
 * To write to parameters in the HAL, the following functions need to be used:
 *      pushValue(Semaphore<datanode>, <datanode>)
 *      pushValue(Semaphore<Param<paramtype>>, <paramtype>)
 *          These two functions are the same (albeit overloaded), to push data to the interface.
 *          The first type puts the contents of datanode, into the Semaphore directly, and is
 *          intended for fault flags.
 *          The second type, is to be used for 'Param<paramtype>' parameters only, as this will
 *          write to the data directly:
 *              i.e. -> Param<float>, will receive a float input
 *
 *      setFault/clearFault -> These functions only work with Param<paramtype> Semaphores (as
 *                             above).
 *
 *      getValue() -> This works the same way as the pushValue function
 *      getFault()
 *************************************************************************************************/
#ifndef IHAL_MANAGEMENT_H_
#define IHAL_MANAGEMENT_H_

/**************************************************************************************************
 * Include all files that are needed to understand this header
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
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
// None

//=================================================================================================

namespace _ihal {
/**************************************************************************************************
 * Exported MACROS
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported Variables
 * ~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
inline constexpr uint8_t    kwrte_loc           = 0;    // Array position for Write

/**************************************************************************************************
 * Exported types
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
enum LockState: uint8_t {kLocked  = 0x5A, kUnlocked = 0x01};
enum FltState : uint8_t {kNoFault = 0x00, kFaulty = 0x5F};

enum TimedTasks : uint8_t {
    kUSART1     = 0x00,
    kSPI1       = 0x01, //
    kI2C1       = 0x02, //
    kADC1       = 0x03, //
    kDAC1       = 0x04, //
    kFAN        = 0x05, //
    kSTEPPER    = 0x06, //

    kMAX_LIMIT  = 0x07
};

template <typename datavalue>
struct Param {
    datavalue       data;
    enum FltState   flt;
};

// Following structure defines a global type for containing the status of various
// devices/communication systems used within device
template <typename Dev, typename Comm>
struct DevComFlt {
    uint8_t     IdleCount;

    Dev         DevFlt;
    Comm        ComFlt;
};

template <typename datanode>
struct Semaphore {
    datanode           content;
    enum LockState      lock;
};

/**************************************************************************************************
 * Exported function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
/**
  * @brief:  Write value to the HAL Parameter, however prior to doing this check that the parameter
  *          is not already locked by another system/task
  * @param:  Template Semaphore pointer, and the new value which needs to be the same type as
  *          Semaphore
  * @retval: void
  */
template <typename datanode>
void pushValue(Semaphore<datanode> *hal_param, datanode new_value) {
    if (hal_param->lock == LockState::kLocked) { return; }

    hal_param->lock = LockState::kLocked;

    hal_param->content = new_value;

    hal_param->lock = LockState::kUnlocked;
}

/**
  * @brief:  Write value to the HAL Parameter, however prior to doing this check that the parameter
  *          is not already locked by another system/task
  * @param:  Semaphore of a Param<paramtype> pointer, and the new value (so needs of type
  *          'paramtype', as this will go into the '.data' of Param)
  * @retval: void
  */
template <typename paramtype>
void pushValue(Semaphore< Param<paramtype> > *hal_param, paramtype new_value) {
    if (hal_param->lock == LockState::kLocked) { return; }

    hal_param->lock = LockState::kLocked;

    hal_param->content.data = new_value;

    hal_param->lock = LockState::kUnlocked;
}

/**
  * @brief:  Set the fault flag for the HAL parameter (Semaphore, of type Param<paramtype>)
  * @param:  Template Semaphore<Param> pointer
  * @retval: void
  */
template <typename paramtype>
void setFault(Semaphore< Param<paramtype> > *hal_param) {
    if (hal_param->lock == LockState::kLocked) { return; }

    hal_param->lock = LockState::kLocked;

    hal_param->content.flt = FltState::kFaulty;

    hal_param->lock = LockState::kUnlocked;
}

/**
  * @brief:  Clear the fault flag for the HAL parameter (Semaphore, of type Param<paramtype>)
  * @param:  Template Semaphore<Param> pointer
  * @retval: void
  */
template <typename paramtype>
void clearFault(Semaphore< Param<paramtype> > *hal_param) {
    if (hal_param->lock == LockState::kLocked) { return; }

    hal_param->lock = LockState::kLocked;

    hal_param->content.flt = FltState::kNoFault;

    hal_param->lock = LockState::kUnlocked;
}

/**
  * @brief:  Retrieve the current value of the HAL Parameter
  * @param:  Template Semaphore pointer, to read the data from
  * @retval: Template datanode (i.e. the type of the Semaphore)
  */
template <typename datanode>
datanode getValue(Semaphore<datanode> *hal_param) {
    return (  hal_param->content  );
}

/**
  * @brief:  Retrieve the current value of the HAL Parameter
  * @param:  Template Semaphore<Param> pointer, to read the data from
  * @retval: Template paramtype (i.e. the same type as the '.data' from the Param type)
  */
template <typename paramtype>
paramtype getValue(Semaphore< Param<paramtype> > *hal_param) {
    return (  hal_param->content.data  );
}

/**
  * @brief:  Retrieve the current fault state of the HAL Parameter
  * @param:  Template Semaphore<Param>, to read the fault data from
  * @retval: FltState
  */
template <typename paramtype>
FltState getFault(Semaphore< Param<paramtype> > *hal_param) {
    return (  hal_param->content.flt  );
}

}
#endif /* IHAL_MANAGEMENT_H_ */
