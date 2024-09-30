// ball.h
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-22
// Last modified: 2024-09-30 by Carl Larsson
// Description: Header file for everything that relates to the ball.
// License: See LICENSE file for license details.
//==============================================================================


#ifndef ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_BALL_H_
#define ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_BALL_H_


// Related .h files

// C++ standard library headers
#include <atomic>

// Other .h files

// Project .h files


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{

//==============================================================================

// Used to indicate who had dwb do work
extern std::atomic_bool atomic_shoot_setup_work;

//==============================================================================

} // namespace individual_robot_behaviour
} // namesapce robot_controller_interface

#endif // ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_BALL_H_
