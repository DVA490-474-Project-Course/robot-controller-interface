// common_types.h
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-19
// Last modified: 2024-09-21 by Carl Larsson
// Description: Common types used by the individual robot behaviour program.
// License: See LICENSE file for license details.
//==============================================================================


#ifndef ROBOTCONTROLLERINTERFACE_COMMONTYPES_H_
#define ROBOTCONTROLLERINTERFACE_COMMONTYPES_H_


// Related .h files

// C++ standard library headers

// Other .h files

// Project .h files


namespace robot_controller_interface
{

//==============================================================================

// Enum class describing the playing field dimensions
enum class PlayingField
{
    kFrameX = 9600,
    kFrameY = 6600,
    kTouchLineX = 9000,
    kGoalLineY = 6000,
    kDefenseAreaX = 1000,
    kDefenseAreaY = 2000,
    kCenterCircleRadius = 500,
    kGoalY = 1000
};

//==============================================================================

} // namespace robot_controller_interface

#endif // ROBOTCONTROLLERINTERFACE_COMMONTYPES_H_
