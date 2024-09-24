// common_types.h
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-19
// Last modified: 2024-09-24 by Carl Larsson
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

// Struct describing the playing field dimensions
struct PlayingField
{
    static constexpr double kFrameX = 9600.0;
    static constexpr double kFrameY = 6600.0;
    static constexpr double kTouchLineX = 9000.0;
    static constexpr double kGoalLineY = 6000.0;
    static constexpr double kDefenseAreaX = 1000.0;
    static constexpr double kDefenseAreaY = 2000.0;
    static constexpr double kCenterCircleRadius = 500.0;
    static constexpr double kGoalY = 1000.0;
};

//==============================================================================

} // namespace robot_controller_interface

#endif // ROBOTCONTROLLERINTERFACE_COMMONTYPES_H_
