// path_planning.h
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-19
// Last modified: 2024-09-19 by Carl Larsson
// Description: Path planning header file.
// License: See LICENSE file for license details.
//==============================================================================

#ifndef ROBOTCONTROLLERINTERFACE_PATHPLANNING_PATHPLANNING_H_
#define ROBOTCONTROLLERINTERFACE_PATHPLANNING_PATHPLANNING_H_

namespace robot_controller_interface
{
namespace path_planning
{

// Struct for a location in 2D space.
struct Position
{
    int x;
    int y;
};

} // namespace path_planning
} // namespace robot_controller_interface

#endif // ROBOTCONTROLLERINTERFACE_PATHPLANNING_PATHPLANNING_H_
