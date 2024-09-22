// path_planning.h
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-19
// Last modified: 2024-09-22 by Carl Larsson
// Description: Path planning header file.
// License: See LICENSE file for license details.
//==============================================================================


#ifndef ROBOTCONTROLLERINTERFACE_PATHPLANNING_PATHPLANNING_H_
#define ROBOTCONTROLLERINTERFACE_PATHPLANNING_PATHPLANNING_H_


// Related .h files

// C++ standard library headers

// Other .h files

// Project .h files
#include "../common_types.h"


namespace robot_controller_interface
{
namespace path_planning
{


// Struct for a pose in 2D space.
struct Pose
{
    int x;
    int y;
    double theta;
};


} // namespace path_planning
} // namespace robot_controller_interface

#endif // ROBOTCONTROLLERINTERFACE_PATHPLANNING_PATHPLANNING_H_
