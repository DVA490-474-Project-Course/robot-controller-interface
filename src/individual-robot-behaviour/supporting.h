/* supporting.h
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-26
 * Last modified: 2024-10-04 by Carl Larsson
 * Description: Supporting functions header file. Everything that is required
 * to maintain and allow operation for the main tasks of the robots are 
 * considered supporting and can be found here.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


/* Related .h files */

/* C++ standard library headers */
#include <atomic>

/* Other .h files */

/* Project .h files */
#include "../individual-robot-behaviour/state.h"


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{

/*============================================================================*/

/* 
 * @brief
 *
 *
 *
 * @param[in,out]
 * @param[in,out]
 */
void initialize_robot(std::atomic_int *robot_id, Pose *target_position);

/*============================================================================*/

/*
 * @brief
 */
void listener();

/*============================================================================*/

/* 
 * @brief
 */
void sender();

/*============================================================================*/

} /* namespace individual_robot_behaviour */
} /* namesapce robot_controller_interface */
