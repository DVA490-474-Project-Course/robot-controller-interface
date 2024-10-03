/* ball.h
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-22
 * Last modified: 2024-10-03 by Carl Larsson
 * Description: Header file for everything that relates to the ball.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


#ifndef ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_BALL_H_
#define ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_BALL_H_


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

/*! 
 * @brief Find where in goal to aim, aim where goalie is not.
 * 
 * This function takes goalies position and which side of the field is friendly
 * and returns where in goal to aim for kicking the ball.
 *
 * @param[in] goalie_pose The pose of the goalie.
 * @param[in] playing_left Whether the left side of the field is friendly or 
 * not.
 * @return The target position for the shot.
 * */
Pose FindShootTarget(Pose goalie_pose, bool playing_left);

/*============================================================================*/

/*! 
 * @brief Global atomic variable which indicates if shoot_setup function set
 * local_path_planning to work.
 *
 * This global atomic variable indicates whether the shoot_setup function set
 * local_path_planning to work so that the callback function knows which 
 * function it should tell that the task has been completed. 
 *
 * */
extern std::atomic_bool atomic_shoot_setup_work;
extern std::mutex goalie_pose_mutex;

/*============================================================================*/

} /* namespace individual_robot_behaviour */
} /* namesapce robot_controller_interface */

#endif /* ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_BALL_H_ */
