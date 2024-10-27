/* ball.cc
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-22
 * Last modified: 2024-10-27 by Carl Larsson
 * Description: Source file for all functions that relate to the ball.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


/* Related .h files */
#include "../individual-robot-behaviour/ball.h"

/* C++ standard library headers */
#include <atomic>
#include <chrono>
#include <thread>

/* Other .h files */

/* Project .h files */
#include "../individual-robot-behaviour/path_planning.h"
#include "../individual-robot-behaviour/state.h"


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{

/*============================================================================*/

/* Indicates if shoot_setup function set path planning to work */
std::atomic_bool atomic_shoot_setup_work = false;

/*============================================================================*/

/* Find where in goal we should shoot, we shoot where goalie is not */
Pose FindShootTarget(Pose goalie_pose, bool playing_left)
{
  Pose shoot_target;
  /* If friendly half is left (then enemy is right, or 0 rad), otherwise if */
  /* friendly half is right (then enemy is left, or pi rad) */
  if(playing_left)
  {
    shoot_target.SetX(PlayingField::kTouchLineX/2);
  }
  else
  {
    shoot_target.SetX(-PlayingField::kTouchLineX/2);
  }

  /* 0 is middle of goal */
  if(goalie_pose.GetY() < 0)
  {
    /* kGoalY is entire size of goal in y */
    shoot_target.SetY(PlayingField::kGoalY * 3/8);
  }
  else 
  {
    /* kGoalY is entire size of goal in y */
    shoot_target.SetY(-PlayingField::kGoalY * 3/8);
  }

  return shoot_target;
}

/*============================================================================*/

/* Ensure robot is setup for a shot, then shoots */
void ShootSetup(Pose *goalie_pose, std::atomic_bool *atomic_shoot_ball, 
    std::atomic_bool *atomic_playing_left, Pose *target_pose)
{
  /* TODO Add checks incase arguments are null */

  /* Declare variables outside loop */
  /* Limit the loop speed to not take up to much CPU */
  std::chrono::milliseconds outer_loop_duration(200);
  std::chrono::milliseconds inner_loop_duration(20);
  std::chrono::time_point<std::chrono::high_resolution_clock> outer_start;
  std::chrono::time_point<std::chrono::high_resolution_clock> inner_start;

  /* Used for finding where to shoot */
  Pose shoot_target;
  double theta;

  while(true)
  {
    /* Limit loop speed of outer loop */
    outer_start = std::chrono::high_resolution_clock::now();

    /* We are to shoot */
    /* If we have been signaled and we have ball */
    if((*atomic_shoot_ball) & (current_state.GetBall()))
    {
      /* Find where in goal to shoot based on goalie pose */
      shoot_target = FindShootTarget(*goalie_pose, *atomic_playing_left);
      theta = CalculateAngle(current_state.GetX(), current_state.GetY(), 
		      shoot_target.GetX(), shoot_target.GetY());

      /* 
       * Setting target_pose will trigger path_planner to start angling 
	     * towards target.
       */
      /* We do not want to move */
      (*target_pose).SetX(current_state.GetX());
      (*target_pose).SetY(current_state.GetY());
      /* We only wanna angle correctly */
      (*target_pose).SetTheta(theta);

      /* Indicate who is calling for path planning work */
      atomic_shoot_setup_work = true;

      /* 
	     * Ensure we do not get stuck in while loop while waiting for getting
       * correct direction, since this command could get abborted and a new
       * command pursued instead, could also loose the ball. 
	     */
      while((*atomic_shoot_ball) & (current_state.GetBall()))
      {
        /* Limit loop speed of inner loop */
        inner_start = std::chrono::high_resolution_clock::now();

        /* Wait for us to have correct direction */
        if(atomic_target_reached_flag == 2)
        {
          /* TODO call function which activates kicker */

          atomic_target_reached_flag = 0;
        }

        /* Enforce inner loop frequency */
        std::this_thread::sleep_until(inner_start + inner_loop_duration);
      }
    }

    /* Enforce outer loop frequency */
    std::this_thread::sleep_until(outer_start + outer_loop_duration);
  }
}

/*============================================================================*/

} /* namespace individual_robot_behaviour */
} /* namesapce robot_controller_interface */
