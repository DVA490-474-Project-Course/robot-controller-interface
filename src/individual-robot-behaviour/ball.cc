/* ball.cc
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-22
 * Last modified: 2024-10-04 by Carl Larsson
 * Description: Source file for all functions that relate to the ball.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


/* Related .h files */
#include "../individual-robot-behaviour/ball.h"

/* C++ standard library headers */
#include <chrono>
#include <thread>
#include <mutex>

/* Other .h files */

/* Project .h files */
#include "../individual-robot-behaviour/state.h"
#include "../individual-robot-behaviour/path_planning.h"


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{

/*============================================================================*/

/* Used to indicate who had dwb do work */
std::atomic_bool atomic_shoot_setup_work = false;
std::mutex goalie_pose_mutex;

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

/*
 *  Ensure robot is setup for a shot, then shoots
 */
void shoot_setup(Pose *goalie_pose, std::atomic_bool *atomic_shoot_ball, 
    std::atomic_bool *playing_left, Pose *target_pose)
{
  /* Declare variables outside loop */
  /* Limit the loop speed to not take up to much CPU */
  const int kLoopRateHz = 10;
  const std::chrono::milliseconds kLoopDuration(1000/kLoopRateHz);

  std::chrono::steady_clock::time_point start_time;
  std::chrono::steady_clock::time_point end_time;
  std::chrono::milliseconds elapsed_time;

  /* Used for finding where to shoot */
  Pose shoot_target;
  double theta;

  while(true)
  {
    start_time = std::chrono::steady_clock::now();

    /* We are to shoot */
    /* If we have been signaled and we have ball */
    if((*atomic_shoot_ball) & (current_state.GetBall()))
    {
      /* Find where in goal to shoot based on goalie pose */
      /* Make thread safe */
      goalie_pose_mutex.lock();
      shoot_target = FindShootTarget(*goalie_pose, *playing_left);
      goalie_pose_mutex.unlock();
      current_state_mutex.lock();
      theta = CalculateAngle(current_state.GetX(), current_state.GetY(), shoot_target.GetX(), shoot_target.GetY());
      current_state_mutex.unlock();

      /* Indicate who is calling for path planning work */
      atomic_shoot_setup_work = true;

      /* Setting target_pose will trigger path_planner to start angling towards */
      /* target */
      target_pose_mutex.lock();
      /* We do not want to move */
      current_state_mutex.lock();
      (*target_pose).SetX(current_state.GetX());
      (*target_pose).SetY(current_state.GetY());
      current_state_mutex.unlock();
      /* We only wanna angle correctly */
      (*target_pose).SetTheta(theta);
      target_pose_mutex.unlock();

      /* TODO fix so while loop also has frequency */
      /* Ensure we do not get stuck in while loop while waiting for getting */
      /* correct direction, since this command could get abborted and a new */
      /* command pursued instead, could also loose the ball. */
      while((*atomic_shoot_ball) & (current_state.GetBall()))
      {
        /* Wait for us to have correct direction */
        if(atomic_target_reached_flag == 2)
        {
          /* TODO call function which activates kicker */

          atomic_target_reached_flag = 0;
        }
      }
    }

    end_time = std::chrono::steady_clock::now();
    elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    if(elapsed_time < kLoopDuration)
    {
      std::this_thread::sleep_for(kLoopDuration - elapsed_time);
    }
  }
}

/*============================================================================*/

} /* namespace individual_robot_behaviour */
} /* namesapce robot_controller_interface */
