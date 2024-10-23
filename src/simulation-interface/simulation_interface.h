/* simulation_interface.h
 *==============================================================================
 * Author: Emil Åberg, Shruthi Puthiya Kunnon, Aaiza Aziz Khan
 * Creation date: 2024-09-25
 * Last modified: 2024-10-07 by Emil Åberg
 * Description: Interface that can send UDP data to grSim to make robots move
 * and kick
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef ROBOTCONTROLLERINTERFACE_SIMULATIONINTERFACE_SIMULATIONINTERFACE_H
#define ROBOTCONTROLLERINTERFACE_SIMULATIONINTERFACE_SIMULATIONINTERFACE_H

/* C system headers */
#include <arpa/inet.h>

/* C++ standard library headers */
#include <string>

/* Project .h files */
#include "grSim_Packet.pb.h"
#include "../common_types.h"

namespace robot_controller_interface
{
namespace simulation_interface
{

/*!
 * @brief Class for interfacíng to grSim .
 * 
 * Class that allows communication with grSim and methods to control one robot
 * in the simulation. Multiple robots can be controlled with multiple instantiations
 * of this class.
 */
class SimulationInterface
{
public:
  /*!
    * @brief Constructor that sets up connection to grSim for one robot.
    *
    * @param[in] ip Ip address of the computer that is running grSim. When running
    * grSim on the same computer that the simulation interface is running on this
    * value should be localhost i.e. "127.0.0.1".
    *
    * @param[in] port The command listen port of grSim. This should
    * be set to the same value as that which is set in the grSim configuration
    * in Communication->Command listen port.
    * 
    * @param[in] id Id number of the robot that is conntrolled in grSim.
    * 
    * @param[in] team Team color of the robot that is controlled in grSim.
    */
  SimulationInterface(std::string ip, uint16_t port, int id, enum Team team);

  /*!
    * @brief Method to change robot to control with the class instance.
    *
    * @param[in] id ID of the robot in grSim that is to be controlled.
    *
    * @param[in] team Team color of the robot that is to be controlled.
    */
  void SetRobot(int id, enum Team team);

  /*!
    * @brief Method to set the velocity of the kicker.
    *
    * @param[in] kicker_speed Set the speed of the kicker in m/s.
    * 
    * @pre In order for robot commands to take effect, UDP packets need
    * to be sent periodica by calling SendPacket().
    * 
    * @see SendPacket for sending UDP packets.
    */
  void SetKickerSpeed(float kicker_speed);

  /*!
    * @brief Method to control the spinner.
    *
    * @param[in] kicker_speed Set the speed of the kicker in m/s.
    * 
    * @pre In order for robot commands to take effect, UDP packets need
    * to be send continuously by calling SendPacket()
    * 
    * @see SendPacket() for sending UDP packets.
    */
  void SetSpinnerOn(bool spinner_on);

  /*!
    * @brief Method to set the robot velocity in terms of x, y and angular speeds.
    *
    * @param[in] x_speed The speed of the robot along the x axis in m/s.
    * 
    * @param[in] y_speed The speed of the robot along the y axis in m/s.
    * 
    * @param[in] angular_speed The angular speed of the robot in radians/s.
    * 
    * @note The velocity can either be set in terms of x, y and theta using this method,
    * or alternatively by setting the speed of the individual wheels by using
    * SetVelocity(float front_left_wheel_speed, float back_left_wheel_speed, float back_right_wheel_speed, float front_right_wheel_speed).
    * 
    * @pre In order for robot commands to take effect, UDP packets need
    * to be sent continuously by calling SendPacket().
    */
  void SetVelocity(float x_speed, float y_speed, float angular_speed);

  /*!
    * @brief Method to set the robot velocity by setting the speeds of the individual wheels.
    *
    * @param[in] front_left_wheel_speed The speed of the front left wheel in m/s.
    * 
    * @param[in] back_left_wheel_speed The speed of the back left wheel in m/s.
    * 
    * @param[in] back_right_wheel_speed The speed of the back right wheel in m/s.
    * 
    * @param[in] front_right_wheel_speed The speed of the front right wheel in m/s.
    * 
    * @note the velocity can either be set in terms of setting the individual wheel speeds
    * using this method, ot by setting the velocity in terms of x, y and theta by using
    * SetVelocity(float x_speed, float y_speed, float angular_speed).
    * 
    * @pre In order for robot commands to take effect, UDP packets need
    * to be sent continuously by calling SendPacket().
    */
  void SetVelocity(float front_left_wheel_speed, float back_left_wheel_speed,
    float back_right_wheel_speed, float front_right_wheel_speed);

  /*!
    * @brief Sends a UDP packet to grSim, carrying the robot command
    * 
    * Sends a UDP packet to grSim, needs to be called periodically
    * in order for communication to be maintained, recommended minimum rate
    * of 50Hz.
    * 
    * @warning A robot that is not continously receiving commands will just
    * stand still.
    */
  void SendPacket();

private:
  /*********************/
  /* Network variables */
  /*********************/

  /*!
   * @brief socket file descriptor.
   */
  int socket;

  /*!
   * @brief Address of grSim.
   */
  sockaddr_in destination;

  /*******************/
  /* Robot variables */
  /*******************/

  /*!
   * @brief Robot id.
   */
  int id;

  /*!
   * @brief Team of the robot.
   */
  enum Team team;

  /*!
   * @brief Flag indicating wheter spinner is on.
   */
  bool spinner_on;

  /*!
   * @brief Kicker speed in m/s.
   */
  float kicker_speed;

  /*!
   * @brief Speed in x direction in m/s.
   */
  float x_speed;

  /*!
   * @brief Speed in y direction in m/s.
   */
  float y_speed;

  /*!
   * @brief Angular speed of the robot in radians/s.
   */
  float angular_speed;

  /*!
   * @brief Speed of front left wheel in m/s.
   */
  float wheel1;

  /*!
   * @brief Speed of back left wheel in m/s.
   */
  float wheel2;

  /*!
   * @brief Speed of back right wheel in m/s.
   */
  float wheel3;

  /*!
   * @brief Speed of front right wheel in m/s.
   */
  float wheel4;

  /*!
   * @brief Flag indicating whether individual wheel speeds are set or if
   * velocity is determined by setting the x, y, theta speeds of the entire robot.
   */
  bool using_wheel_speed;

  /********************/
  /* Helper functions */
  /********************/

  /*!
   * @brief Send a grSim protobuf packet.
   */
  void SendPacket(grSim_Packet packet);
};

} /* namespace simulation_interface */
} /* namesapce robot_controller_interface */

#endif /* ROBOTCONTROLLERINTERFACE_SIMULATIONINTERFACE_SIMULATIONINTERFACE_H */