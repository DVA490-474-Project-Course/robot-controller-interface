
/* Related .h files */
#include "../simulation-interface/simulation_interface.h"
#include "../../src/simulation-interface/generated/grSim_Commands.pb.h"
#include "../../src/simulation-interface/generated/grSim_Packet.pb.h"
#include "../../src/simulation-interface/generated/grSim_Replacement.pb.h"

/* Project .h files */
#include "../../src/common_types.h"

/* Other .h files */
#include <gtest/gtest.h>
//#include <gmock/gmock.h>

class SimulationInterface {
public:
  // Constructor
  SimulationInterface(std::string ip, uint16_t port, int id, robot_controller_interface::Team team)
    : ip_address(ip), port_number(port), robot_id(id), robot_team(team) {
    destination.sin_family = AF_INET;
    destination.sin_port = htons(port);
    destination.sin_addr.s_addr = inet_addr(ip.c_str());
    socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    SetRobot(id, team);
    SetVelocity(0.0F, 0.0F, 0.0F);
    SetKickerSpeed(0.0F);
    SetSpinnerOn(false);
  }

  // Sets robot id and and robot team.
  void SetRobot(int id, robot_controller_interface::Team team) {
    robot_id = id;
    robot_team = team;
  }

  void SetVelocity(float x_speed, float y_speed, float angular_speed) {
    using_wheel_speed = false;
    this->x_speed = x_speed;
    this->y_speed = y_speed;
    this->angular_speed = angular_speed;
  }

  void SetVelocity(float front_left_wheel_speed, float back_left_wheel_speed,
    float back_right_wheel_speed, float front_right_wheel_speed) {
    using_wheel_speed = true;
      this->wheel1 = -front_left_wheel_speed;
      this->wheel2 = -back_left_wheel_speed;
      this->wheel3 = back_right_wheel_speed;
      this->wheel4 = front_right_wheel_speed;
    }

    void SetKickerSpeed(float kicker_speed) {
      this->kicker_speed = kicker_speed;
    }

    void SetSpinnerOn(bool spinner_on) {
      this->spinner_on = spinner_on;
    }

    // Send packet functions
    bool SendPacket() {
      grSim_Packet packet;
      grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
      packet.mutable_commands()->set_isteamyellow(robot_team == robot_controller_interface::Team::kYellow);
      packet.mutable_commands()->set_timestamp(0.0L);

      command->set_id(robot_id);
      command->set_kickspeedx(kicker_speed);
      command->set_kickspeedz(0.0F);
      command->set_spinner(spinner_on);
      command->set_wheelsspeed(using_wheel_speed);
      command->set_wheel1(wheel1);
      command->set_wheel2(wheel2);
      command->set_wheel3(wheel3);
      command->set_wheel4(wheel4);
      command->set_veltangent(x_speed);
      command->set_velnormal(y_speed);
      command->set_velangular(angular_speed);

      return SendPacket(packet);
    }

    bool SendPacket(grSim_Packet packet) {
      size_t size = packet.ByteSizeLong();
      void* buffer = malloc(size);

      if (!buffer) {
        return false; // Return false if memory allocation fails
      }

      packet.SerializeToArray(buffer, size);
      ssize_t bytes_sent = sendto(socket_fd, buffer, size, 0,
                                    reinterpret_cast<sockaddr*>(&destination),
                                    sizeof(destination));
      free(buffer);

      return bytes_sent == size; // Return true if the full packet was sent
    }

private:
  std::string ip_address;
  uint16_t port_number;
  int robot_id;
  robot_controller_interface::Team robot_team;
  int socket_fd;
  sockaddr_in destination;
  bool using_wheel_speed;
  float x_speed, y_speed, angular_speed;
  float wheel1, wheel2, wheel3, wheel4;
  float kicker_speed;
  bool spinner_on;
};

TEST(SimulationInterfaceTest, SendPacketReturnsTrueOnSuccess) {
  // Initialize SimulationInterface with test parameters
  SimulationInterface interface("127.0.0.1", 20011, 1, robot_controller_interface::Team::kYellow);

  // Configure robot settings
  interface.SetKickerSpeed(5.0F);
  interface.SetSpinnerOn(true);
  interface.SetVelocity(1.0F, 0.0F, 0.5F);

  // Test if SendPacket() returns true, indicating successful send
  EXPECT_TRUE(interface.SendPacket());
}