/* simulation_interface_test.cc
 *==============================================================================
 * Author: Aaiza Aziz Khan and Shruthi Puthiya Kunnon
 * Creation date: 2024-11-01
 * Last modified: 2024-11-01 by Aaiza Aziz Khan
 * Description: Test suite for simulation interface
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h files */
#include "../../src/simulation-interface/simulation_interface.h"

/* Other .h files */
#include "gtest/gtest.h"

    class TestableSimulationInterface 
        : public robot_controller_interface::simulation_interface::SimulationInterface {
      public:
        TestableSimulationInterface(std::string ip, uint16_t port, int id, 
            robot_controller_interface::Team team)
                : SimulationInterface(ip, port, id, team) {}
          GrSimPacket CallCreateProtoPacket() {
          return CreateProtoPacket();
          }

    };

/* Test to verify a packet is constructed correctly */
TEST(SimulationInterfaceTest, CreateProtoPacketTest) {
  TestableSimulationInterface sim_interface("127.0.0.1", 10001, 1, 
        robot_controller_interface::Team::kYellow);

  /* Set values using the provided setter methods */
  sim_interface.SetKickerSpeed(5.0f);
  sim_interface.SetSpinnerOn(true);
  sim_interface.SetVelocity(2.0f, 3.0f, 1.0f);

  /* Call CreateProtoPacket */
  GrSimPacket packet = sim_interface.CallCreateProtoPacket();

  /* Check the values in the packet */
  EXPECT_EQ(
      packet.commands().isteamyellow(), true);
  EXPECT_FLOAT_EQ(
      packet.commands().robot_commands(0).kickspeedx(), 5.0f);
  EXPECT_FLOAT_EQ(
      packet.commands().robot_commands(0).kickspeedz(), 0.0f);
  EXPECT_EQ(
      packet.commands().robot_commands(0).spinner(), true);
  EXPECT_FLOAT_EQ(
      packet.commands().robot_commands(0).veltangent(), 2.0f);
  EXPECT_FLOAT_EQ(
      packet.commands().robot_commands(0).velnormal(), 3.0f);
  EXPECT_FLOAT_EQ(
      packet.commands().robot_commands(0).velangular(), 1.0f);
}
