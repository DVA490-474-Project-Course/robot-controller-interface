// test/simulation_interface_test.cc
#include <gtest/gtest.h>
#include "../../src/simulation-interface/simulation_interface.h" // Adjust the include path as necessary

namespace robot_controller_interface {
namespace simulation_interface {

class TestableSimulationInterface : public SimulationInterface {
public:
    TestableSimulationInterface(std::string ip, uint16_t port, int id, Team team)
        : SimulationInterface(ip, port, id, team) {}

    // Expose the protected CreateProtoPacket method
    using SimulationInterface::CreateProtoPacket;
};

} // namespace simulation_interface
} // namespace robot_controller_interface

TEST(SimulationInterfaceTest, CreateProtoPacketTest) {
    robot_controller_interface::simulation_interface::TestableSimulationInterface sim_interface(
        "127.0.0.1", 10001, 1, robot_controller_interface::Team::kYellow);

    // Set values using the provided setter methods
    sim_interface.SetKickerSpeed(5.0f);
    sim_interface.SetSpinnerOn(true);
    sim_interface.SetVelocity(2.0f, 3.0f, 1.0f);

    // Call CreateProtoPacket
    grSim_Packet packet = sim_interface.CreateProtoPacket();

    // Check the values in the packet
    EXPECT_EQ(packet.commands().isteamyellow(), true);
    EXPECT_FLOAT_EQ(packet.commands().robot_commands(0).kickspeedx(), 5.0f);
    EXPECT_FLOAT_EQ(packet.commands().robot_commands(0).kickspeedz(), 0.0f);
    EXPECT_EQ(packet.commands().robot_commands(0).spinner(), true);
    EXPECT_FLOAT_EQ(packet.commands().robot_commands(0).veltangent(), 2.0f);
    EXPECT_FLOAT_EQ(packet.commands().robot_commands(0).velnormal(), 3.0f);
    EXPECT_FLOAT_EQ(packet.commands().robot_commands(0).velangular(), 1.0f);
}

