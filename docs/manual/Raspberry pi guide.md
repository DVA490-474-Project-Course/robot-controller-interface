Raspberry Pi Guide
=======================


Getting started
-----------------------

### Instructions for enabeling uart on raspberry pi
Install Ubuntu server 22.04 and ROS humble on raspberry pi4.

To enable UART communication on raspberry pi:

enable UART by writing 'enable_uart=1' in 'sudo nano /boot/firmware/usercfg.txt'
Disable bluetooth by writing 'dtoverlay=disable-bt' in 'sudo nano /boot/firmware/usercfg.txt'
Open cmdline.txt and edit it. it should look like this:

Check bluetooth status with this command
sudo systemctl status bluetooth
if it is still active, disable Bluetooth in rfkill.
Run following command to check 
rfkill list bluetooth
if bluetooth no blocked, run this command
sudo rfkill block bluetooth
To keep it block even after rebooting, edit this file
sudo nano /etc/rc.local
add following line befire exit 0
rfkill block bluetooth
save and exit.
make script executable with following command
sudo chmod +x /etc/rc.local
run this command
sudo reboot
check blue tooth status by this command
sudo rfkill list bluetooth



/dev/serial0 should link to ttyAMA0 confirm it by running ls -l /dev/serial0
it should shouw something like, /dev/serial0 -> ttyAMA0
if it is not assigned properly, assign it manually with running following commands
sudo ln -s /dev/ttyAMA0 /dev/serial0
sudo ln -s /dev/ttyS0 /dev/serial1
sudo udevadm control --reload
ls -l /dev/serial* #check if symlinks are created


cmdline.txt should look like this 
multipath=off dwc_otg.lpm_enable=0 root=LABEL=writable rootfstype=ext4 rootwait fixrtc
run command 'sudo nano /boot/firmware/cmdline.txt'

make sure no other process is using tty AMA0. check it using this command
sudo lsof /dev/serial0
if any process is using it, kill that process using 'sudo kill <PID>' for example 'sudo kill 1234'


connect Raspbery Pi UART TX to stm32 UART RX and Raspberry pi UART RX to stm32 UART TX. 
Join the grounds.
now connection is establish. write, build and execute the code.

### Instructions for using ros 2 for communication via uart

Install ROS2 Dependencies
sudo apt install ros-<ros2-distro>-serial
Replace <ros2-distro> with the ROS2 distribution. in out case it is 'sudo apt install ros-humble-serial-driver'
run commands
sudo apt update
rosdep update
rosdep install --from-paths /opt/ros/humble/share/serial_driver --ignore-src -y

verify installation by running command 'ros2 pkg list | grep serial' , output should have 'ros-humble-serial-driver'.


CREAT ROS2 package for UART communication
ros2 pkg create --build-type ament_cmake uart_communication
rosdep install --from-paths src --ignore-src -r -y   //install dependencies in package folder
cd ~/ros2_ws/src/uart_communication    //goto directory

open cmake
nano CMakeLists.txt

cmake is as follows

cmake_minimum_required(VERSION 3.8)
project(uart_ros_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)


# Add your executable and specify the source files
add_executable(uart_ros_communication_node src/uart_ros_communication.cc)

# Link the dependencies
ament_target_dependencies(uart_ros_communication_node rclcpp serial)

# Install the executable (optional, if you want to install the package)
install(TARGETS uart_ros_communication_node
  DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

------------------------
open package.xml with in smae directory
nano package.xml

Locate the <depend> tags and add dependencies for rclcpp and serial under the <build_depend> and <exec_depend> sections.

package.xml should look like this


<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>uart_ros_communication</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="ubuntu@todo.todo">ubuntu</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

<!-- Add build and runtime dependencies -->
  <build_depend>rclcpp</build_depend>
  <build_depend>serial</build_depend>
  <exec_depend>rclcpp</exec_depend>
  <exec_depend>serial</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>



------------------------

goto ros2 workspace to build the package
cd ~/ros2_ws
colcon build --packages-select uart_ros_communication






-----------------------
