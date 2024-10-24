Robot Controller Interface
=======================

About
-----------------------
This repository contains the executable code intended for running on the individual robots (on the raspberry pi), termed Individual Robot Behaviour, and the accompanying API Simulation Interface and Hardware Interface.

### Built with
The Robot Controller Interface is built with the following:

- [Protobuf 3](https://protobuf.dev/)
- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- [nav2](https://docs.nav2.org/)

Getting started
-----------------------

### Prerequisites
On Ubuntu and other Debian derivatives, the dependencies can be installed with:
```
sudo apt install build-essential cmake libprotobuf-dev protobuf-compiler
```

For ros2 humble see: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

For nav2 see: https://docs.nav2.org/getting_started/index.html#installation

### Installation
1. Clone the repository:
```sh
git clone https://github.com/DVA490-474-Project-Course/robot-controller-interface.git
```
2. Navigate to the project directory:
```
cd robot-controller-interface
```
3. Create a build directory and navigate to it:
```
mkdir build & cd build
```
4. Build the source code:
```
cmake ..
```
5. Locate the binaries which should be stored in bin:
```
cd ../bin
```
6. Execute the desired binaries.

Usage
-----------------------

Roadmap
-----------------------
API:
- [x] Develop Simulation Interface
- [ ] Develop Hardware Interface

Individual Robot Behaviour (executable):
- [x] Develop Path planning 
- [x] Develop robot ability to shoot
- [ ] nav2 stack configuration
- [ ] Integrate Simulation/Hardware Interface
- [ ] Ability to send data
- [ ] Ability to receive data
- [ ] Initilization of robot

Design diagrams
-----------------------
Design diagrams/files can be found in the [docs](/docs) directory. Additionally they are available on:
- [Simulation Interface](https://www.mermaidchart.com/raw/16fc3609-d826-440a-bef5-40a7a39f1140?theme=dark&version=v0.1&format=svg)
- [Hardware Interface]()
- [Individual Robot Behaviour](https://www.mermaidchart.com/raw/dc459e07-4c98-46b8-8ac0-41c56aa6950f?theme=dark&version=v0.1&format=svg)

License
-----------------------
Distributed under the MIT License. See [License](/LICENSE) for more information.

Contributors and contact
-----------------------
- Aaiza Aziz Khan: akn23018@student.mdu.se
- Carl Larsson: cln20001@student.mdu.se
- Shruthi Puthiya Kunnon: spn23001@student.mdu.se
- Emil Åberg: eag24002@student.mdu.se
