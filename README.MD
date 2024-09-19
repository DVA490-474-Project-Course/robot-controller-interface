Robot Controller Interface
=======================



Dependencies
-----------------------

The Robot Controller Interface depends on:

- [CMake](https://cmake.org/)
- [Protobuf 3](https://protobuf.dev/)

On Ubuntu and other Debian derivatives, the dependencies can be installed with:

```
sudo apt install build-essential cmake libprotobuf-dev protobuf-compiler
```

Building the source code
-----------------------

The Robot Controller Interface can be built by running the following commands in the repo root:

```
cmake -B ./build
cmake --build ./build
```

Running the binaries
-----------------------

Built binaries can be found in the /bin folder. During the development phase of the project, it is recommended to run them from a terminal, since in the case they do not return from main, it is possible to stop the application by pressing ctrl-C.

