# CMake generated Testfile for 
# Source directory: /home/aaiza/Desktop/projectrepo/robot-controller-interface
# Build directory: /home/aaiza/Desktop/projectrepo/robot-controller-interface/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(my_test "/home/aaiza/Desktop/projectrepo/robot-controller-interface/bin/my_tests")
set_tests_properties(my_test PROPERTIES  _BACKTRACE_TRIPLES "/home/aaiza/Desktop/projectrepo/robot-controller-interface/CMakeLists.txt;94;add_test;/home/aaiza/Desktop/projectrepo/robot-controller-interface/CMakeLists.txt;0;")
subdirs("_deps/googletest-build")
