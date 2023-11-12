cmake_minimum_required(VERSION 3.0.2)
project(tcp_socket)

find_package(catkin_simple REQUIRED)
# Add the python directory to the PYTHONPATH
catkin_python_setup()

catkin_simple()

cs_install()

file(GLOB python_scripts scripts/*.py)
catkin_install_python(PROGRAMS
    ${python_scripts} DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION})

cs_export()