#!/bin/bash

catkin_make_isolated --install --use-ninja

#catkin_make_isolated --install --use-ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes

source devel_isolated/setup.bash
# source install_isolated/setup.bash
