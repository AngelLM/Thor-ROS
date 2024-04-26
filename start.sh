#!/bin/bash

# Source ros2 as underlay
source /opt//ros/humble/setup.bash
echo ">> Ros2 sourced."
# Navigate to work space
cd ws_thor/
# Source ws as overlay
source install/setup.bash
echo ">> Overlay sourced."
echo ">> All set! Ready to start working."