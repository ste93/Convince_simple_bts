#! /bin/bash
source /opt/ros/iron/setup.bash && \
cd bt_interfaces/ && colcon build && source install/setup.bash && \
cd ../other_interfaces && colcon build && source install/setup.bash && \
cd ../bt_nodes && colcon build && source install/setup.bash && \
cd ../alarm_battery_low_skill && colcon build && source install/setup.bash && \
cd ../battery_drainer_component && colcon build && source install/setup.bash && \
cd ../battery_drainer_skill && colcon build && source install/setup.bash && \
cd ../battery_level_skill && colcon build && source install/setup.bash && \
cd ../bt_executable && colcon build && source install/setup.bash && cd ..