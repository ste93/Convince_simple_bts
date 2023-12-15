FROM elandini84/r1images:tourCore2_ubuntu22.04_iron_stable

USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y libqt6scxml6 qt6-base-dev qt6-scxml-dev libqt6scxml6-bin

USER user1

RUN /bin/bash -c "source /opt/ros/iron/setup.bash && \
    cd /home/user1 && git clone https://github.com/ste93/convince_simple_bts.git -b ros2 && \
    cd /home/user1/convince_simple_bts/ros2prova/bt_interfaces/ && colcon build && source install/setup.bash && \
    cd /home/user1/convince_simple_bts/ros2prova/other_interfaces && colcon build && source install/setup.bash && \
    cd /home/user1/convince_simple_bts/ros2prova/bt_nodes && colcon build && source install/setup.bash && \
    cd /home/user1/convince_simple_bts/ros2prova/alarm_battery_low_skill && colcon build && source install/setup.bash && \
    cd /home/user1/convince_simple_bts/ros2prova/battery_drainer_component && colcon build && source install/setup.bash && \
    cd /home/user1/convince_simple_bts/ros2prova/battery_drainer_skill && colcon build && source install/setup.bash && \
    cd /home/user1/convince_simple_bts/ros2prova/battery_level_skill && colcon build && source install/setup.bash && \
    cd /home/user1/convince_simple_bts/ros2prova/bt_executable && colcon build && source install/setup.bash"

RUN pip install reelay websocket-client jedi prompt_toolkit

RUN /bin/bash -c "source /opt/ros/iron/setup.bash && \
    cd /home/user1 && https://github.com/autonomy-and-verification-uol/ROSMonitoring.git -b ros2 && \
    cd ROSMonitoring/generator/ros2_devel

RUN echo "source /home/user1/convince_simple_bts/ros2prova/bt_interfaces/install/setup.bash" >> ~/.bashrc && \
    echo "source /home/user1/convince_simple_bts/ros2prova/other_interfaces/install/setup.bash" >> ~/.bashrc && \
    echo "source /home/user1/convince_simple_bts/ros2prova/bt_nodes/install/setup.bash" >> ~/.bashrc && \
    echo "source /home/user1/convince_simple_bts/ros2prova/alarm_battery_low_skill/install/setup.bash" >> ~/.bashrc && \
    echo "source /home/user1/convince_simple_bts/ros2prova/battery_drainer_component/install/setup.bash" >> ~/.bashrc && \
    echo "source /home/user1/convince_simple_bts/ros2prova/battery_drainer_skill/install/setup.bash" >> ~/.bashrc && \
    echo "source /home/user1/convince_simple_bts/ros2prova/battery_level_skill/install/setup.bash" >> ~/.bashrc && \
    echo "source /home/user1/convince_simple_bts/ros2prova/bt_executable/install/setup.bash" >> ~/.bashrc 