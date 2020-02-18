# rob_swe_p5

    mkdir src
    mkdir src/slam_gmapping
    mkdir src/turtlebot
    mkdir src/turtlebot_interactions
    mkdir src/turtlebot_simulator
    mkdir src/map
    mkdir src/scripts
    mkdir src/rvizConfig
    mkdir src/pick_objects
    mkdir src/add_markers
    mkdir src/pick_objects/src
    mkdir src/add_markers/src
    git clone https://github.com/ros-perception/slam_gmapping.git src/slam_gmapping/
    git clone https://github.com/turtlebot/turtlebot.git src/turtlebot/
    git clone https://github.com/turtlebot/turtlebot_interactions.git src/turtlebot_interactions/
    git clone https://github.com/turtlebot/turtlebot_simulator.git src/turtlebot_simulator/
    vim src/test_slam.sh
    sudo apt-get install ros-melodic-openslam-gmapping
    sudo apt-get install ros-melodic-joy
    sudo apt-get install ros-melodic-joystick-drivers
    sudo apt-get install ros-melodic-kobuki-*
    git clone https://github.com/yujinrobot/kobuki.git src/kobuki/
    sudo apt-get install ros-melodic-yocs-controllers
    sudo apt-get install ros-melodic-ecl-streams

Some enumerations in kobuki messages need to be  changed. KEYCODE_LEFT -> Keycode_Left etc, BUTTON_0 -> Button_0 etc
