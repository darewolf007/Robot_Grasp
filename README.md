# Robot_Grasp
## Introduction
    this is a robot grasp project, which is based on the robot operating system(ROS) and sawyer robot.
    The project is mainly divided into two parts: the first part is the simulation of the robot grasping process, and the second part is the real robot grasping process.
## How to use
    ### first terminal
        ./intera.sh
        source devel/setup.bash
        source activate py27
        roslaunch robot_grasp Sawyer_control.launch
    ### second terminal
        ./intera.sh
        source devel/setup.bash
        source activate py38
        python src/Robot_Grasp/src/scripts/Run_robot_grasp.py
