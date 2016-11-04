# Robot_Mapping_TG
Trabalho de graduação - Engenharia de Controle e Automação (UNESP).
Final project - Control and automation engineering (São Paulo State Universisty - UNESP)

Developed with ROS Indigo, V-REP EDU 3.0 and Opencv2 on Ubuntu 14.04LTS.

The catkin_workspace mainly includes the following two ROS packages:

- tg (tg_node): used to detect ArUco (Augmented reality) tags and to estimate the position and orientation of the robot via OpenCV.
- odometria(odometria_node): This node is complementary to tg_node, so run them both before starting the simulation on V-REP.
This node estimates the odometry based in a kinematic model of the differential robot (open loop). Also, it does automatically 
correct this estimation when a AruCo Tag is detected (closed loop), by subscribing from a topic from tg_node.

Also, it includes some auxiliary packages that can be used for comparing the method used by "tg" and "odometria" with other
known techniques:

- odometria simples(odometria_simples_node): This node estimates the odometry based in a kinematic model of the differential
robot (open loop). But there is no correction of pose at all (used only for comparison).
- slam_gmapping: This package is the most popular method of SLAM (Simultaneous Localization and Mapping), and can be also 
downloaded at http://wiki.ros.org/slam_gmapping .
- pose_publisher_master: Used for convertion from "tf" to a "pose_stamped" message. Used for visualization in RVIZ software.

Besides, it includes the V-REP simulation file (.ttt) and video files of the project.

