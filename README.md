Teleoperation Control of ROS-based Industrial Robot with EMG signals.

This System is used in ROS in Ubuntu environments (PC). Robot Arm with 6 DOF is controlled remotely using a EMG sensor. 
Robot arms and PC are connected, and the sensor is wirelessly connected to PC using a Bluetooth adapter. 
Using ROS-communication, arm movement data recognized by the sensor are sent to the robot arm.

The robot arm moves in the same direction as the human movement.
And a gripper is operated by a human hand gesture.


**Development Environment and Equipment**


Robot arm : UR3 (Universal Robots)

Gripper : 2F - 140 Gripper (Robotiq)

OS : Linux Ubuntu 18.04

ROS : ROS Melodic Morenia

Sensor : Myo armband (Thalmic Labs)




**Working System**(in Ubuntu)


$ roscore

$ roslaunch ur_robot_driver ur3_bringup.launch robot_ip:= (input_your_ip)

$ rosrun ur_robot_driver myo-rawNode.py

$ rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0

$ rosrun robotiq_2f_gripper_control Robotiq2FGripperStatusListener.py

$ rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py

$ rosrun ur_robot_driver RemoteControl_UR3_2to3.py
