# SAUVC 2019 Bubbles Control

## Installation

* Install ROS
* Create catkin workspace (skip if already created): http://wiki.ros.org/catkin/Tutorials/create_a_workspace
* Clone this project to your catkin workspace
```
cd ~/catkin_ws/src
git clone https://github.com/heyuhang0/SAUVC2019-ROS.git
```
* Install dependencies
```
sudo apt install ros-melodic-pid
sudo apt install ros-melodic-vision-msgs
```
* Then build your workspace
```
cd ~/catkin_ws
catkin_make
```

## Run

* To start sauvc simulation, follow instructions in https://github.com/heyuhang0/sauvc_simulator

* Launch base control: `roslaunch bubbles_control base.launch`

    This is equivalent to:
    ```
    # node to process raw sensor messages
    roslaunch bubbles_sensors state_publisher.launch

    # pid controller
    roslaunch bubbles_pid bubbles_pid.launch

    # cv tracker for gate and flare
    roslaunch bubbles_cv launch_all.launch
    ```

* Adjust AUV initial position, facing it to the gate

    For simulaiton environment, this can be done using `roslaunch bubbles_pid teleop.launch` . By default, it will set yaw to the correct angle. Once done, exit the program.

* Start control strategy: `rosrun bubbles_control sauvc2019_final.py`

## ROS graph
![rosgraph](https://user-images.githubusercontent.com/10456378/72127390-721a3b00-33aa-11ea-93ad-5f78dc28fd2e.png)

## Screenshots
![image](https://user-images.githubusercontent.com/10456378/72126508-52354800-33a7-11ea-91c5-76d825bf73ca.png)

![image](https://user-images.githubusercontent.com/10456378/72126566-80b32300-33a7-11ea-96dd-a57d90d450eb.png)
