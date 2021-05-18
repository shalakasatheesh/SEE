## Create static IP address in Ubuntu
* Go to edit connections
* Add new *Ethernet* connection and name it *youBot_connection*
* Go to *IPv4 Settings* tab and chose *Manual* method
* In the same tab add new address: 192:168:1:2
* Additionally, set *Netmask* to 24 and *Gateway* to 192:168:1:1
* Press save!

## Set up a catkin workspace

    source /opt/ros/catkin_ws/setup.bash
    mkdir desired_directory/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    catkin build

## Clone following repositories in ROS workspace
    git clone: 
* https://github.com/mas-group/youbot_description
* https://github.com/mas-group/youbot_driver_ros_interface
* https://github.com/mas-group/youbot_driver.git
* https://github.com/mas-group/youbot-manipulation
* youBot_placing_experiment


## Make changes in following files
* Under youbot_driver repository:
   + In youbot − *ethercat.cfg* file set *EthernetDevice* name to be the same name as one in used PC. This name can be found by running command *ifconfig* on a terminal.

  + In *youbot−manipulator.cfg* file set set following indices for *[JointT opology]*:
    ```
    ManipulatorJoint1 = 1

    ManipulatorJoint2 = 2
    
    ManipulatorJoint3 = 3
    
    ManipulatorJoint4 = 4
    
    ManipulatorJoint5 = 5
    ```

* Under youbot_driver_ros_interface repository, in file *launch/youbot_driver.launch*
set value of parameter *youBotHasBase* to false!
* In the same file, change robot description parameter to *youbot_arm_only.urdf.xacro.* 

* Under youbot-manipulation repository, in youbot_moveit/config/youbot.srdf file, remove following lines:
  + The comment: VIRTUAL JOINT: Purpose: this element defines a virtual joint between a robot link and an external frame of...
  + <virtual_joint name="odom" type="fixed" parent_frame="odom" child_link="base_footprint" />

## Before running launch files. 

For youbot_driver_ros_interface repository files it is required to set root access
==================================================================================
* The youBot Driver needs access to the raw ethernet device, which Under Linux a normal user does not have. You can grand this capability to a program by the tool setcap.

* To provide a program with raw access to a ethernet device use:
```
 sudo setcap cap_net_raw+ep <path_to_your_program_executable>/<name_of_your_program_executable>
 sudo ldconfig <path_to_your_program_executable>
 ```
* If the program is compiled with catkin_make the executables are created in folder 
```
<your catkin workspace>/devel/lib/<your program name>.
```

* If you are installing from debian packages the executables can be found in /opt/ros/indigo/lib/ folder.

In youBot_placing.launch under youBot_placing_experiment package
==================================================================================
* Set number of the youBot arm on which experiment is conducted
* This number is given as yb_number parameter in launch file
* Parameter should take integer values 3 or 1

In youbot-manipulation/youbot_moveit/config/joint_limits.yaml
===================================================================================
* Reduce arm's joint velocity limits, to avoid overshooting the arm:
    + I.e. parameter  "max_velocity" should be changed to 0.5.


## Run in separate terminals
* roscore
* roslaunch youbot_driver_ros_interface youbot_driver.launch
* roslaunch youbot_moveit move_group.launch
* roslaunch youBot_placing_experiment youBot_placing.launch
