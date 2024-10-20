# ECSE473 Modern Robotic Programming Laboratory 3

The project directory structure is in this way as mentioned in Laboratory description:

```
  ik_service
  ├── CMakeLists.txt
  ├── package.xml
  ├── launch
  │   ├── ik_service.launch 
  ├── src
  │   ├── ik_client.cpp
  │   └── ik_service.cpp
  ├── srv
  │   └── PoseIK.srv
  └── README.md
```
### Prerequisites

In order to launch the project, you should add the code repository: 
- Installed ROS Noetic
- To install ur_kinematics package, add public key:
  ```
  
    wget -q https://cwru-ecse-373.github.io/cwru-ecse-373.asc -O - | sudo apt-key add -
  
  ```
  -  Add repository:
  ```
  
    sudo apt-add-repository https://cwru-ecse-373.github.io/repo
  
  ```
    -  Upgrade all packages wıth updates:
  ```
  
     sudo apt upgrade
  
  ```
      -  Add the ur_kinematics package:
  ```
  
     sudo apt install ros-noetic-ur-kinematics
  
  ```
  -  Run the system ROS configuration script again.:
  ```
  
     source /opt/ros/noetic/setup.bash
  
  ```
  -  Check the ur_kinematics package is installed or not:
  ```
  
     rospack find ur_kinematics
  
  ```

### Pull this package to your computer

   
## Authors

  - **Ilke Kas** - *PhD at ECSE* -
    [Ilke Kas Github](https://github.com/ilke-kas)

