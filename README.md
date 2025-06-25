# ECSE473 Modern Robotic Programming Laboratory 4

This project was developed as part of ECSE 473: Modern Robotic Programming – Laboratory 4, which focused on implementing an Inverse Kinematics (IK) Service using ROS Noetic for the UR10 robotic arm. The objective was to create a custom ROS service that receives a target pose and returns all valid IK solutions using the ur_kinematics package. The project includes both the service and client nodes, demonstrates integration of ROS services, and supports both automated and manual testing via rosservice.

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

### Create workspace in your computer
- Run Configuration Script ROS Noetic
   ```
  
     source /opt/ros/noetic/setup.bash
  
  ``` 
- Make a directory ik_service_ws
   ```
  
     mkdir ik_service_ws
  
  ```
- Make a directory src inside the ws
  ```
  
     cd ik_service_ws
     mkdir src
  
  ```
  - Finish configuring directory structure
  ```
  
     catkin_make
  
  ```
  - Run workspace configuration to be used by ROS
  ```
  
     source devel/setup.bash
  
  ```
  ### Clone this repository
  - Clone this repository
   ```
     cd src
     git clone git@github.com:cwru-courses/ecse473_f24_ixk238_ik_service.git
  
  ```

  ### How to use
    - First run the ik_service node by using launch file:
   ```
  
     roslaunch ik_service ik_service.launch
  
  ```
   - You wil see this output which shows that the service is ready:
  ```
  
     [ INFO] [1729448274.068106260]: Ready to pose ik.
  
  ```
  - Secondly. run the ik_client node:
  ```
  
     rosrun ik_service ik_client_node 
  
  ```
  
  - When you run the client code, you should see the following output for the service side:
    
 
    ```
  
    [ INFO] [1729448362.219009300]: Call to ik_service returned [8] solutions
    [ INFO] [1729448362.219512604]: Solution Num: 1
    [ INFO] [1729448362.219521016]: Joint angle [1]: 5.9491 
    [ INFO] [1729448362.219525757]: Joint angle [2]: 5.3502 
    [ INFO] [1729448362.219529881]: Joint angle [3]: 2.1023 
    [ INFO] [1729448362.219534278]: Joint angle [4]: 0.4015 
    [ INFO] [1729448362.219538355]: Joint angle [5]: 1.5708 
    [ INFO] [1729448362.219542655]: Joint angle [6]: 1.2367 
    [ INFO] [1729448362.219551431]: Solution Num: 2
    [ INFO] [1729448362.219555764]: Joint angle [1]: 5.9491 
    [ INFO] [1729448362.219559854]: Joint angle [2]: 1.0522 
    [ INFO] [1729448362.219563753]: Joint angle [3]: 4.1809 
    [ INFO] [1729448362.219567561]: Joint angle [4]: 2.6209 
    [ INFO] [1729448362.219571608]: Joint angle [5]: 1.5708 
    [ INFO] [1729448362.219575615]: Joint angle [6]: 1.2367 
    [ INFO] [1729448362.219579447]: Solution Num: 3
    [ INFO] [1729448362.219583433]: Joint angle [1]: 5.9491 
    [ INFO] [1729448362.219587444]: Joint angle [2]: 5.2220 
    [ INFO] [1729448362.219592569]: Joint angle [3]: 2.5303 
    [ INFO] [1729448362.219599139]: Joint angle [4]: 3.2433 
    [ INFO] [1729448362.219603913]: Joint angle [5]: 4.7124 
    [ INFO] [1729448362.219607888]: Joint angle [6]: 4.3783 
    [ INFO] [1729448362.219612568]: Solution Num: 4
    [ INFO] [1729448362.219616627]: Joint angle [1]: 5.9491 
    [ INFO] [1729448362.219621314]: Joint angle [2]: 1.2574 
    [ INFO] [1729448362.219625398]: Joint angle [3]: 3.7529 
    [ INFO] [1729448362.219630095]: Joint angle [4]: 5.9853 
    [ INFO] [1729448362.219634175]: Joint angle [5]: 4.7124 
    [ INFO] [1729448362.219638744]: Joint angle [6]: 4.3783 
    [ INFO] [1729448362.219643333]: Solution Num: 5
    [ INFO] [1729448362.219647875]: Joint angle [1]: 3.4757 
    [ INFO] [1729448362.219652591]: Joint angle [2]: 1.8842 
    [ INFO] [1729448362.219657278]: Joint angle [3]: 2.5303 
    [ INFO] [1729448362.219662001]: Joint angle [4]: 3.4395 
    [ INFO] [1729448362.219666474]: Joint angle [5]: 1.5708 
    [ INFO] [1729448362.219671247]: Joint angle [6]: 5.0464 
    [ INFO] [1729448362.219675847]: Solution Num: 6
    [ INFO] [1729448362.219680404]: Joint angle [1]: 3.4757 
    [ INFO] [1729448362.219685036]: Joint angle [2]: 4.2028 
    [ INFO] [1729448362.219689661]: Joint angle [3]: 3.7529 
    [ INFO] [1729448362.219694234]: Joint angle [4]: 6.1815 
    [ INFO] [1729448362.219698881]: Joint angle [5]: 1.5708 
    [ INFO] [1729448362.219703428]: Joint angle [6]: 5.0464 
    [ INFO] [1729448362.219708302]: Solution Num: 7
    [ INFO] [1729448362.219712818]: Joint angle [1]: 3.4757 
    [ INFO] [1729448362.219717451]: Joint angle [2]: 2.0894 
    [ INFO] [1729448362.219722094]: Joint angle [3]: 2.1023 
    [ INFO] [1729448362.219726752]: Joint angle [4]: 0.5207 
    [ INFO] [1729448362.219731338]: Joint angle [5]: 4.7124 
    [ INFO] [1729448362.219735975]: Joint angle [6]: 1.9049 
    [ INFO] [1729448362.219740575]: Solution Num: 8
    [ INFO] [1729448362.219745230]: Joint angle [1]: 3.4757 
    [ INFO] [1729448362.219749898]: Joint angle [2]: 4.0746 
    [ INFO] [1729448362.219754424]: Joint angle [3]: 4.1809 
    [ INFO] [1729448362.219758971]: Joint angle [4]: 2.7401 
    [ INFO] [1729448362.219763608]: Joint angle [5]: 4.7124 
    [ INFO] [1729448362.219768231]: Joint angle [6]: 1.9049 
  ```
- In the service side, you will see the following output:
  ```
      [ INFO] [1729448362.218754408]: pose_ik service was called.
      [ INFO] [1729448362.218798789]: Number of solutions inverse kinematics solutions: 8
  
  ```

  
### Manually call the service and find no solution

  - Run the following command to see what will happen if the inverse kinematics could not find any solution:
    
```
    
     rosservice call /pose_ik "part_pose: {position: {x: 5.0, y: 4.0, z: 3.0 }, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" 

```

- What will you see in the service ouput:

  ```

    [ERROR] [1729448658.172994442]: Inverse Kinematics algorithm could not find a solution for Target position: (5.00, 4.00, 3.00)
    
  ```

- What will you see in the output after rosservice call command:

  ```
  
    ERROR: service [/pose_ik] responded with an error: b
  
  ```

### Manually call the service and find some solution

  - Run the following command to see what will happen if the inverse kinematics find some solution:
    
```
  rosservice call /pose_ik "part_pose: {position: {x: 0.5, y: 0.5, z: 1.0 }, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" 

```

- What will you see in the service ouput:

    ```
    [ INFO] [1729448704.919337545]: pose_ik service was called.
    [ INFO] [1729448704.919420441]: Number of solutions inverse kinematics solutions: 4

  ```

- What will you see in the output after rosservice call command:
  
    ```
        num_sols: 4
        joint_solutions: 
      - 
        joint_angles: [0.5514215705384928, 4.932377054147698, 0.6531741153528371, 5.410023118063741, 4.712388980384689, 5.263810550923182]
      - 
        joint_angles: [0.5514215705384928, 5.562843333299117, 5.630011191826749, 6.085905069617996, 4.712388980384689, 5.263810550923182]
      - 
        joint_angles: [4.160967409846197, 3.861934627470262, 0.6531741153528371, 3.3388728911513836, 1.5707963267948954, 5.731763736641094]
      - 
        joint_angles: [4.160967409846197, 4.492400906621682, 5.630011191826749, 4.0147548427056385, 1.5707963267948954, 5.731763736641094]
      - 
        joint_angles: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      - 
        joint_angles: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      - 
        joint_angles: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      - 
        joint_angles: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    
  ```


## Authors

  - **Ilke Kas** - *PhD at ECSE* -
    [Ilke Kas Github](https://github.com/ilke-kas)

