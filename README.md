# Fundindo PRO 4WD + openmanipulator

## 1. Prepare your catkin workspace

Abra seu prompt de comando e vá para o diretório home:

```
cd ~
```


Se ainda não existe, faça um novo catkin workspace para todos os seus robotic setups:

```
mkdir -p ~/robotic_setups/src
```
```
cd ~/robotic_setups
```
```
catkin_make 
```


Então vá para seu ```src``` e faça um pacote com seu novo setup openmanipulator_with_wheeltec:

```
cd ~/robotic_setups/src
```
```
catkin_create_pkg openmanipulator_with_wheeltec
```
Isso vai criar uma pasta openmanipulator_with_wheeltec que contem um ```package.xml``` e um ```CMakeLists.txt```. Então abra o ```package.xml``` e adicione as linhas a seguir depois da linha ```<buildtool_depend>catkin</buildtool_depend>```:


```
<buildtool_depend>catkin</buildtool_depend>
<test_depend>roslaunch</test_depend>
<build_export_depend>joint_state_publisher</build_export_depend>
<build_export_depend>robot_state_publisher</build_export_depend>
<build_export_depend>rviz</build_export_depend>
<build_export_depend>xacro</build_export_depend>
<exec_depend>joint_state_publisher</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>rviz</exec_depend>
<exec_depend>xacro</exec_depend>
```


Opicionalmente, modifique  ```email``` ,```licence``` e ```version```.

Então crie 4(+2) pastas: launch, rviz, urdf and meshes (com as pastas visual e collision):

```
mkdir ~/robotic_setups/src/openmanipulator_with_wheeltec/{launch,rviz,urdf,meshes,meshes/visual,meshes/collision}
```

Copie suas malhas para dentro de ```meshes/visual``` e ```meshes/collision```.



## 2. Create xacros and generate urdf

Go to the urdf folder and create a xacro file for openmanipulator with the text editor of your choice (e.g. gedit):

```
cd ~/robotic_setups/src/openmanipulator_with_wheeltec/urdf
```
```
gedit open_manipulator_append.xacro
```
Então copiei o seguinte arquivo:
```
<?xml version="1.0"?>
<!-- Open_Manipulator Chain -->
<robot xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link1.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="grey"/>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link1.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="3.0876154e-04 0.0000000e+00 -1.2176461e-04" />
      <mass value="7.9119962e-02" />
      <inertia ixx="1.2505234e-05" ixy="0.0" ixz="-1.7855208e-07"
               iyy="2.1898364e-05" iyz="0.0"
               izz="1.9267361e-05" />
    </inertial>
  </link>

  <!-- Joint 1 -->
  <joint name="joint1" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.012 0.0 0.017" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit velocity="4.8" effort="1" lower="${-pi*0.9}" upper="${pi*0.9}" />
  </joint>

  <!--  Link 2 -->
  <link name="link2">
    <visual>
      <origin xyz="0 0 0.019" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link2.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="grey"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.019" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link2.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="-3.0184870e-04 5.4043684e-04 ${0.018 + 2.9433464e-02}" />
      <mass value="9.8406837e-02" />
      <inertia ixx="3.4543422e-05" ixy="-1.6031095e-08" ixz="-3.8375155e-07"
               iyy="3.2689329e-05" iyz="2.8511935e-08"
               izz="1.8850320e-05" />
    </inertial>
  </link>

  <!--  Joint 2 -->
  <joint name="joint2" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0.0 0.0 0.0595" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit velocity="4.8" effort="1" lower="${-pi*0.57}" upper="${pi*0.5}" />
  </joint>

  <!--  Link 3 -->
  <link name="link3">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link3.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="grey"/>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link3.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="1.0308393e-02 3.7743363e-04 1.0170197e-01" />
      <mass value="1.3850917e-01" />
      <inertia ixx="3.3055381e-04" ixy="-9.7940978e-08" ixz="-3.8505711e-05"
               iyy="3.4290447e-04" iyz="-1.5717516e-06"
               izz="6.0346498e-05" />
    </inertial>
  </link>

  <!--  Joint 3 -->
  <joint name="joint3" type="revolute">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="0.024 0 0.128" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit velocity="4.8" effort="1" lower="${-pi*0.3}" upper="${pi*0.44}" />
  </joint>

  <!--  Link 4 -->
  <link name="link4">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link4.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="grey"/>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link4.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="9.0909590e-02 3.8929816e-04 2.2413279e-04" />
      <mass value="1.3274562e-01" />
      <inertia ixx="3.0654178e-05" ixy="-1.2764155e-06" ixz="-2.6874417e-07"
               iyy="2.4230292e-04" iyz="1.1559550e-08"
               izz="2.5155057e-04" />
    </inertial>
  </link>

  <!--  Joint 4 -->
  <joint name="joint4" type="revolute">
    <parent link="link4"/>
    <child link="link5"/>
    <origin xyz="0.124 0.0 0.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit velocity="4.8" effort="1" lower="${-pi*0.57}" upper="${pi*0.65}" />
  </joint>

  <!--  Link 5 -->
  <link name="link5">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link5.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="grey"/>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link5.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="4.4206755e-02 3.6839985e-07 8.9142216e-03" />
      <mass value="1.4327573e-01" />
      <inertia ixx="8.0870749e-05" ixy="0.0" ixz="-1.0157896e-06"
               iyy="7.5980465e-05" iyz="0.0"
               izz="9.3127351e-05" />
    </inertial>
  </link>

  <!--  Gripper joint -->
  <joint name="gripper" type="prismatic">
    <parent link="link5"/>
    <child link="gripper_link"/>
    <origin xyz="0.0817 0.021 0.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit velocity="4.8" effort="1" lower="-0.010" upper="0.019" />
  </joint>

  <!--  Gripper link -->
  <link name="gripper_link">
    <visual>
      <origin xyz="0.0 0.0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link_grip_l.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="grey"/>
    </visual>

    <collision>
      <origin xyz="0.0 0.0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link_grip_l.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0" />
      <mass value="1.0e-03" />
      <inertia ixx="1.0e-03" ixy="0.0" ixz="0.0"
               iyy="1.0e-03" iyz="0.0"
               izz="1.0e-03" />
    </inertial>
  </link>

  <!--  Gripper joint sub -->
  <joint name="gripper_sub" type="prismatic">
    <parent link="link5"/>
    <child link="gripper_link_sub"/>
    <origin xyz="0.0817 -0.021 0" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit velocity="4.8" effort="1" lower="-0.010" upper="0.019" />
    <mimic joint="gripper" multiplier="1"/>
  </joint>

  <!--  Gripper link sub -->
  <link name="gripper_link_sub">
    <visual>
      <origin xyz="0.0 -0.0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link_grip_r.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="grey"/>
    </visual>

    <collision>
      <origin xyz="0.0 -0.0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://open_manipulator_description/meshes/chain_link_grip_r.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0" />
      <mass value="1.0e-03" />
      <inertia ixx="1.0e-03" ixy="0.0" ixz="0.0"
               iyy="1.0e-03" iyz="0.0"
               izz="1.0e-03" />
    </inertial>
  </link>

  <!-- end effector joint -->
  <joint name="end_effector_joint" type="fixed">
    <origin xyz="0.126 0.0 0.0" rpy="0 0 0"/>
    <parent link="link5"/>
    <child link="end_effector_link"/>
  </joint>

  <!-- end effector link -->
  <link name="end_effector_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.01 0.01 0.01" />
      </geometry>
      <material name="red"/>
     </visual>

    <inertial>
      <origin xyz="0 0 0" />
      <mass value="1.0e-03" />
      <inertia ixx="1.0e-03" ixy="0.0" ixz="0.0"
               iyy="1.0e-03" iyz="0.0"
               izz="1.0e-03" />
    </inertial>
  </link>

  <!-- Realsense Camera Joint Position -->
<!--   <joint name="camera_joint" type="fixed">
    <axis xyz="0 1 0" />
    <origin xyz="0.070 0.032 0.052" rpy="0 0 0"/> 
    <parent link="link5"/>
    <child link="camera_link"/>
  </joint>

  <link name="camera_link"/> -->
  
</robot>

```

