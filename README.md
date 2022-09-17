# Fundindo PRO 4WD + openmanipulator

Open your command prompt and go to your home directory:

```
cd ~
```


If not yet there, make a new catkin workspace for all your robotic setups:

```
mkdir -p ~/robotic_setups/src
cd ~/robotic_setups
catkin_make 
```


Then go to your src folder and make a package with your new setup openmanipulator_with_wheeltec:


```
cd ~/robotic_setups/src
catkin_create_pkg openmanipulator_with_wheeltec
```

This will create a openmanipulator_with_wheeltec folder which contains a package.xml and a CMakeLists.txt. Then open package.xml and add the following lines after the line <buildtool_depend>catkin</buildtool_depend>.

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


Optionally, modify email and licence, version tags.

Then create 4(+2) folders: launch, rviz, urdf and meshes (with visual and collision folders):


```
mkdir ~/robotic_setups/src/openmanipulator_with_wheeltec/{launch,rviz,urdf,meshes,meshes/visual,meshes/collision}
```

Copy your meshes into meshes/visual and meshes/collision.
