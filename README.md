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
gedit measurement_tool.xacro
```
