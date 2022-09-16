update : 2018.11.21

***run command:***


* 启动 aubo_i5 moveit 运行环境

```
roslaunch aubo_i5_moveit_config moveit_planning_execution.launch robot_ip:=`<your robot IP address>`

```

* 启动 aubo_i5 demo

```bash
# for ros melodic
roslaunch aubo_demo MoveGroupInterface_To_Melodic.launch

# for ros kinetic 
roslaunch aubo_demo MoveGroupInterface_To_Kinetic.launch

# for ros indigo
roslaunch aubo_demo MoveGroupInterface_To_indigo.launch
```
