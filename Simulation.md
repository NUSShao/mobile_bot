# 利用编辑好的模型进行仿真

## 在Gazebo中生成模型

### 运行状态发布器

首先，在生成模型之前，我们需要在ROS2中运行状态发布器：

`ros2 launch (package名字) rsp.launch.py use_sim_time:=true`

注意，这里将use_sim_time设置为了True，是为了让ROS2节点的时间与Gazebo仿真环境中的时间对齐。

另外，你可以启动一个额外的Terminal，用以下指令检查是否设置成功：

`ros2 param get /robot_state_publisher use_sim_time`

### 启动gazebo环境

另启动一个Terminal，输入以下指令，启动Gazebo：

`ros2 launch gazebo_ros gazebo.launch.py`

### 生成机器人实体

接下来，我们使用gazebo_ros包中自带的功能生成机器人实体：

`ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_bot`

随后，我们的小车就出现在Gazebo中了：

![小车出现在Gazebo中](img/BotInGazebo.jpg)

## 打包启动指令

依次重新运行前面的三条指令比较冗余，因此JoshNewans大佬为我们写好了一份[更方便的launch文件](https://github.com/joshnewans/articubot_one/blob/adb08202d3dafeeaf8a3691ddd64aa8551c40f78/launch/launch_sim.launch.py)

我们只需在package文件夹下的launch子文件夹中新建一个`rsp_sim.launch.py`的文件，将内容复制进去即可。

同时，别忘了为新建的文件重新build一下你的package：

```
cd ~/(工作空间名字)
colcon build --symlink-install
```

完成之后，运行一下我们的launch文件就可以一次性将机器人模型放到Gazebo中了：

`ros2 launch mobile_bot rsp_sim.launch.py`