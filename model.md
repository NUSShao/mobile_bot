# 为你的机器人建模

## 文件格式

URDF（Unified Robot Description Format）和Xacro（XML Macro）文件在ROS项目中起着至关重要的作用，主要用于描述和配置机器人的模型。

## URDF文件

URDF 是一种基于 XML 的文件格式，专门用于定义机器人的物理模型。它详细描述了机器人的各个组成部分，包括关节、连杆、形状、惯性、碰撞检测和视觉元素。URDF 文件的主要作用如下：

**1.定义机器人结构**:URDF 用于描述机器人的连杆（links）和关节（joints）的几何结构、质量属性、碰撞模型和可视化模型。通过这种描述，ROS 系统能够理解机器人的形状和动态特性。

**2.支持仿真与可视化**:URDF 文件通常用于工具如 Gazebo（用于物理仿真）和 RViz（用于机器人状态的可视化）中，让开发者能够仿真和可视化机器人在虚拟环境中的行为。

**3.机器人状态推断**：通过 URDF 文件，ROS 可以推断机器人在不同时间的状态信息，如关节位置和姿态，从而为控制和感知模块提供数据。

## XACRO文件

Xacro 是一种基于 XML 的宏扩展格式，用于生成更简洁和模块化的 URDF 文件。Xacro 文件的主要功能包括：

**1.简化代码复用**：Xacro 允许使用宏、变量和参数，使得可以重用代码片段。例如，如果多个地方使用相同的组件，可以用 Xacro 编写一次宏，然后在需要的地方调用，从而减少重复代码。

**2.提高灵活性**：Xacro 文件通过参数化的方式定义机器人模型，可以根据不同的参数值自动生成适应不同配置的 URDF 文件。这在需要生成多个不同配置的机器人模型时非常有用。

**3.更好的管理复杂模型**：Xacro 使得管理大型和复杂的机器人模型更加容易。通过分解代码并使用宏和参数，可以更有效地组织模型的不同部分。

## 编写机器人外观框架XACRO文件

首先，Josh Newans提供的模板当中的description文件夹下，有一个robot.urdf.xacro范例文件。
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"  name="robot">

    <!-- Example link -->
    <link name="base_link"></link>

</robot>
```

我们将最后在这个文件中进行机器人模型的整合。

因此，我们首先新建一个robot_core.xacro文件，用于表示机器人外观框架。开始编写之前，先在robot.urdf.xacro文件中引用我们新建的xacro文件：

`<xacro:include filename="robot_core.xacro"/>`

随后，我们为新建的xacro文件套用模板(注意，这里的robot标签中删掉了name属性，因为这个文件只作为引用文件)：

```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <link name="base_link"></link>

</robot>
```

在开始编写文件之前，推荐大家从左侧Extension栏中安装XML插件，然后从右下角的文件类型处，将格式选择为XML格式：

![点击右下角](img/ChangeFileFormat1.jpg)

![搜索XML格式](img/ChangeFileFormat2.jpg)

接下来正式开始编写机器人主体框架。

首先，创造一些material标签，用来为模型的各个link上色：

```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <material name="white">
        <color rgba="1 1 1 1" />
    </material>

    <material name="orange">
        <color rgba="1 0.3 0.1 1" />
    </material>

    <material name="blue">
        <color rgba="0.2 0.2 1 1" />
    </material>

    <material name="black">
        <color rgba="0 0 0 1" />
    </material>

    <link name="base_link"></link>

</robot>
```

这里为了方便大家理解文件的构成，把整体代码放上去了。

后面为了简洁性，只展示新增的代码片段（[整体代码](https://github.com/BIT-Gs/mobile_bot/blob/main/description/robot_core.xacro)）

接下来，我们先不管base link，先把底盘（chassis）部分写好，我们需要首先利用一个fixed类型的joint来连接Chassis和base link，然后再编写Chassis link：

```

```
