# [URDF](http://wiki.ros.org/urdf/Tutorials)

## [模型描述](http://wiki.ros.org/urdf/XML)

关节属性关系<img src="/home/ou/.config/Typora/typora-user-images/image-20201013234830120.png" alt="image-20201013234830120" style="zoom:10%;" />

连杆属性关系<img src="/home/ou/.config/Typora/typora-user-images/image-20201013234930525.png" alt="image-20201013234930525" style="zoom:15%;" />,当子连杆未定义oigin则origin为joint_frame，没有parent的link为rootlink，一个robot只有一个rootlink.



写一下基本结构、具体内容对照标题链接书写。

```xml
<?xml version="1.0"?>
<robot name="rb_name">
    <link>
        <!-- link description -->
    </link>

    <joint>
        <!-- joint description -->
    </joint>

    <transmission>
        <!-- urdf extention for descripte the relationship between actuator and joint -->
    </transmission>
    <gazebo>
        <!-- urdf exention for gazebo simulation env  -->
        <!-- http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros -->
    </gazebo>
</robot>

```

### 常用指令

* 检查urdf合法性

  `sudo apt-get install liburdfdom-tools`

  然后`check_urdf ~/workspace/ros_ws/src/wheel5/urdf/wheel5.urdf`
  
* 可视化文档

  `urdf_to_graphiz pr2.urdf`

### 模型模板

```xml
<?xml version="1.0"?>
<robot name="test_robot">
    <link name="link1">
        <visual>
            <geometry>
                <cylinder length="0.6" radius="0.2" />
            </geometry>
        </visual>
    </link>
    <link name="link4" >
        <visual>
            <origin rpy="0 -1.57 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="package://urdf_csdn/urdf/mesh/knife.stl"/>
            </geometry>
        </visual>
    </link>
    <joint name="joint1" type="fixed">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="0.22 0 0.6" rpy="0 1.57 0" />
   </joint>
    <joint name="head_swivel" type="continuous">
        <parent link="base_link"/>
        <child link="head"/>
        <axis xyz="0 0 1"/>
        <origin xyz="0 0 0.3"/>
    </joint>
    <joint name="gripper_extension" type="prismatic">
        <parent link="base_link"/>
        <child link="gripper_pole"/>  
        <limit effort="1000.0" lower="-0.38" upper="0" velocity="0.5"/>
        <origin rpy="0 0 0" xyz="0.19 0 0.2"/>
    </joint>
</robot>
```

### gazebo拓展标签

http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros

# [xacro](http://wiki.ros.org/xacro)

## 转URDF

``rosrun xacro xacro.py `rospack find pr2_description`/robots/pr2.urdf.xacro -o /tmp/pr2.urdf``

## 常用宏格式

" " 内的" " 要写作' '

### 宏语句

* 定义宏常量：

  * 常量：

    `<xacro:property name="rotor_velocity_slowdown_sim" value="10" />`

    调用宏常亮：`${ sin(M_PI/2) }`语句。语句中可以调用并解析宏与进行数学运算，可可以`"$(find xacro)" `来

  * 常量块：

    ```xml
    <xacro:property name="body_inertia">
        <inertia ixx="0.0347563" ixy="0.0" ixz="0.0" iyy="0.0458929" iyz="0.0" izz="0.0977" /> <!-- [kg m^2] [kg m^2] [kg m^2] [kg m^2] [kg m^2] [kg m^2] -->
    </xacro:property>
    ```

    调用常量宏块：`<xacro:insert_block name="body_inertia" />`

* 定义宏函数

  * 宏函数：其中带*的参数为宏块，而**参数可以传入包含任意多子元素的元素

    ```xml
    <xacro:macro name="box_inertial" params="x y z mass *origin **content" >
        <inertial>
            <mass value="${mass}" />
            <xacro:insert_block name="origin" />
            <xacro:insert_block name="content" />
            <xacro:box_inertia x="${x}" y="${y}" z="${z}" mass="${mass}" />
        </inertial></xacro:macro>
    ```
    
    ```xml
    调用：带*或**参数在调用时需要按定义顺序(左到右)写。
      
      ```xml
      <xacro:box_inertial x="0"y="0"z="0" mass="${mass}">
          <xacro:insert_block name="inertia_origin" />  <!-- 或 <pose xyz="0 1 0" rpy="0 0 0" /> -->
          <container>
              <color name="yellow"/>
              <mass>0.1</mass>
          </container> 
      </xacro:box_inertial>
      ----------
      好像说也可以这样调用，如果没块参数的话
      <xacro:box_inertial x="0"y="0"z="0" mass="${mass}"/>
    ```
    
    

* 包含宏文件

  `<xacro:include filename="$(find rotors_description)/urdf/$(arg mav_name)_base.xacro" />`
  
  `<xacro:include filename="other_file.xacro" ns="namespace"/>` => 访问元素  `${namespace.property}`
  
* 条件语句

  类似于#ifndef xxx 条件编译

  ```xml
  <xacro:if value="<expression>"> <!--          value="${var.startswith('use') and var.endswith('it')}"        value="${ var == '123' }"      -->
      <... some xml code here ...>
  </xacro:if>
  ```

  

### 传入宏与启动

```xml
（圆括号的是launch参数宏）
<param name="robot_description" command="
    $(find xacro)/xacro '$(arg model)'
    enable_logging:=$(arg enable_logging)
    enable_ground_truth:=$(arg enable_ground_truth)
    enable_mavlink_interface:=$(arg enable_mavlink_interface)
    log_file:=$(arg log_file)
    wait_to_record_bag:=$(arg wait_to_record_bag)
    mav_name:=$(arg mav_name)
    namespace:=$(arg namespace)"
  />
---------------------
在 ${model}="$(find rotors_description)/urdf/mav_generic_odometry_sensor.gazebo"
<robot name="$(arg mav_name)" xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- Instantiate the mav-->
</robot>
这里的robot名字"$(arg mav_name)"  就是从robot_description 中后面附带的,   mav_name:=$(arg mav_name)(这个是launch宏)。
然后用以下节点来加载模型。
 <node name="spawn_$(arg namespace)" pkg="gazebo_ros" type="spawn_model"
   args="-param robot_description
         -urdf
         -x $(arg x)
         -y $(arg y)
         -z $(arg z)
         -model $(arg namespace)"
   respawn="false" output="screen">
  </node>
```

##  模板

```xml
<?xml version="1.0"?>
<robot name="robot_name" xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:include filename="$(find rotors_description)/urdf/$(arg mav_name)_base.xacro" /><!-- 包含其他xacro文件 -->
    <xacro:property name="rotor_velocity_slowdown_sim" value="10" />
</robot>
```

## 例程

https://blog.csdn.net/weixin_40863346/article/details/80498424

http://wiki.ros.org/urdf/Examples

推荐看ETH的无人机rotors仿真的xacro文档结构：第一层各种形状的inertial宏函数与传感器宏。第二层... 第三层...

##  



# 定义坐标的问题

## link和joint坐标关系

<img src="/home/ou/.config/Typora/typora-user-images/image-20201016005856673.png" alt="image-20201016005856673" style="zoom:67%;" />

link_frame为其与父link相连的joint的joint_frane为本link的frame。而joint的frame为本joint的orgin相对于其父link。

link中有orgin的只有三个地方，分别是visual、inertial、collision。origin为他们原点相对于本link_frame的坐标。