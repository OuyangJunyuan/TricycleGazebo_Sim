# [SDF](http://sdformat.org/spec?ver=1.6&elem=sdf)

[和URDF文件转换](https://zhuanlan.zhihu.com/p/129661375)

sdf的建议书写顺序：

* Add a linkSet the collision element
* Set the visual element
* Set the inertial properties
* Go to 1 until all links have been added
* Add all joints (if any)
* Add all plugins (if any)



## 模型模板

[具体属性查看](http://sdformat.org/spec?ver=1.6&elem=sdf)

```xml
<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='model_name'>
    <link name='link_0'>
      <pose frame=''>x y z r p y</pose>
      <inertial>   </inertial>
      <visual name='visual'>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <geometry>
          <box>
            <size>1 0.53331 0.345017</size>
          </box>
        </geometry>
      </visual>
      <collision name='collision'>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <geometry>
          <box>
            <size>1 0.53331 0.345017</size>
          </box>
        </geometry>
      </collision>
    </link>
      
    <joint name='link_0_JOINT_0' type='revolute'>
      <parent>link_0</parent>
      <child>link_1</child>
      <pose frame=''>0 0 0 0 -0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <static>0</static>
    <allow_auto_disable>1</allow_auto_disable>
    <plugin name='model_plugins' filename='libmymodel_plugin.so'/>
  </model>
</sdf>
```

[如何设置惯性参数](http://gazebosim.org/tutorials?tut=inertia&cat=build_robot)：对于3D外形的link可以用meshlab或solidworks计算其惯性参数如质量、质心、惯性矩阵。

---

可视化层数：

对于gazebo可视化模型，可以通过控制可现实层数和指定模型的各个部件的层数来控制哪些东西可以显示。

然后在gazebo 的client中左栏有一个layer分栏。<img src="/home/ou/.config/Typora/typora-user-images/image-20201005235914498.png" alt="image-20201005235914498" style="zoom:67%;" />

```xml
<visual name='visual_0'>
  <meta>
    <layer>0</layer>
  </meta>
  ...
</visual>
```

### GAZEBO中创建模型

**添加joint**：在模型编辑器的右侧图像界面的顶栏有一个双箭头符号，这个就是joint添加器。

以旋转关节为例：

<img src="/home/ou/.config/Typora/typora-user-images/image-20201005180145624.png" alt="image-20201005180145624" style="zoom:50%;" />

<img src="/home/ou/.config/Typora/typora-user-images/image-20201005175737926.png" alt="image-20201005175737926" style="zoom:40%;" />

* fixed：刚体固连、零自由度

  * screw：螺旋连接、单自由度运动，由一个平动和旋转的复合
  * revolute：单角度限制的旋转关节、单自由度旋转
  * universal：万向节、双自由度旋转
  * revolute2：两个串联的revolute、双自由度旋转
  * ball：球(套)关节、三自由度旋转
  * parismatic：上下限滑轨、单自由度平动
  * gearbox：齿轮组、变换转速


* 关节轴：表示旋转轴，采用旋转向量方式

* 部件对齐：父子部件在xyz方向上以 min center max 方式对齐，其中还可以勾选reverse表示，在父原件内还是外部对齐。

  默认是以子部件对齐到父部件的，也可以选择parent to children 来反过来。

  这样就可以用来贴合两个部件的表面。



**添加link**：左侧菜单有一个insert，可以插入基础模型，也可以插入自定义形状。

自定义类型可以插入3Dmesh如.ada、.stl 或.svg将2D图拉伸成3D

点击一个简单部件到GUI中放置，然后双击物体弹出选项框，其中有：部件、视觉、碰撞三分栏。

* 在部件中可以设置物体的位姿、动力学、密度、惯性
* 在视觉中可以设置物体的外形、阴影、雷达反射率、透明度、位姿(以部件自身参考系为参考)、几何形状、材质
* 在碰撞中可以设置物体的雷达反射率、最大接触数、位姿(以部件自身参考系为参考)、几何碰撞外形、表面属性(摩擦、粘度等接触属性)



### 传感器

[传感器SDF元素](http://sdformat.org/spec?ver=1.6&elem=sensor)

可以通过在model中插入sdf片段，也可以在模型上直接固连fixed另一个module为link 即使用嵌套modle。

传感器是需要依附在link的，所以可以在link中

---

噪声属性：

```xml
<sensor>
    ......
    <ray/camera...>
        ......
        <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
		</noise>
        ......
        </ray/camera...>
    ......
</sensor>
```

---

失真：可以先用opencv标定真实相机的畸变参数，然后用这个来使得仿真更真实。

[畸变原理](https://en.wikipedia.org/wiki/Distortion_(optics)#Software_correction)

```xml
<sensor>
    ......
    <ray/camera...>
        ......
        <distortion>
            <k1>-0.25</k1>
            <k2>0.12</k2>
            <k3>0.0</k3>
            <p1>-0.00028</p1>
            <p2>-0.00005</p2>
            <center>0.5 0.5</center>
        </distortion>
        ......
        </ray/camera...>
    ......
</sensor>

```



---

雷达：sdf

```xml
<sensor name="laser" type="ray">
        <pose>0.01 0 0.03 0 -0 0</pose>
        <ray>
          <scan>
            <horizontal>
              <samples>640</samples>
              <resolution>1</resolution>
              <min_angle>-2.26889</min_angle>
              <max_angle>2.268899</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.08</min>
            <max>10</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="laser" filename="libRayPlugin.so" />
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
 </sensor>
```

或直接插入模块

```xml
  <modle>
      <include>
          <uri>model://hokuyo</uri>
          <pose>0.2 0 0.2 0 0 0</pose>
      </include>
      <joint name="hokuyo_joint" type="fixed">
          <child>hokuyo::link</child>
          <parent>chassis</parent>
    </joint>
</modle>
```

---

相机： [跳转到下文的ROS_Gazebo_相机关联](#ROS _GAZEBO_Camera插件)

```xml
<sensor name='my_camera' type='camera'>
    <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
            <width>1920</width>
            <height>1080</height>
        </image>
        <clip>
            <near>0.1</near>
            <far>100</far>
        </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>30</update_rate>
</sensor>

```

```xml
在camera下增加，可以自动保存图像为视频在磁盘中
<save enabled="true">
    <path>/tmp/camera_save_tutorial</path>
</save>
        
```

---

力、力矩传感器:一般用在fixed关节，也可以靠近revolute关节放置。

```xml
<joint>
    ...
    <sensor name="force_torque" type="force_torque">
        <update_rate>30</update_rate>
        <force_torque>
            <frame>child</frame>
            <measure_direction>child_to_parent</measure_direction>
        </force_torque>
    </sensor>
    ...
</joint>

```

如果出问题，就把世界标签子层中设置

```xml
    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.0</sor> <!-- Important, see issue #2209 -->
          <use_dynamic_moi_rescaling>false</use_dynamic_moi_rescaling>
        </solver>
      </ode>
    </physics>
```



---

接触传感器

```xml
  <model name="box">
      <link name="link">
        <pose>0 0 0.5 0 0 0</pose>

        <collision name="box_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>

        <sensor name='my_contact' type='contact'>
          <contact>
            <collision>box_collision</collision>
          </contact>
        </sensor>
      </link>
    </model>
```



## 世界创建

世界：该术语用于描述机器人和物体（例如建筑物，桌子和灯光）的集合，以及包括天空，环境光和物理属性的全局参数。

静态：SDF中标记为```<static>true</static>```的物体只有几何碰撞，如果不想让一个物体动就设置为static可以提高计算性能。

动态：没有static标记或设置乘false的实体，同时具有惯性和碰撞属性。

### GAZEBO中创建

* 创建：构建可以直接在gui中插入各种模型后。<img src="/home/ou/.config/Typora/typora-user-images/image-20201006003152151.png" alt="image-20201006003152151" style="zoom:67%;" />

  * GUI设置网格大小
  * scene 场景：设置地板颜色、背景色、是否有阴影
  * physics物理：设置物理引擎、物理计算更新频率、重力、磁场、算法约束、算法步长
  * atmosphere大气：气温、气压、密度
  * wind风：风速

* 保存：```file-save world as ``` 就可以保存当前gazebo环境为xxxx.world文件

* ```gazebo my_world.sdf```

### 世界模板

  ```xml
  <?xml version="1.0" ?>
  <sdf version="1.4">
    <world name="default">
      <include>
        <uri>model://ground_plane</uri>
      </include>
      <include>
        <uri>model://sun</uri>
      </include>
        
        
      <include>
        <uri>model://gas_station</uri>
        <name>gas_station</name>
        <pose>-2.0 7.0 0 0 0 0</pose>
      </include>
        
	  <plugin name="world_plugins" filename="libworld_plugins.so" />	
    </world>
  </sdf>
  ```

## 文件包含与模块调用

* 一个模型的SDF可以包含别的model包作为一个model。就是说可以存在多model标签。
* 不同model之间用modlename::joint1 访问元素。

以上include和多module都属于嵌套module写法。

```xml
<include>
  <uri>model://hokuyo</uri>  //这个名字是模型文件夹或SDF中模型的名字，而不是.config文件中的Name属性(这个在Gazebo模型库中显示)
  <pose>0 0 0.2 0 0 0</pose>
</include>

 <joint name="hokuyo_joint" type="fixed">
    <child>hokuyo::link</child>		//hokuyo是一个导入的模块，其有一个link名字叫link，用::来访问模型中的内部元素名字
    <parent>chassis</parent>
 </joint>
```

文件路径在ros+gazebo中可以用 `package://package_name`获取ros软件包路径

```xml
<collision>
    <origin  xyz="0 0 0"   rpy="0 0 0" />
    <geometry> 
        <mesh   filename="package://wheel5/meshes/bar_10.STL" />   
    </geometry>
</collision>
```

##  ROS启动模型

```xml
<launch>
    ...
    <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find MYROBOT_gazebo)/worlds/MYROBOT.world"/>
        <!-- more default parameters can be changed here -->
    </include>
    ...
</launch>
```

如果模型还是sdf而不是模型包，则无法用launch一个world文件方式启动，需要：

```xml
<param name="robot_description" command="$(find xacro)/xacro.py $(find pr2_description)/robots/pr2.urdf.xacro" />

<node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model pr2" />
--------------------
`<node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-file $(find baxter_description)/urdf/baxter.urdf -urdf  -x 0 -y 0 -z 1 -model model_name" />`
--------------------
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="TRUE"/>
  </node>

  <!-- Combine joint values in TF format-->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>
```





##  解析API 

  `if (_sdf->HasElement("velocity"))` `  velocity = _sdf->Get<double>("velocity");`

