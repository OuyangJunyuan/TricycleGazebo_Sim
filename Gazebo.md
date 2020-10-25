# Gazebo记录	

## [错误解决](#issue)

## API

[写代码不知道怎么用api可以查看](https://osrf-distributions.s3.amazonaws.com/gazebo/api/6.1.0/namespacegazebo.html)

[插件API](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Model.html)

## 介绍

Gazebo使用了一个分布式架构，其中有独立的库，用于物理模拟、呈现、用户界面、通信和传感器生成。

此外，gazebo还提供了两个可执行程序来运行模拟：服务器gzserver用于仿真物理、呈现和传感器客户端gzclient提供一个图形化界面来可视化并与仿真交互。
客户端和服务器使用gazebo通信库进行通信。通信库目前使用开放源码的Google Protobuf来进行消息序列化和针对传输机制的boost::ASIO。它支持发布/订阅通信范型。例如，模拟世界发布身体姿势更新，传感器生成和GUI将使用这些消息来生成输出。
该



机制允许对正在运行的仿真进行自省，并提供了一种方便的机制来控制Gazebo的各个方面。这个模式像ros一样。



术语：In the following context, the pose and twist of a rigid body object is referred to as its '''state''. An object also has intrinsic '''properties''', such as mass and friction coefficients.	In Gazebo, a '''body''' refers to a rigid body, synonymous to '''link''' in the URDF context. A Gazebo '''model''' is a conglomeration of bodies connected by '''joints'''.

## 常用命令

终端输入：gzserver启动gazebo服务，gzclient启动可视化界面，gazebo则等于同时输入两者。

服务器负责物理引擎仿真，服务器和客户端可视化界面是通过google的protobuf通信库完成的。



* 编辑模型：点击模型，右键编辑模型。
* 关节控制：右侧有...栏，往左拖可以打开速度、位置、力矩的控制器。但是需要点击对应模型，才会显示出其内部关节在这个栏目



### 记录回放

右上角有![image-20201006001205026](/home/ou/.config/Typora/typora-user-images/image-20201006001205026.png)

分别是截图、记录设置、参数曲线绘制、记录

回放必须用命令行启动```gazebo -u -p ~/logs/double_pendulum/2016-01-25T15\:09\:49.677400/gzserver/state.log```



### 曲线绘制

右上角plot，找到对应的属性，然后拖到右侧窗口，就可以显示出曲线了，而且可以点击右下角加号多个图标，或一个图标内拖入多个属性来展示多个曲线

<img src="/home/ou/.config/Typora/typora-user-images/image-20201006232134498.png" alt="image-20201006232134498" style="zoom:50%;" />





## 模型

### 模型路径设置

模型构建完毕后可以```export GAZEBO_MODEL_PATH=$HOME/gazebo_plugin_tutorial/models:$GAZEBO_MODEL_PATH```来添加路径到环境变量

查看环境变量可以用`env | grep GAZEBO_MODEL_PATH`

### [文件结构](http://gazebosim.org/tutorials?tut=inertia&cat=build_robot)与编写

```xml
└── ouyjy_car2wheel
    ├── model.config    		//除了顶层database有一个config来描述database，gazebo寻找包就是寻找这个配置文件，然后读取对应的.sdf文件加载。
    ├── meshes						 //用来存放网格文件如.stl和.ada文件 		
    └── materials					//只能用来包含textures和scripts
		├── textures				 		//纹理文件,图片格式 (jpg, png, etc)
		└── scripts							  //OGRE 材质脚本
    ├── plugins						   //插件源码和头文件				
    └── model.sdf				   //模型的仿真描述文件，注：引用模型时名字是模型文件夹或SDF中模型的名字，而不是.config文件中的Name属性(在Gazebo模型库中显示)

```

#### [SDF](./SDF.md)

#### [URDF](./URDF.md)





### 建筑物

[中文教程](https://www.jianshu.com/p/ec1f80892517)

http://gazebosim.org/tutorials?tut=building_editor&cat=build_world



### 3D地形

地形(Digital Elevation Model——DEM)构建

http://gazebosim.org/tutorials?tut=dem&cat=build_world



### 大量相同模型(如人群)

http://gazebosim.org/tutorials?tut=model_population&cat=build_world



## 节点与话题

[订阅话题专题教程](http://gazebosim.org/tutorials?tut=topics_subscribed&cat=transport)

[插件中使用发布者和接收者](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5)

列出所有话题```gz topic -l```

打印话题中的消息```gz topic -e /gazebo/default/box/link/my_contact``





---

发布者

```c++
transport::NodePtr node(new transport::Node());
node->Init(_parent->Name());
transport::PublisherPtr physicsPub = node->Advertise<msgs::Physics>("~/physics");
//话题类型有很多，可以自己输入msgs::自动补全查看，比如有Joint、Factory......


msgs::Physics physicsMsg;
physicsMsg.set_type(msgs::Physics::ODE);
physicsMsg.set_max_step_size(0.01);
// Change gravity
msgs::Set(physicsMsg.mutable_gravity(), ignition::math::Vector3d(0.01, 0, 0.1));
physicsPub->Publish(physicsMsg);

//因为世界只有一个，所以不区分发给谁。对于关节和部件等对象的信息，可以用
msg.set_name("xxx");//指定消息是发给哪个对象的
```

---

订阅者

```c++
class xxx :public xxxplugin
{
    loadfunction{
        std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";     
		this->sub = this->node->Subscribe(topicName,   &VelodynePlugin::OnMsg, this);
    }
	//回调函数写法为   public: void callback(const 消息类型 &形参名字)
	private: void OnMsg(ConstVector3dPtr &_msg)
	{
   	  	this->SetVelocity(_msg->x());
	}
}
```



发布到~/factory 可以插入模型

发布到~/physics 可以修改世界的物理参数

发布到~/light/modify 可以改变对应名字的light灯光如

```language-cpp
msgs::Light lightMsg;
lightMsg.set_name("post_light"); //接受者为名字为post_light的light
 lightMsg.set_range(15.0);
```

![image-20201006234244243](/home/ou/.config/Typora/typora-user-images/image-20201006234244243.png)





## 插件

[写代码不知道怎么用api可以查看](https://osrf-distributions.s3.amazonaws.com/gazebo/api/6.1.0/namespacegazebo.html)

[插件API](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Model.html)

<u>**如果把world文件认为是ros中的launch文件，则插件就是ros中的节点、服务、动作等。**</u>

插件是被编译成共享库的代码块，可以插入到仿真中。如果不想增加额外的通信开销(如用ros控制仿真)，可以使用插件。

插件的种类有很多种：world、model、sensor、system、vivsual、gui。不同种类的插件是由gazebo的不同组件管理的，比如模型插件是附加并控制gazebo中model的，同理world插件添加在world中、传感器插件添加在传感器中、系统插件在命令行中指定并在gazebo启动期间首先加载来控制启动过程。



根据想要什么功能来决定使用哪种插件类型：使用World插件控制世界属性，例如物理引擎，环境照明等。使用Model插件控制关节和模型状态。使用Sensor插件获取传感器信息并控制传感器属性。





### 路径设置

```export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_plugin_tutorial/build```

指定插件路径后，就可以在SDF中直接填写插件名字和库名字了(不需要绝对路径) ，gazebo会解析SDF然后搜索以上路径，加载对应共享库。

```<plugin name="hello_world" filename="libhello_world.so"/>```根据本插件类型，这句话要包含在对应元素下，比如是world插件就应该

```xml
  <world name="default">
    <plugin name="hello_world" filename="libhello_world.so"/>
  </world>
```



也可以用命令行插入插件

```gzserver -s <plugin_filename><world_file>```

```gzclient -g <plugin_filename>```



### 插件传参

sdf是xml，他只是准守了xml格式，具体的sdf属性是gazebo定义所以解析后有对应效果代码。

如果是自定义的属性比如 velocity，可以自己用sdf接口解析，然后写对应的处理代码。

```xml
<plugin name="velodyne_control" filename="libvelodyne_plugin.so">
  <velocity>25</velocity>
</plugin>
```

```c++
if (_sdf->HasElement("velocity"))
  velocity = _sdf->Get<double>("velocity");
//do something to use "velocity" 
```





### 世界插件

http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin

在gazebo.hh中跳转WorldPlugin可以看到定义

<img src="/home/ou/.config/Typora/typora-user-images/image-20201006111819493.png" alt="image-20201006111819493" style="zoom:67%;" />

该类虚写了3个成员函数需要用户继承后补充其实现：Load Init Reset。其中load(必须实现)在插件第一次创建且世界加载完毕后调用且该函数不能堵塞。

下面举例如何书写世界插件

#### 插入模型

```c++
#include <gazebo/gazebo.hh>  //必须的头文件，而gazebo/physics/physics.hh, gazebo/rendering/rendering.hh, or gazebo/sensors/sensors.hh 按需添加
//所有插件都要在gazebo名称空间下
namespace gazebo
{													//每种插件都必须从特定类型的插件类型中继承
	class my_worldplugin : public WorldPlugin
    {
    public: my_worldplugin() : WorldPlugin()
        {
            printf("Hello World!\n");
        }
	//由上图声明可以看出Load是纯虚函数，=0表示不需要在本类中实现即没有{}，所以子类中必须实现。并加载sdf文件。
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
        	//1.0 动态插入模型
        	  _world->InsertModelFile("model://box");
        	//2.0 预创建模型
                sdf::SDF sphereSDF;
                sphereSDF.SetFromString(
                   "<sdf version ='1.4'>\
                      <model name ='sphere'>\
                        <pose>1 0 0 0 0 0</pose>\
                        <link name ='link'>\
                          <pose>0 0 .5 0 0 0</pose>\
                          <collision name ='collision'>\
                            <geometry>\
                              <sphere><radius>0.5</radius></sphere>\
                            </geometry>\
                          </collision>\
                          <visual name ='visual'>\
                            <geometry>\
                              <sphere><radius>0.5</radius></sphere>\
                            </geometry>\
                          </visual>\
                        </link>\
                      </model>\
                    </sdf>");
                // Demonstrate using a custom model name.
                sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
                model->GetAttribute("name")->SetFromString("unique_sphere");
                _parent->InsertModelSDF(sphereSDF);
        
        	//3.0 通过消息系统插入模型
                {
                  // Create a new transport node
                  transport::NodePtr node(new transport::Node());

                  // Initialize the node with the world name
                  node->Init(_parent->Name());

                  // Create a publisher on the ~/factory topic
                  transport::PublisherPtr factoryPub =
                  node->Advertise<msgs::Factory>("~/factory");

                  // Create the message
                  msgs::Factory msg;

                  // Model file to load
                  msg.set_sdf_filename("model://cylinder");

                  // Pose to initialize the model to
                  msgs::Set(msg.mutable_pose(),
                      ignition::math::Pose3d(
                        ignition::math::Vector3d(1, -2, 0),
                        ignition::math::Quaterniond(0, 0, 0)));

                  // Send the message
                  factoryPub->Publish(msg);
                }
    	}
    };
    //一个宏，唯一的参数是本插件名，用来声明一个成员函数，这个函数是工厂函数，用来在gazebo底层调用以返回本插件实例。
    //对应插件类型有GZ_REGISTER_MODEL_PLUGIN, GZ_REGISTER_SENSOR_PLUGIN, GZ_REGISTER_GUI_PLUGIN, GZ_REGISTER_SYSTEM_PLUGIN and GZ_REGISTER_VISUAL_PLUGIN
    GZ_REGISTER_WORLD_PLUGIN(my_worldplugin)
   
}
```

其中通过消息插入模型是因为gazebo内部已经订阅了这个话题，然后会接收并寻找这个已在环境变量路径下的模型来创建。

![image-20201006174002575](/home/ou/.config/Typora/typora-user-images/image-20201006174002575.png)







#### 修改世界属性

```c++
#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

///创建一个世界插件，并且初始化一个传输节点，通过发送消息的方式来修改重力
namespace gazebo
{
  class WorldEdit : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
      // Create a new transport node
      transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      node->Init(_parent->Name());

      // Create a publisher on the ~/physics topic
      transport::PublisherPtr physicsPub =
        node->Advertise<msgs::Physics>("~/physics");

      msgs::Physics physicsMsg;
      physicsMsg.set_type(msgs::Physics::ODE);

      // Set the step time
      physicsMsg.set_max_step_size(0.01);

      // Change gravity
      msgs::Set(physicsMsg.mutable_gravity(),
          ignition::math::Vector3d(0.01, 0, 0.1));
      physicsPub->Publish(physicsMsg);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldEdit)
}
```

![image-20201006175023533](/home/ou/.config/Typora/typora-user-images/image-20201006175023533.png)

和ros差不多，gazebo的server内估计有程序在监听这个physics话题，然后根据收到的消息来修改对应属性。

CMakeList

```cmake
cmake_minimum_required(VERSION 2.8)
project(gazebo_plugin)
set(CMAKE_CXX_STANDARD 14)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(my_worldplugin SHARED main.cpp)
target_link_libraries(my_worldplugin  ${GAZEBO_LIBRARIES})
```

### 模型插件(*)

http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin

[模型插件通过话题接受目标值](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5)

模型插件是对我们来说最重要的，我觉得。因为插件允许完全访问模型及其基础元素（链接，关节，碰撞对象）的物理属性，就可以实现控制。

需要从ModelPlugin类型中继承，这是很多库都会用到的让用户定制自己类型的用法。

<img src="/home/ou/.config/Typora/typora-user-images/image-20201006122407994.png" alt="image-20201006122407994" style="zoom:53%;" />

需要重写Load Init 和Reset。

```c++
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
	   //直接设置模型位姿的线速度
//       this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
       //或者通过两个轮子关节旋转来前进
        auto jleft = this->model->GetJoint("link_0_JOINT_0");
        jleft->SetVelocity(0,0.1);
        auto jright = this->model->GetJoint("link_0_JOINT_1");
        jright->SetVelocity(0,0.1);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
```

由于设定的```      this->model = _parent;```把本插件的父元素作为模型，所以模型插件必须**<u>在模型的子标签内指定</u>**。

```xml
<model name="box">
     ........
      <plugin name="model_push" filename="libmodel_push.so"/>
</model>       
```



**<u>Tips</u>**：[写代码不知道怎么用api可以查看](https://osrf-distributions.s3.amazonaws.com/gazebo/api/6.1.0/namespacegazebo.html)

* 获取节点和部件

  ```c++
  auto link1 = this->model->GetLink("link1");  //获取节点
  int count = this->model->GetJointCount(); //获取部件数量
  auto j0=this->model->GetJoints()[1]; //获取所有部件并按顺序放入。
  
  
  ```

* 设置节点和部件属性

  ```c++
  auto jleft = this->model->GetJoint("link_0_JOINT_0");
  	jleft->SetVelocity(0,0.1);
  auto jright = this->model->GetJoint("link_0_JOINT_1");
  	jright->SetVelocity(0,0.1);
  	
  //获取节点后可以设置其大部分属性。
  link1->SetAngularVel()；
      link1->SetTorque(); 
  ```

* pid控制器

  ```c++
  this->pid = common::PID(0.1, 0, 0); //初始化pid
  this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid); //将pid用在关节上，GetScopedName是带名称空间后的名字
  this->model->GetJointController()->SetVelocityTarget(  this->joint->GetScopedName(), 10.0);  //设置pid目标值
  ```

* 事件event

  ```
  
  //事件类型可以从Events.hh中查看，也可以跳转以下这句话查看其他的事件消息
   event::Events::ConnectWorldUpdateBegin;
  ```



对于c++的function和bind 忘记了可以看[bind/function](https://blog.csdn.net/p942005405/article/details/84760715?utm_medium=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param)   [bind+this](https://blog.csdn.net/shujianlove0/article/details/84564311)



### 传感器插件

传感器插件是插入在传感器中的代码段，比如可以把仿真中的camera通过传感器插件转发到ROS的topic中然后用ros那边的固定架构代码实现算法。这样当真正插入摄像头的时候，只需要更改或甚至不需要更改话题名，而其他代码都不需要改动。

```c++
#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class ContactPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: ContactPlugin();

    /// \brief Destructor.
    public: virtual ~ContactPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;
  };
}
#endif
```

```c++
#include "ContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

ContactPlugin::ContactPlugin() : SensorPlugin(){}
ContactPlugin::~ContactPlugin(){}

void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =  std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    std::cout << "Collision between[" << contacts.contact(i).collision1()
              << "] and [" << contacts.contact(i).collision2() << "]\n";

    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      std::cout << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << "\n";
      std::cout << "   Normal:"
                << contacts.contact(i).normal(j).x() << " "
                << contacts.contact(i).normal(j).y() << " "
                << contacts.contact(i).normal(j).z() << "\n";
      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
    }
  }
}
```







### 常用插件

重力补偿插件

```xml
  <plugin name="gravity_compensation" filename="libGravityCompensationPlugin.so">
    <uri>model://r2_description</uri>
  </plugin>
```

---

ContainPlugin:世界插件、可以触发事件当机器人进入某个区域

http://gazebosim.org/tutorials?tut=contain_plugin&cat=plugins

```xml
<world name="default">
    <plugin name='ContainPlugin' filename='libContainPlugin.so'>
          <enabled>true</enabled>
          <entity>unit_sphere::only_link</entity>
          <namespace>contain_example</namespace>
          <pose>0 -3 0.5 0 0 0</pose>
          <geometry>
            <box>
              <size>4 4 1</size>
            </box>
          </geometry>
        </plugin>
</world>
```

---

led插件

http://gazebosim.org/tutorials?tut=led_plugin&cat=plugins







### 系统插件

http://gazebosim.org/tutorials?tut=system_plugin&cat=write_plugin







## 客户端

http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5

```c++
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

/
int main(int _argc, char **_argv)
{
  // Load gazebo as a client
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::setupClient(_argc, _argv);
#else
  gazebo::client::setup(_argc, _argv);
#endif

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to the  velodyne topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Vector3d>("~/my_velodyne/vel_cmd");

  // Wait for a subscriber to connect to this publisher
  pub->WaitForConnection();

  // Create a a vector3 message
  gazebo::msgs::Vector3d msg;

  // Set the velocity in the x-component
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::msgs::Set(&msg, gazebo::math::Vector3(std::atof(_argv[1]), 0, 0));
#else
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(_argv[1]), 0, 0));
#endif

  // Send the message
  pub->Publish(msg);

  // Make sure to shut everything down.
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::shutdown();
#else
  gazebo::client::shutdown();
#endif
}
```

## 事件

好多事件

比如event::Events::ConnectWorldUpdateBegin。

区域占据event

```xml
plugin中
  <region>
    <name>region1</name>
    <volume>
      <min>0 0 0</min>
      <max>0 1 1</max>
    </volume>
  </region>

  <event>
    <name>region1_event</name>
    <type>occupied</type>
    <region>region1</region>
    <topic>~/elevator</topic>
    <msg_data>0</msg_data>
  </event>
```





# ROS-Gazebo：[源码](https://github.com/ros-simulation/gazebo_ros_pkgs)

[教程](http://gazebosim.org/tutorials?cat=connect_ros)：gazebo_ros_pkg 这组ros软件包将gazebo的接口用ros包装起来了。只需要用ros接口就可以实现gazebo仿真的控制。有了这些，就不需要了解gazebo插件编程时候哪些属性怎么获取、怎么设置的问题了。只需要启动这个包，然后通过熟悉的ros话题、服务来查询和设置属性

将gazebo和ros结合起来，就可以用相同的接口来控制仿真和实物了。

不同版本的ros和gazebo组合为：kinetic-gazebo7、melodic/Dashing/eloquent-gazebo9。但是一般都不管他，下载ros自动关联版本gazebo绑定安装了。

---

<img src="https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/ros_overview/figs/775px-Gazebo_ros_api.png" alt="img" style="zoom:100%;" />



本包启动 rosrun gazebo_ros gazebo  会启动gazebo节点并运行gazebo服务器和客户端。如上图叫订阅和发布一些话题，通过话题接受ROS接口数据，然后转发和gazebo对接。还会运行一些服务比如`~/spawn_urdf_model  ~/spawn_sdf_model` 来提供加载服务。 且自带一些常用的传感器插件让gazebo中传感器获取的数据发布到ros中然后用ros包处理，比如gazebo中深度相机的插件`gazebo_ros_depth_camera`



常用可运行节点

- `rosrun gazebo_ros gazebo` launch both the Gazebo server and GUI.
- `rosrun gazebo_ros gzclient` launch the Gazebo GUI.
- `rosrun gazebo_ros gzserver` launch the Gazebo server.



也可以从launch启动

* `roslaunch gazebo_ros empty_world.launch`
* `roslaunch gazebo_ros range_world.launch` 对emtpy_world.launch进行传参调用，可以模仿这个文件(修改对应参数为想要的值[参数意义](http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros))

启动gazebo_ros 中的gazebo节点后(还不运行其他模型在gazebo中)就会发布和订阅以下话题、并产生一些服务。

这些数据如 /gazebo/link_states 用的数据类型就是本包自定义的[gazebo_msgs/LinkStates]<img src="/home/ou/.config/Typora/typora-user-images/image-20201007213535338.png" alt="image-20201007213535338" style="zoom:67%;" />

```bash
--------------------------------------------------------------------------------
Node [/gazebo]
Publications: 
 * /clock [rosgraph_msgs/Clock]
 * /gazebo/link_states [gazebo_msgs/LinkStates]
 * /gazebo/model_states [gazebo_msgs/ModelStates]
 * /gazebo/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /gazebo/parameter_updates [dynamic_reconfigure/Config]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /gazebo/set_link_state [unknown type]
 * /gazebo/set_model_state [unknown type]

Services: 
 * /gazebo/apply_body_wrench
 * /gazebo/apply_joint_effort
 * /gazebo/clear_body_wrenches
 * /gazebo/clear_joint_forces
 * /gazebo/delete_light
 * /gazebo/delete_model
 * /gazebo/get_joint_properties
 * /gazebo/get_light_properties
 * /gazebo/get_link_properties
 * /gazebo/get_link_state
 * /gazebo/get_loggers
 * /gazebo/get_model_properties
 * /gazebo/get_model_state
 * /gazebo/get_physics_properties
 * /gazebo/get_world_properties
 * /gazebo/pause_physics
 * /gazebo/reset_simulation
 * /gazebo/reset_world
 * /gazebo/set_joint_properties
 * /gazebo/set_light_properties
 * /gazebo/set_link_properties
 * /gazebo/set_link_state
 * /gazebo/set_logger_level
 * /gazebo/set_model_configuration
 * /gazebo/set_model_state
 * /gazebo/set_parameters
 * /gazebo/set_physics_properties
 * /gazebo/spawn_sdf_model
 * /gazebo/spawn_urdf_model
 * /gazebo/unpause_physics


contacting node http://ou-pc:39291/ ...
Pid: 21099
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (60617 - 127.0.0.1:59682) [35]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo
    * direction: outbound
    * transport: INTRAPROCESS
 * topic: /clock
    * to: /gazebo (http://ou-pc:39291/)
    * direction: inbound
    * transport: INTRAPROCESS

```

### 解析原理

但是我有一点没想明白，就是如果添加了带gazebo_ros插件的模型比如gazebo_ros_camera 深度相机后。发布消息的并不是这个模型上插件内的一个节点，而依旧是/gazebo发布图像信息。为何？难道gazebo_ros和插件是直接通过TCP啥的通信的？[gazebo的通信机制，任何程序可以TCP和GAZEBO通信](http://gazebosim.org/tutorials?tut=topics_subscribed&cat=transport)

[这个可以源码中找到答案](https://github.com/ros-simulation/gazebo_ros_pkgs)

仔细看以下几个源码就能了解如gazebo_ros包是如何运作的+

```
/gazebo_ros_pkgs-kinetic-devel/gazebo_plugins/include/gazebo_plugins/gazebo_ros_camera.h   及其源码
/gazebo_ros_pkgs-kinetic-devel/gazebo_plugins/include/gazebo_plugins/gazebo_ros_camera_utils.h 及其源码
/gazebo_ros_pkgs-kinetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp 及其头文件 
```



看完源码后发现<img src="/home/ou/.config/Typora/typora-user-images/image-20201007234603649.png" alt="image-20201007234603649" style="zoom:67%;" />gazebo_ros_api_plugin其实是一个gazebo系统插件其伴随着gzserver启动而加载。然后在这个gazebo_ros_api_plugin内部有一个叫gazebo的ros节点、一大堆服务、一大堆话题订阅和发布，一起构成了gazebo和ros之间的api。

而插件和gazebo节点说是用TCP通信的也没错...因为ROS消息机制也是基于TCP的。其实所有插件或说说有gazebo_ros_api共用一个节点叫gazebo节点

<img src="/home/ou/.config/Typora/typora-user-images/image-20201007235752042.png" alt="image-20201007235752042" style="zoom:67%;" />所有信息都是通过他接受、发送、请求、响应的。各个传感器插件比如相机插件只是接受gazebo消息、然后通过这个ros节点gazebo转发出来。

这个截图是gazebo_ros_camera_plugin.so内 onnewframe中的一部分，OnNewFrame 是由 gazebo sensor <update_rate>触发的。

<img src="/home/ou/.config/Typora/typora-user-images/image-20201007235836726.png" alt="image-20201007235836726" style="zoom:80%;" />





### 自己写带gazebo的ros包

[启动ros_gazebo的方法](http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros)

描述机器人模型的放在./xxx_description 而node/launch/world 则放在./xxx_gazebo中

```
../catkin_ws/src
    /MYROBOT_description
        package.xml
        CMakeLists.txt
        /urdf
            MYROBOT.urdf
        /meshes
            mesh1.dae
        /materials
        /cad
    /MYROBOT_gazebo
        /launch
            MYROBOT.launch
        /worlds
            MYROBOT.world
        /models
            world_object1.dae
       /materials
        /plugins
```

#### launch文件

在gazebo_ros结合使用的时候，可以用world插入模型，然后在launch 中用gazebo_ros 启动world。也可以直接调用ros服务插入sdf描述文件。

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

这里用的world文件其实可以随便来一个，然后进入gazebo后用gui来创建。然后保存后移动到本软件包路径下。

##### launch文件加载URDF机器人

* 通过ros service 调用生成办法，这样就可以在ros包路径下放机器人的描述文件了(使得ros软件包完整，且只需要.sdf文件不需要完整的gazebo模型)。

  * 使用spawn_model 客户端(也是用rosrun的因为rosrun是运行可执行文件)文件向`~/spawn_sdf/urdf_model` 发送请求来加载模型。

    ```rosrun gazebo_ros spawn_model -file `rospack find MYROBOT_description`/urdf/MYROBOT.urdf -urdf -x 0 -y 0 -z 1 -model MYROBOT```

    使用 rosrun gazebo_ros spawn_model -h` 查看以上各个参数的意思,这些-h 啥的参数都是main程序入口传参，只是他解析后打印给你看的提示而已。

  * 使用roslaunch 启动ros客户端发送请求

    `<node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-file $(find baxter_description)/urdf/baxter.urdf -urdf -z 1 -model baxter" />`

  * 如果是xacro文件

    ```xml
    <!-- Convert an xacro and put on parameter server -->
    <param name="robot_description" command="$(find xacro)/xacro  $(find pr2_description)/robots/pr2.urdf.xacro" />
    
    <!-- Spawn a robot into Gazebo -->
    <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model pr2" />
    ```

    或者先`rosrun xacro xacro `rospack find rrbot_description`/urdf/rrbot.xacro >> `rospack find rrbot_description`/urdf/rrbot.xml`转换成urdf在加载

  * 也可以![img](https://img-blog.csdnimg.cn/2020091615224612.png)，

    然后用一下节点来发布TF

    ```xml
       <!-- 靠读取到参数服务器中的描述文件内容，来加载模型  -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
        args="-urdf -model rrbot -param robot_description"/>
    
    
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
        <param name="use_gui" value="TRUE"/>
      </node>
    
      <!-- Combine joint values in TF format-->
      <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>
    
      <!-- Show in Rviz   -->
      <node name="rviz" pkg="rviz" type="rviz" args="-d $(find rrbot_description)/launch/rrbot.rviz"/>
    ```

    

* 通过自定义launch包含gazebo_ros的launch文件，然后指定世界文件，在世界文件中包含想生成的机器人模型，缺点是描述文件必须放在gazebo模型库中，或者需要指定环境变量为本ros包。而且还需要创建完整的gazebo模型包，包括.config文件。不过都是gui搭建模型的，工作量没差。

##### CMakeList和XML

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(YOURROBOT_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)
find_package(gazebo REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

add_library(...) 

catkin_package(
    DEPENDS
      gazebo_ros
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)
```

```xml
<build_depend>gazebo_ros</build_depend>
<run_depend>gazebo_ros</run_depend>
```

### 带ROS的GAZEBO插件  

[自带的gazebo_ros插件使用教程](http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros)

[带ros的gazebo插件如何编写](http://gazebosim.org/tutorials?tut=ros_plugins&cat=connect_ros)

创建软件包

```
cd ~/catkin_ws/src
catkin_create_pkg gazebo_tutorials gazebo_ros roscpp
```

编写源码

```c++
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    ROS_INFO("Hello World!");
  }

};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
```

CMakeLists

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(gazebo_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

add_library(${PROJECT_NAME} src/simple_world_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
```

package.xml：终端中输入`rospack plugins --attrib="gazebo_media_path"` 可以查看是否正确设置,[参考最后几行](http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros)

```xml
添加
<gazebo_ros plugin_path="${prefix}/../../lib" gazebo_media_path="${prefix}" />
```



然后编译

```bash
cd ~/catkin_ws
catkin_make
```

创建世界文件

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

    <!-- reference to your plugin -->
    <plugin name="gazebo_tutorials" filename="libgazebo_tutorials.so"/>
  </world>
</sdf>
```

创建启动程序

```xml
<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find gazebo_tutorials)/worlds/hello.world"/>
    <!-- more default parameters can be changed here -->
  </include>
</launch>
```

运行

`roslaunch gazebo_tutorials hello.launch`









#### ROS深度相机集成

[和ros连接](http://gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros)，并且可以在rviz中观察到深度图像。

gazebo_ros中有gazebo的相机插件<a name ="ROS _GAZEBO_Camera插件"></a>。因为gazebo和ros是独立的两个工程，gazebo是没有ros相关插件的。所以我们需要写一个新的相机，然后添加ros_gazebo插件到新的相机中，来让gazebo相机发布点云信息和图像在ros话题上。



首先需要一个模型，可以用有外观的gazebo自带的模型Kinetic，可以在gazebo中官方在线仓库中下载也可以点击[下载](http://github.com/osrf/gazebo_tutorials/raw/master/ros_depth_camera/files/kinect.zip)。然后移动到`~/.gazebo/models`中找到他或自行解压到此路径下。修改.config内的名字(Gazebo中显示)和.sdf中模型的名字(url引用模型时的名字)



打开.sdf文件，在<sensor>中的</camera>后插入

```xml
        <plugin name="camera_plugin" filename="libgazebo_ros_openni_kinect.so">
          <baseline>0.2</baseline>
          <alwaysOn>true</alwaysOn>
          <!-- Keep this zero, update_rate in the parent <sensor> tag
            will control the frame rate. -->
          <updateRate>0.0</updateRate>
          <cameraName>camera_ir</cameraName>
          <imageTopicName>/camera/color/image_raw</imageTopicName>
          <cameraInfoTopicName>/camera/color/camera_info</cameraInfoTopicName>
          <depthImageTopicName>/camera/depth/image_raw</depthImageTopicName>
          <depthImageCameraInfoTopicName>/camera/depth/camera_info</depthImageCameraInfoTopicName>
          <pointCloudTopicName>/camera/depth/points</pointCloudTopicName>
          <frameName>camera_link</frameName>
          <pointCloudCutoff>0.5</pointCloudCutoff>
          <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
          <distortionK1>0</distortionK1>
          <distortionK2>0</distortionK2>
          <distortionK3>0</distortionK3>
          <distortionT1>0</distortionT1>
          <distortionT2>0</distortionT2>
          <CxPrime>0</CxPrime>
          <Cx>0</Cx>
          <Cy>0</Cy>
          <focalLength>0</focalLength>
          <hackBaseline>0</hackBaseline>
        </plugin>
```

注：本插件updateRate被置0后会以和sensor的更新率同频率发布信息，而设置低于snesor更新率后会丢弃一定数据后按指定频率发布。话题名是按真实kinetic插入后发布的话题名字取的。

​		保存后启动roslaunch gazebo_ros empty_world.wolrd 然后插入kinetic_ros。在终端输入`rostopic list`会发现有数据发布

<img src="/home/ou/.config/Typora/typora-user-images/image-20201007123421860.png" alt="image-20201007123421860" style="zoom:80%;" />









### [urdf在gazebo中使用](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros)



URDF是ROS使用的机器人描述文件。而GAZEBO用的是SDF。

在GAZEBO中用URDF需要添加一些额外的`tags`而不必从头写URDF。 又由于URDF没法定义闭链结构、语法缺陷没法定义非机器人的东西、无法定义摩擦力等。



如果URDF要在GAZEBO中使用，必须：

* 每个link都配置好正确的inertia属性。
* 添加一些插件、关节限制或阻尼、颜色格式。



**<u>只需要</u>**在.urdf的边上同时创建一个.gazebo文件。然后在里面从指定urdf文件中的一些gazebo补充信息即可。

```xml
<robot>
  ... robot description ...
  <link name="sensor_link">
    ... link description ...
  </link>

  <gazebo reference="sensor_link">
    <sensor type="camera" name="camera1">
      ... sensor parameters ...
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        ... plugin parameters ..
      </plugin>
    </sensor>
  </gazebo>
</robot>
```







### GAZEBO_ROS_CONTROL包

1. [ros_controllers](http://wiki.ros.org/ros_controllers?distro=noetic),可以看到所有的控制器类型。
2. [ros_control](http://wiki.ros.org/ros_control),查看所有接口类型。ROS控制将硬件接口与上述ROS控制器之一结合使用，以向硬件发送和接收命令。
3. 如何用cpp控制gazebo关节https://www.theconstructsim.com/gazebo-qa-007-set-velocity-joint-gazebo-cpp/

#### 用ros控制器控制gazebo仿真(关节)

[起始和用话题直接控制仿真是差不多的，只不过这里是自带pid帮我们做好了，而且可以设置速度，但是通过/gazebo/set_model_state 话题只能设置部件、模型的位姿和速度(twist)，且不能设置关节的，而下文介绍的是专门设置关节的]()

这是ROS中控制器接口的新标准。是ros包和gazebo没啥关系。

仿真、硬件、控制器、信号传输关系如下。

[网友解释这个图](https://blog.csdn.net/wubaobao1993/article/details/81054570)

![img](https://github.com/osrf/gazebo_tutorials/raw/master/ros_control/Gazebo_ros_transmission.png)



---

`sudo apt-get install ros-xxxx-gazebo-ros-control`

`sudo apt-get install ros-melodic-effort-controllers`

`sudo apt-get install ros-melodic-joint-state-controller`

第一步：对URDF中每个想要控制的关节都加用ransmission标签。且joint和actuator都要加入hardwareInterface属性。

```xml
<transmission name="simple_trans">
  <type>transmission_interface/SimpleTransmission</type> - the type of transmission. Currently only "transmission_interface/SimpleTransmission" is implemented. (feel free to add more)
  <joint name="foo_joint">  - the name must correspond to a joint else where in your URDF
    <hardwareInterface>EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="foo_motor">
    <mechanicalReduction>50</mechanicalReduction>
    <hardwareInterface>EffortJointInterface</hardwareInterface>- within the <actuator> and <joint> tags, this tells the gazebo_ros_control plugin what hardware interface to load (position, velocity or effort interfaces). Currently only effort interfaces are implemented. (feel free to add more)
  </actuator>
</transmission>
```

---

第二步：添加gazebo 插件该插件会解析urdf文件中的transmission标签，然后加载相应的接口和控制器管理器。

```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/MYROBOT</robotNamespace>
  </plugin>
</gazebo>
此外
The gazebo_ros_control <plugin> 标签还有其他可选属性
<robotNamespace>: The ROS namespace to be used for this instance of the plugin, defaults to robot name in URDF/SDF
<controlPeriod>: The period of the controller update (in seconds), defaults to Gazebo's period
<robotParam>: The location of the robot_description (URDF) on the parameter server, defaults to '/robot_description'
<robotSimType>: The pluginlib name of a custom robot sim interface to be used (see below for more details), defaults to 'DefaultRobotHWSim'。这个就是图中simulation模块gazebo中的那个 defalutRobotHWsim，也可以替换成custom plugins。
    默认的插件提供一下三种ros_control接口
hardware_interface::JointStateInterface
hardware_interface::EffortJointInterface
hardware_interface::VelocityJointInterface - not fully implemented
    
    
```

或者自定义gazebo_ros_control 子类

gazebo_ros_control Gazebo插件还提供了一个基于pluginlib的接口，用于在Gazebo和ros_control之间实现自定义接口，以模拟更复杂的机制（非线性弹簧，连杆等）。自定义gazebo和ros_control之间接口插件必须继承于gazebo_ros_control::RobotHWSim(这个类实现了模拟的ros_control hardware_interface::RobotHW)，RobotHWSim提供API级访问，以在Gazebo仿真器中读取和命令关节属性。

然后在<robotSimType>标签中写明：以下使用默认的SimType，默认参数与不写这个标签内容时效果一样。

```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/MYROBOT</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  </plugin>
</gazebo>
```

**<u>TIPS</u>**：

<robotSimType>: The pluginlib name of a custom robot sim interface to be used (see below for more details), defaults to 'DefaultRobotHWSim'。

'DefaultRobotHWSim'这个就是图中simulation模块gazebo中的那个 defalutRobotHWsim，也可以替换成custom plugins。
默认的仿真类型(DefaultRobotHWSim 或不写)提供一下三种hardwareInterface

* hardware_interface::JointStateInterface
* hardware_interface::EffortJointInterface
* hardware_interface::VelocityJointInterface - not fully implemented

这个接口将ros_controld(ros程序，也就是ros_control_manager即图中下方的黄色框)接受和发布的ros信息(joint状态，joint command控制指令)然后在gazebo插件gazebo_ros_control中实现订阅和发布对应话题，然后解析ros信息，发布ros信息，转换成gazebo中对关节的指令。来实现ros_control_manager 接受用户指令，使得gazebo仿真程序受到控制	



---

第三步：创建参数文件和launch文件来启动与gazebo交互的ros_control 控制器

```bash
cd ~/catkin_ws
catkin_create_pkg MYROBOT_control controller_manager joint_state_controller robot_state_publisher
cd MYROBOT_control
mkdir config
mkdir launch
```

参数文件 xxx.yaml

```yaml
#effort_controllers/JointEffortController
#effort_controllers/JointVelocityController
#effort_controllers/JointPositionController
rrbot(名称空间，即第二步插件中robotNamespace定义的):
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50  

  # Position Controllers ---------------------------------------
  joint1_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint1
    pid: {p: 100.0, i: 0.01, d: 10.0}
  joint2_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint2
    pid: {p: 100.0, i: 0.01, d: 10.0}
```

启动文件 xxx.launch

controller_spawner节点通过运行python脚本启动两个关节位置控制器，该脚本对ros_control控制器管理器进行服务调用。调用服务通知控制器管理器您想要哪个控制器。它还加载了第三个控制器，该控制器使用hardware_interfaces发布所有关节的关节状态，并在/ joint_states上发布主题。然后robot_state_publisher订阅接收关节状态后按TF格式转发。

```xml
<launch>

  <!--加载节点控制器的配置到参数服务器，控制器节点会读取这个参数值 -->
  <rosparam file="$(find rrbot_control)/config/rrbot_control.yaml" command="load"/>

  <!-- load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" ns="/rrbot" args="joint1_position_controller joint2_position_controller joint_state_controller"/>

  <!-- 将关节状态以TF形式发布以便在rviz等软件中使用-->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
    respawn="false" output="screen">
    <remap from="/joint_states" to="/rrbot/joint_states" />
  </node>

</launch>
```

以上是launch方法启动的，当然也可以手动呼叫服务器

```bash
rosservice call /rrbot/controller_manager/load_controller "name: 'joint1_position_controller'"
rosservice call /rrbot/controller_manager/load_controller "name: 'joint2_position_controller'"
```

启动控制器

```bash
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: ['joint1_position_controller','joint2_position_controller'], stop_controllers: [], strictness: 2}"
```

停止控制器

```bash
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['joint1_position_controller','joint2_position_controller'], strictness: 2}"
```

---

第四步：发布指令

`rostopic			 pub		 -1 			/rrbot/joint2_position_controller/command 			std_msgs/Float64 "data: 1.0"`

实际值`/rrbot/joint1_position_controller/state/process_value`



#### 学会如何用rqt

`rosrun rqt_gui rqt_gui`

![image-20201007203802870](/home/ou/.config/Typora/typora-user-images/image-20201007203802870.png)

然后可以添加rqt插件，比如 [dynamic reconfigure](http://ros.org/wiki/dynamic_reconfigure) 插件。然后就可以拖动的方式来发布数据。

![image-20201007203318157](/home/ou/.config/Typora/typora-user-images/image-20201007203318157.png)

保存配置![image-20201007204152318](/home/ou/.config/Typora/typora-user-images/image-20201007204152318.png)选择export，然后保存到指定地点。

将以下加入到launch文件中可以启动rqt_gui 并按照之前保存的文件启动。

`  <node name="rrbot_rqt" pkg="rqt_gui" type="rqt_gui" respawn="false"output="screen" args="--perspective-file $(find rrbot_control)/launch/rrbot_rqt.perspective"/>`





### [gazebo和ros通信](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i6)

将讲述如何在gazebo中加入ros的通信机制，就像在添加gazebo通信机制一样。

在插件中：

插件成员添加

```c++
/// \brief A node use for ROS transport
private: std::unique_ptr<ros::NodeHandle> rosNode;

/// \brief A ROS subscriber
private: ros::Subscriber rosSub;

/// \brief A ROS callbackqueue that helps process messages
private: ros::CallbackQueue rosQueue;

/// \brief A thread the keeps running the rosQueue
private: std::thread rosQueueThread;
```

load函数添加

```c++
// Initialize ros, if it has not already bee initialized.
if (!ros::isInitialized())
{
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_client",
      ros::init_options::NoSigintHandler);
}

// Create our ROS node. This acts in a similar manner to
// the Gazebo node
this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

// Create a named topic, and subscribe to it.
ros::SubscribeOptions so =
  ros::SubscribeOptions::create<std_msgs::Float32>(
      "/" + this->model->GetName() + "/vel_cmd",
      1,
      boost::bind(&VelodynePlugin::OnRosMsg, this, _1),
      ros::VoidPtr(), &this->rosQueue);
this->rosSub = this->rosNode->subscribe(so);

// Spin up the queue helper thread.
this->rosQueueThread =
  std::thread(std::bind(&VelodynePlugin::QueueThread, this));
```

其他函数实现

```c++
/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
{
  this->SetVelocity(_msg->data);
}

/// \brief ROS helper function that processes messages
private: void QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
```



Cmakelist

```cmake
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

target_link_libraries(velodyne_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
```



#### 	GAZEBO与ROS通信二

---



<img src="/home/ou/.config/Typora/typora-user-images/image-20201007211141284.png" alt="image-20201007211141284" style="zoom:50%;" />

* gazebo_ros_api_plugin：与gzserver一起启动

  是一个在gazebo_ros包中，**<u>会初始化一个叫gazebo的节点</u>**。它将ROS回调调度程序（消息传递）与Gazebo的内部调度程序集成在一起，以提供以下所述的ROS接口。该ROS API使用户可以在ROS上操纵仿真环境的属性，以及在环境中生成和检测模型的状态。

![image-20201007210721628](/home/ou/.config/Typora/typora-user-images/image-20201007210721628.png)

* gazebo_ros_path_plugin:和gzserver和gzclient一起启动

  gazebo_ros软件包中提供了一个名为gazebo_ros_paths_plugin的辅助插件，该插件仅允许Gazebo查找ROS资源，即解析ROS软件包路径名。

  ![image-20201007211114977](/home/ou/.config/Typora/typora-user-images/image-20201007211114977.png)

  ---

  参数

* ros参数：/use_sim_time 

  如果应通过/ use_sim_time参数使用仿真时间，则Gazebo使用ROS参数服务器通知其他应用程序，尤其是Rviz。启动gazebo_ros时，凉亭应该自动将其设置为true。如果要让gazebo_ros发布到ROS / clock主题以便为ROS系统提供模拟同步时间，**<u>则/ use_sim_time为true</u>**。

  ---

  gazebo节点发布的话题

`/clock` : `rosgraph_msgs/Clock` - Publish simulation time, to be used with `/use_sim_time` parameter.

`~/link_states` : `gazebo_msgs/LinkStates` - Publishes states of all the links in simulation.

`~/model_states` : `gazebo_msgs/ModelStates` - Publishes states of all the models in simulation.

我们查看model_state话题

```bash
---
name: [ground_plane, kinect_ros]
pose: 
  - 
    position: 
      x: 0.0
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  - 
    position: 
      x: -0.0286297649495
      y: 0.0806988058894
      z: 0.0360033276129
    orientation: 
      x: -1.11909422574e-05
      y: 3.27293754037e-06
      z: 3.2519569361e-07
      w: 0.999999999932
twist: 
  - 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  - 
    linear: 
      x: -9.83114834272e-06
      y: -9.85760777408e-05
      z: -0.000387800233985
    angular: 
      x: 0.00273790497671
      y: -0.000273198532206
      z: 3.6332569112e-08
---

```

---

#### 通过ros服务设置gazebo(仿真、世界、关节、部件、模型)

gazebo节点提供的服务：创建、销毁模型

`~/spawn_urdf_model` : `gazebo_msgs/SpawnModel` - Use this service to spawn a Universal Robotic Description Format (URDF)

`~/spawn_sdf_model` : `gazebo_msgs/SpawnModel` - Use this service to spawn a model written in Gazebo Simulation Description Format (SDF)

`~/delete_model` : `gazebo_msgs/DeleteModel` - This service allows the user to delete a model from simulation.

删除模型`rosservice call gazebo/delete_model '{model_name: rrbot1}'`





此外还不只有以下：

```bash
~/set_link_properties` : `gazebo_msgs/SetLinkProperties
~/set_physics_properties` : `gazebo_msgs/SetPhysicsProperties
~/set_model_state` : `gazebo_msgs/SetModelState
~/set_model_configuration` : `gazebo_msgs/SetModelConfiguration` - This service allows the user to set model joint positions without invoking dynamics.
~/set_joint_properties` : gazebo_msgs/SetJointProperties
~/set_link_state` : `gazebo_msgs/SetLinkState
~/set_link_state` : `gazebo_msgs/LinkState
~/set_model_state` : `gazebo_msgs/ModelState
```

发现不但topic(右)而且service(左)都有/gazebo/set_model/link_state ，此外还有joint关节的属性设置。

![image-20201007224103443](/home/ou/.config/Typora/typora-user-images/image-20201007224103443.png)

所以服务是可以设置很多属性的，比如更改模型位姿和广义速度(twist)：

```bash
rosservice call /gazebo/set_model_state '{model_state: { model_name: coke_can, pose: { position: { x: 0.3, y: 0.2 ,z: 0 }, orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
```



还可以用来获取模型中的个元素的属性：

`~/get_model_properties` : `gazebo_msgs/GetModelProperties`- This service returns the properties of a model in simulation.

`~/get_model_state` : `gazebo_msgs/GetModelState` - This service returns the states of a model in simulation.

`~/get_world_properties` : `gazebo_msgs/GetWorldProperties` - This service returns the properties of the simulation world.

`~/get_joint_properties` : `gazebo_msgs/GetJointProperties` - This service returns the properties of a joint in simulation.

`~/get_link_properties` : `gazebo_msgs/GetLinkProperties` - This service returns the properties of a link in simulation.

`~/get_link_state` : `gazebo_msgs/GetLinkState` - This service returns the states of a link in simulation.

`~/get_physics_properties` : `gazebo_msgs/GetPhysicsProperties` - This service returns the properties of the physics engine used in simulation.

`~/link_states` : `gazebo_msgs/LinkStates` - Publish complete link states in world frame

`~/model_states` : `gazebo_msgs/ModelStates` - Publish complete model states in world frame

例如可以获取模型的位姿和速度

```bash
rosservice call gazebo/get_model_state '{model_name: coke_can}'
```



获取世界信息的服务有

```
rosservice call gazebo/get_world_properties
```



用来力控的服务有：可以设置关节或模型上的广义力(力螺旋——wrench)

`~/apply_body_wrench` : `gazebo_msgs/ApplyBodyWrench` - Apply wrench to a body in simulation. All active wrenches applied to the same body are cumulative.

`~/apply_joint_effort` : `gazebo_msgs/ApplyJointEffort` - Apply effort to a joint in simulation. All active efforts applied to the same joint are cumulative.

`~/clear_joint_forces` : `gazebo_msgs/JointRequest` - Clear applied efforts to a joint.

`~/clear_body_wrenches` : `gazebo_msgs/ClearBodyWrenches` - Clear applied wrench to a body.

例如这样调用：

```bash
插入一个可乐罐模型：rosrun gazebo_ros spawn_model -database coke_can -gazebo -model coke_can -y 1
设置物理世界没重力：rosservice call /gazebo/set_physics_properties "
                                                    time_step: 0.001
                                                    max_update_rate: 1000.0
                                                    gravity:
                                                      x: 0.0
                                                      y: 0.0
                                                      z: 0.0
                                                    ode_config:
                                                      auto_disable_bodies: False
                                                      sor_pgs_precon_iters: 0
                                                      sor_pgs_iters: 50
                                                      sor_pgs_w: 1.3
                                                      sor_pgs_rms_error_tol: 0.0
                                                      contact_surface_layer: 0.001
                                                      contact_max_correcting_vel: 100.0
                                                      cfm: 0.0
                                                      erp: 0.2
                                                      max_contacts: 20"
//这里时间是以微妙为单位的故意味在1s时刻施加力螺旋保持1s，coke_can::link用来指定是哪个模型，或者是哪个模型下的连杆部件。
调用服务来设置施加力：rosservice call /gazebo/apply_body_wrench '{body_name: "coke_can::link" , wrench: { torque: { x: 0.01, y: 0 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }' 


调用服务来设置关节力螺旋：
rosservice call /gazebo/apply_joint_effort "joint_name: 'joint2'
                        effort: 10.0
                        start_time:
                          secs: 0
                          nsecs: 0
                        duration:
                          secs: 10
                          nsecs: 0"
清除关节上设置的力螺旋
rosservice call /gazebo/clear_joint_forces '{joint_name: joint2}'
```



控制仿真系统的服务

`~/pause_physics` : `std_srvs/Empty` - Pause physics updates.

`~/unpause_physics` : `std_srvs/Empty` - Resume physics updates.

`~/reset_simulation` : `std_srvs/Empty` - Resets the entire simulation including the time

`~/reset_world` : `std_srvs/Empty` - Resets the model's poses

```
rosservice call gazebo/pause_physics  	//暂停仿真
rosservice call gazebo/unpause_physics  //启动仿真
```







#### 用topic设置gazebo模型参数(部件和模型)

gazebo节点订阅的话题

`~/set_link_state` : `gazebo_msgs/LinkState` - Sets the state (pose/twist) of a link.

`~/set_model_state` : `gazebo_msgs/ModelState` - Sets the state (pose/twist) of a model.

只需要发布gazebo_msgs/ModelState 类型的数据

```bash
rostopic pub -r 20 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: coke_can, pose: { position: { x: 1, y: 0, z: 2 }, orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } }, twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'
```

### 例程

强烈推荐：[ETH的无人机仿真rotors](https://github.com/ethz-asl/rotors_simulator)

其中所有ros相关和gazebo的消息连接通过一个wolrd插件ros_interface_plugin实现。

---

<a name="issue"></a>

问题1：只有在默认path下才能gazebo_ros ，如果source了自己的工作空间，就无法找到gazebo_ros 中的gzclient和gzserver两个可执行文件。

解决：建立软连接`sudo ln -s /opt/ros/kinetic/lib/gazebo_ros/gzserver /opt/ros/kinetic/share/gazebo_ros/gzserverer`

 			`sudo ln -s /opt/ros/kinetic/lib/gazebo_ros/gzclient /opt/ros/kinetic/share/gazebo_ros/gzclient`

