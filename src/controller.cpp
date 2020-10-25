#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/GetJointProperties.h"
#include <sstream>
#include "tricycle_ctl.h"
static long count;
int main(int argc, char **argv)
{
    //第三个参数(基本名字即不能带'/')为节点名，除非在launch中的<node ... name="xxx"/>重新指定
    //rosrun pack <exename> 由CMakeList指定
    //rosnode list 中出现的是 <nodename> 由第三个参数
    ros::init(argc, argv, "talker");
    //作为和ROS通信的主要途径：在第一个Handle被完全初始化本节点；在最后销毁一个的Handle来完全关闭节点
    TricycleRosCtl ctl;
    ros::Rate loop_rate(1000);
    //todo:多线程读取反馈。通过teleop节点获取遥控指令。
    while (ros::ok())
    {




        //ROS_INFO("loop:%d",count++);
        //注意spinOnce()区别:spinOnce只读取当前消息队列第一个元素调用callback后返回;spin直接堵塞，一有消息进队就调用回调，否则原地死循环。
        ros::spinOnce();
        //根据ros::Rate loop_rate(10)指定的频率，自动记录上次sleep到本次sleep之间的时间，然后休眠直到超过对应频率的延时。
        loop_rate.sleep();//或直接用 ros::Duration(0.5).sleep(); //second
    }
    return 0;
}
