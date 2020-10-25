//
// Created by ou on 2020/10/19.
//

#ifndef TRICYCLE_TRICYCLE_CTL_H
#define TRICYCLE_TRICYCLE_CTL_H
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/GetJointProperties.h"
#include <string>
class PID{
private:
    double P=0,I=0,D=0,limit=0,ref=0;
    double e=0,e1=0,e2=0,output=0;
public:
    double target=0;
public:
    PID();
    void SetPID(double _P,double _I,double _D,double _limit);
    double RunOnce(double ref);
};

class TricycleRosCtl{
public:
    TricycleRosCtl();
    void SetPose();
    void CtlLoop();
private:
    double PoseTarget[3],PoseRef[3];

private:
    ros::NodeHandle n;
    ros::Publisher pubs[3*2];
    ros::ServiceClient client;
    gazebo_msgs::GetJointProperties srv[3];

private:
    PID PosePID[3],WheelPID[3];
    double joint_velocity_fb[3];
    void ReadJointVeloFb();
    void TricycleMathModel();
private:

private:
    std::string joint_names;
};


#endif //TRICYCLE_TRICYCLE_CTL_H
