//
// Created by ou on 2020/10/19.
//
#include "tricycle_ctl.h"
PID::PID(){
    ;
}
void PID::SetPID(double _P,double _I,double _D,double _limit) {
    P=_P;
    I=_I;
    D=_D;
    limit=_limit;
}
double PID::RunOnce(double ref) {
    this->ref=ref;
    e= target-ref;
    double delta=P*(e-e1)+I*e+D*(e-2*e1+e2);
    e2=e1;
    e1=e;
    output+=delta;
    if(output>limit)
        return output=limit;
    else if(output<-limit)
        return output=-limit;
    else return output;
}


TricycleRosCtl::TricycleRosCtl() {
    pubs[0] = n.advertise<std_msgs::Float64>("/tricycle/joint0_velocity_controller/command", 100);
    pubs[1] = n.advertise<std_msgs::Float64>("/tricycle/joint1_velocity_controller/command", 100);
    pubs[2] = n.advertise<std_msgs::Float64>("/tricycle/joint2_velocity_controller/command", 100);
    client = n.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
    srv[0].request.joint_name="base_wheel_0_joint";
    srv[1].request.joint_name="base_wheel_1_joint";
    srv[2].request.joint_name="base_wheel_2_joint";
}
void TricycleRosCtl::ReadJointVeloFb()
{
    for(int i=0;i<3;i++)
    {
        if(client.call(srv[i]))
        {
            joint_velocity_fb[i] = srv[i].response.rate.data()[0];
        }
    }
}
void TricycleRosCtl::TricycleMathModel()
{
    ;
}
void TricycleRosCtl::CtlLoop(){
    for (int i = 0; i < 3; ++i) {
        std_msgs::Float64 f64;
        f64.data = WheelPID[i].RunOnce(joint_velocity_fb[i]);
        pubs[i].publish(f64);
    }
}