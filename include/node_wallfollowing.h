//包含必要的头文件，激光雷达的数据和机器人的速度消息
#ifndef  __WALL_FOLLOWING_H__
#define __WALL_FOLLOWING_H__

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
 

 class NodeWallFollowing
 {
 public:
    /*
        pub,                机器人topic的发布
        wallDist,      机器人离墙距离的期望值;
        maxSp,          机器人的设定最大速度    
        dir                    机器人贴墙的方向，1代表左边，-1代表右边
        pr                     PD控制器的比例常数 
        di                     PD控制器的微分常数
        an                    P控制器的角度系数
    */
     NodeWallFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, 
                                                double pr,double di,double an);
    ~NodeWallFollowing();

//publishMessage方法是用来计算控制器干预从数据在diffE, e, AngleMin变量和设置线速度，
//这取决于distFront变量。
    void publishMessage();

    //雷达消息的回调函数
    void messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    
    //要使用的变量
    double WallDistance;                        //离墙距离的期望值;
    double e;                                               //离墙距离期望值与真实值之间的差;
    double diffE;                                       //PD控制器的导数
    double maxSpeed;                          //机器人的最大速度
    double P;                                            //PD控制器的P参数
    double D;                                          //PD控制器的D参数
    double AngleCoef;                       //P控制器的系数
    double AngleMin;                        //测量到的最短距离的角度值
    double DistFront;                       //机器人测量的距离值
    int  direction;                                 //机器人的朝向，1代表左边，-1是右侧，
    ros::Publisher pubMessage;
 };

#endif