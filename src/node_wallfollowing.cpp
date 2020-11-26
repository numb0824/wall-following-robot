#include "node_wallfollowing.h"
#include "math.h"
/*
 geometry_msgs::Twist     ***sensor_msgs::LaserScan
 Vector3  linear                      *** Header header
            float64 x                       *** float32 angle_min //扫描开始的角度值
            float64 y                       ***float32 angle_max //扫描结束的角度值
            float64z                        ***float32 angle_increment//两次扫描之间增加的角度
Vector3  angular                   ***float32 time_increment//两次激光发射的时间间隔
            float64 x                       ***float32 scan_time//有歧义，暂时不说???
            float64 y                       ***float32 range_min//最近距离
            float64 z                       ***float32 range_max//最远距离
                                                    *** float32[] ranges 障碍物距离数组
                                                    ***flaot32[] intensities//光强 
*/                                                  
#define PI 3.1415926
NodeWallFollowing::NodeWallFollowing(ros::Publisher pub, double wallDist, 
                                                                                    double maxSp, int dir, double pr, double di, 
                                                                                    double an)
{
    WallDistance = wallDist;
         maxSpeed = maxSp;
            direction = dir;
                            P = pr;
                            D = di;
          AngleCoef = an; 
                             e = 0;
            AngleMin = 0;
     pubMessage = pub;
}

NodeWallFollowing::~NodeWallFollowing()
{   
}

//publisher
void    NodeWallFollowing::publishMessage()
{
    //读取Twist 消息类型，准备发送数据
    geometry_msgs::Twist msg;

    //PD控制器，控制Twist中的角度值
   // msg.angular.z = 0;
    msg.angular.z = direction*(P*e + D*diffE) + AngleCoef * (AngleMin - PI * direction/2);
    
    //速度逻辑判断
    if (DistFront < WallDistance)
    {
        msg.linear.x = 0 ;
    }
    else if (DistFront < WallDistance * 2)
    {
        msg.linear.x = maxSpeed * 0.5;
        //msg.linear.x = maxSpeed * 0;
    }
    else if (fabs(AngleMin) > 1.75)
    {
        msg.linear.x = 0.4*maxSpeed;
        //msg.linear.x = maxSpeed * 0;
    }
    else
    {
        msg.linear.x = maxSpeed;
        //msg.linear.x = maxSpeed * 0;
    }
    
    //发布消息
    pubMessage.publish(msg);
}

//subscriber
void NodeWallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int size = msg->ranges.size();//一次扫描有多少个障碍物距离长度

    //相当于对雷达数据做一次采样，每次只取一半的范围，这样是为了避免突然去墙的另一侧
    int minIndex = size * (direction + 1) / 4;
    int maxIndex = size * (direction +3) / 4;
//使用循环查找范围向量中的最小值。通过选择大于0.01米的值来过滤测量误差。
    for (int i = minIndex; i < maxIndex; i++)
    {
        if(msg -> ranges[i] < msg->ranges[minIndex] && msg->ranges[i] > 0.01)
        {
            minIndex = i;
        }
    }

    AngleMin = (minIndex - size / 2) * msg -> angle_increment;

    double distMin;
    distMin = msg->ranges[minIndex];
    DistFront = msg->ranges[size/2];
    diffE = (distMin - WallDistance) - e ;//PD控制器的导数
    e = distMin - WallDistance;
    ROS_INFO("min: angle=%f, distance=%f, front=%f", AngleMin, distMin, DistFront);
    publishMessage();
}