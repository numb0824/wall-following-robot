#include "node_wallfollowing.h"

#define SUBSCRIBER_BUFFER_SIZE 1
#define PUBLISHER_BUFFER_SIZE 1
#define WALL_DISTANCE 0.5
#define MAX_SPEED 0.3

#define P 10
#define D 5
#define ANGLE_COEF 1
#define DIRECTION 1//1左，-1右
#define PUBLISHER_TOPIC "/cmd_vel"
#define SUBSCRIBER_TOPIC "/scan"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wallfollowing");
    ros::NodeHandle n;

    ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);
    NodeWallFollowing *nodeWallFollowing = new NodeWallFollowing(pubMessage, WALL_DISTANCE, MAX_SPEED, DIRECTION, P, D , 1);

    ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeWallFollowing::messageCallback, nodeWallFollowing);
    ros::spin();
    return 0;
}

