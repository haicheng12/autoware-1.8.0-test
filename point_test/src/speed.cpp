#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

ros::Subscriber cmd_vel_sub;
ros::Publisher cmd_vel_pub;

using namespace std;

double vx = 0.0;
double vth = 0.0;

//filling the velocity
geometry_msgs::Twist velocity;

void cmd_velCallback(const geometry_msgs::TwistStamped &msg)
{
    vx = msg.twist.linear.x;
    vth = msg.twist.angular.z;

    velocity.linear.x = vx;
    velocity.angular.z = vth;

    cout << "vx: " << velocity.linear.x << endl;
    cout << "vth: " << velocity.angular.z << endl;

    cmd_vel_pub.publish(velocity);
    ros::spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_trans");
    ros::NodeHandle n;

    cmd_vel_sub = n.subscribe("/twist_cmd", 10, cmd_velCallback);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);//发布速度

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
