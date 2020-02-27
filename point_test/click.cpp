#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <vector>

ros::Subscriber pose_sub;
ros::Subscriber box_sub;
ros::Publisher click_pub;

using namespace std;

double click_x = 0.0;
double click_y = 0.0;

double pose_x = 0.0;
double pose_y = 0.0;

double add_click_x = 0.0;
double add_click_y = 0.0;

void poseCallback(const geometry_msgs::PoseStamped &pose)
{
    pose_x = pose.pose.position.x;
    pose_y = pose.pose.position.y;
}

int i = 0;
int box_num = 0;

vector<double> vecTemp_x;
vector<double> vecTemp_y;
void boxCallback(const jsk_recognition_msgs::BoundingBoxArray &box)
{
    box_num = box.boxes.size();
    for(i = 0; i < box_num; i++)
    {
        vecTemp_x.push_back(box.boxes.at(i).pose.position.x);
        vecTemp_y.push_back(box.boxes.at(i).pose.position.y);
    }

    for(i = 0; i < vecTemp_y.size(); i++)
    {
        add_click_x = pose_x + vecTemp_x[i];
        add_click_y = pose_y + vecTemp_y[i];
        cout << "add_click_x: " << add_click_x << endl;
        cout << "add_click_y: " << add_click_y << endl;
        //filling the click
        geometry_msgs::PointStamped click_num;

        click_num.header.frame_id = "world";
        click_num.point.x = add_click_x;
        click_num.point.y = add_click_y;
        click_num.point.z = 0.0;
        click_pub.publish(click_num);
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "click");
    ros::NodeHandle n;

    pose_sub = n.subscribe("/current_pose", 10, poseCallback);//pose
    box_sub = n.subscribe("/detected_bounding_boxs", 10, boxCallback);//box
    click_pub = n.advertise<geometry_msgs::PointStamped>("/clicked_point", 10);//click

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
