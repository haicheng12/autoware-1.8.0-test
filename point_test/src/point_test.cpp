//Author: Haicheng Yang
//Date: 2020.2.26

//功能描述：
//1.获取多线激光雷达的点云数据
//2.对点云进行高度、长度和角度的裁剪，只使用雷达前面一定范围内的点云数据
//3.进行滤波之后，对点云进行聚类，框选出雷达前面一定范围内的障碍物

//ROS中使用PCL参考：https://blog.csdn.net/qq_41925420/article/details/90202521
//点云格式转化参考：https://blog.csdn.net/u010284636/article/details/79214841
//聚类例程参考：http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction
//聚类原理参考：https://www.cnblogs.com/jerrylead/archive/2011/04/06/2006910.html

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>

#include <iostream>

#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;

ros::Subscriber cloud_sub;//点云接收
ros::Publisher cloud_pub1;//点云发布1，裁剪后的数据
ros::Publisher cloud_pub2;//点云发布2，滤波后的数据
ros::Publisher cloud_pub3;//点云发布3，聚类后的数据

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)//点云回调
{
    /*点云裁剪*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin_point(new pcl::PointCloud<pcl::PointXYZ>);//定义需要处理的原始点云类型
    pcl::PointCloud<pcl::PointXYZ>::Ptr height_point(new pcl::PointCloud<pcl::PointXYZ>);//定义一个容器来储存处理过的点云数据，这里为截取高度的容器
    pcl::PointCloud<pcl::PointXYZ>::Ptr length_point(new pcl::PointCloud<pcl::PointXYZ>);//定义一个容器来储存处理过的点云数据，这里为截取长度的容器
    pcl::PointCloud<pcl::PointXYZ>::Ptr angle_point(new pcl::PointCloud<pcl::PointXYZ>);//定义一个容器来储存处理过的点云数据，这里为截取角度的容器
    pcl::fromROSMsg(*input, *origin_point);//ros的点云类型转化为PCL的点云类型

    //点云处理步骤一：截取高度
    for(size_t i = 0; i < origin_point->points.size(); i++)
    {
	if(origin_point->points[i].z > -1 && origin_point->points[i].z < 1)//以雷达中心点为水平面，截取高度在-1～1范围内的点云
	{
            height_point->points.push_back(origin_point->points[i]);//将处理后的点云仍到容器里面
	}
    }
    //点云处理步骤二：截取长度
    for(size_t j = 0; j < height_point->points.size(); j++)
    {
	double length = hypot(height_point->points[j].x, height_point->points[j].y);
	if(length > 1.5 && length < 5)//以雷达中心点为原点，截取长度在0～5范围内的点云
	{
            length_point->points.push_back(height_point->points[j]);//将处理后的点云仍到容器里面
	}
    }
    //点云处理步骤三：截取角度
    for(size_t k = 0; k < length_point->points.size(); k++)
    {
	double angle = atan2(length_point->points[k].y, length_point->points[k].x);
	if(angle > -2/M_PI && angle < 2/M_PI)//以雷达中心点为原点，截取角度为-30度~30度的点云
	{
            angle_point->points.push_back(length_point->points[k]);//将处理后的点云仍到容器里面
	}
    }

    sensor_msgs::PointCloud2 output1;//定义发布的ROS点云格式，第一次发布为裁剪后的数据
    pcl::toROSMsg(*angle_point, output1);//PCL点云类型转化为ROS的点云类型

    output1.header = input->header;//赋予数据头
    cloud_pub1.publish(output1);//发布处理过后的点云数据

    /*点云滤波*/
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(output1, *cloud);//ROS转PCL数据格式sensor_msgs::PointCloud2转pcl::PCLPointCloud2

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.1, 0.1, 0.1);//滤波参数调节，数值越大点云越细腻，为了减少计算量，我们通常先进行降采样
    sor.filter (cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output2;//定义发布的ROS点云格式，第二次发布为滤波后的数据
    pcl_conversions::moveFromPCL(cloud_filtered, output2);

    output2.header = input->header;//赋予数据头
    cloud_pub2.publish(output2);//发布处理过后的点云数据

    /*欧式聚类*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);//定义原始的聚类点云

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(output2, *cluster_cloud);//ros的点云类型转化为PCL的点云类型，裁剪和滤波后的点云消息类型转化

    tree->setInputCloud (cluster_cloud);//pcl官方文档的没有去掉z坐标的值

    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.2); // 2cm 聚类搜索半径
    ec.setMinClusterSize (10); //限制一个聚类最少需要的点数目
    ec.setMaxClusterSize (2500); //限制一个聚类最多需要的点数目
    ec.setSearchMethod (tree); //设置点云的搜索机制
    ec.setInputCloud (cluster_cloud);
    ec.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中

    //迭代访问点云索引cluster_indices，直到分割出所有聚类
    int m = 0;
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
	cout << "test cluster" << endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
        for(std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cluster_cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        sensor_msgs::PointCloud2 output3;//定义发布的ROS点云格式，第三次发布为聚类后的数据
        pcl::toROSMsg(*cloud_cluster, output3);//PCL点云类型转化为ROS的点云类型

        output3.header = input->header;//赋予数据头
        cloud_pub3.publish(output3);//发布处理过后的点云数据

        m++;
    }
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "point_test");
    ros::NodeHandle nh;

    cloud_sub = nh.subscribe ("/points_raw", 10, cloud_cb);//接收的点云数据
    cloud_pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/output_cut", 10);//发布的点云数据1，裁剪后的数据
    cloud_pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/output_filter", 10);//发布的点云数据2，滤波后的数据
    cloud_pub3 = nh.advertise<sensor_msgs::PointCloud2> ("/output_cluster", 10);//发布的点云数据3，聚类后的数据

    ros::spin();
}
