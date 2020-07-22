#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include"pcl_conversions/pcl_conversions.h"
#include"pcl_ros/point_cloud.h"
#include"pcl_ros/transforms.h"

#include"pcl/io/pcd_io.h"
#include"pcl/point_types.h"
#include"pcl/point_cloud.h"

#include "math.h"
#include "vector"

#define _USE_MATH_DEFINES

using namespace std;

typedef pcl::PointXYZI PointType;
ros::Publisher pub;

void pointCallback(const sensor_msgs::PointCloud2ConstPtr &msg){
	pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);

	pcl::fromROSMsg(*msg, *cloud);

	for (size_t i=0; i<cloud->points.size(); i++){
		cloud->points[i].x = cloud->points[i].x*cos(12*M_PI/180) + cloud->points[i].z*sin(12*M_PI/180);
		cloud->points[i].y = cloud->points[i].y;
		cloud->points[i].z = -cloud->points[i].x*sin(12*M_PI/180) + cloud->points[i].z*cos(12*M_PI/180);
	}

	pub.publish(cloud);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "calibration_node");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, pointCallback);
	pub = nh.advertise<sensor_msgs::PointCloud2>("calibration_points", 1);

	ros::spin();
}
