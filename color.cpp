#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

ros::Publisher pub;

sensor_msgs::PointCloud2 color_filter(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){

// Object for storing the point cloud, with color information.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::fromROSMsg(*cloud_msg, *cloud);

std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;
// kd-tree object for searches.
pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
kdtree->setInputCloud(cloud);
std::cerr << "kd-tree object created." << std::endl;
// Color-based region growing clustering object.
pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
std::cerr << "Color-based region growing clustering object created." << std::endl;
clustering.setInputCloud(cloud);
std::cerr << "clustering input cloud set to cloud." << std::endl;
clustering.setSearchMethod(kdtree);
std::cerr << "clustering search method set to kdtree." << std::endl;

// Here, the minimum cluster size affects also the postprocessing step:
// clusters smaller than this will be merged with their neighbors.
clustering.setMinClusterSize(10);
std::cerr << "clustering minimum cluster size set to 10 points." << std::endl;

// Set the distance threshold, to know which points will be considered neighbors.
clustering.setDistanceThreshold(5);
std::cerr << "clustering distance threshold set to 5." << std::endl;

// Color threshold for comparing the RGB color of two points.
clustering.setPointColorThreshold(3);
std::cerr << "clustering point color threshold set to 3." << std::endl;

// Region color threshold for the postprocessing step: clusters with colors
// within the threshold will be merged in one.
clustering.setRegionColorThreshold(3);
std::cerr << "clustering region clolor threshold set to 3." << std::endl;

std::vector <pcl::PointIndices> clusters;
std::cerr << "point indices object clusters created." << std::endl;

clustering.extract(clusters);
std::cerr << "clustering extracted into clusters." << std::endl;

// For every cluster...
int currentClusterNum = 1;
std::cerr << "variable currentClusterNum set to 1." << std::endl;

// ...add all its points to a new cloud...
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
std::cerr << "point cloud cluster created." << std::endl;

for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
{

	for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++) 
{
	cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;
	std::cerr << "cluster size evaluated." << std::endl;
}

	if (cluster->points.size() <= 0)
		break;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;

	
currentClusterNum++;
std::cerr << "currentClusterNum now has a value of " << currentClusterNum << "." << std::endl;
}
		sensor_msgs::PointCloud2 filtered_msg;
		std::cerr << "sensor msg filtered_msg created." << std::endl;

		pcl::toROSMsg(*cluster, filtered_msg);
		std::cerr << "conversion of cluster into ROSMsg filtered_msg completed." << std::endl;
		filtered_msg.header.frame_id = cloud_msg->header.frame_id;

std::cerr << "returning filtered_msg." << std::endl;
return filtered_msg; 
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {

sensor_msgs::PointCloud2 filtered_msg = color_filter(cloud_msg);

pub.publish(filtered_msg); 

return;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"listener");

    ros::NodeHandle nh;

    pub=nh.advertise<sensor_msgs::PointCloud2>("/cloud_color_filtered", 1000);

    ros::Subscriber sub=nh.subscribe("/camera/depth_registered/points", 1000, callback);

    ros::spin();

    return 0;
}
