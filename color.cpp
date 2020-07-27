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

ros::Publisher pub;

sensor_msgs::PointCloud2 color_filter(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){

// Object for storing the point cloud, with color information.
pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2 ());
pcl_conversions::toPCL(*cloud_msg, *cloud);

// kd-tree object for searches.
pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
kdtree->setInputCloud(cloud);

// Color-based region growing clustering object.
pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
clustering.setInputCloud(cloud);
clustering.setSearchMethod(kdtree);

// Here, the minimum cluster size affects also the postprocessing step:
// clusters smaller than this will be merged with their neighbors.
clustering.setMinClusterSize(100);

// Set the distance threshold, to know which points will be considered neighbors.
clustering.setDistanceThreshold(10);

// Color threshold for comparing the RGB color of two points.
clustering.setPointColorThreshold(6);

// Region color threshold for the postprocessing step: clusters with colors
// within the threshold will be merged in one.
clustering.setRegionColorThreshold(5);

std::vector <pcl::PointIndices> clusters;
clustering.extract(clusters);

// For every cluster...
int currentClusterNum = 1;

// ...add all its points to a new cloud...
pcl::PCLPointCloud2::Ptr cluster(new pcl::PCLPointCloud2 ());

pcl::PCLPointCloud<pcl::PointXYZRGB>::Ptr filtered_xyzrgb_cloud (new pcl::PCLPointCloud<pcl::PointXYZRGB>);

for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
{

	for (std::vector<int>::const_iterator point = i->indices.begin(); point != i- >indices.end(); point++) 
{
	cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;
}

	if (cluster->points.size() <= 0)
		break;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;

		pcl::fromPCLPointCloud2(*cluster,*filtered_xyzrgb_cloud);
		sensor_msgs::PointCloud2 filtered_msg;

		pcl::toROSMsg(*filtered_xyzrgb_cloud, filtered_msg);
		filtered_msg.header.frame_id = cloud_msg->header.frame_id;
		
currentClusterNum++;

}

/*pcl::fromPCLPointCloud2(*cluster,filtered_xyzrgb_cloud);
sensor_msgs::PointCloud2 filtered_msg;

pcl::toROSMsg(*filtered_xyzrgb_cloud, filtered_msg);
filtered_msg.header.frame_id = cloud_msg->header.frame_id; */

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
