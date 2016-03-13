#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
 
#include <iostream>
 

//ROS Libraries
#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//Voxel Grid
#include <pcl/filters/voxel_grid.h>

//imported for eigen

//NARF
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/visualization/range_image_visualizer.h>

using namespace std;
using namespace ros;
using namespace pcl;

Publisher pub;

	// Object for storing the SHOT descriptors for the scene.
	pcl::PointCloud<pcl::SHOT352>::Ptr sceneDescriptors(new pcl::PointCloud<pcl::SHOT352>());
	// Object for storing the SHOT descriptors for the model.
	pcl::PointCloud<pcl::SHOT352>::Ptr modelDescriptors(new pcl::PointCloud<pcl::SHOT352>());
	// temp descriptors
	pcl::PointCloud<pcl::SHOT352>::Ptr tempDescriptors(new pcl::PointCloud<pcl::SHOT352>());

void cloud_cb( const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{ 


	sceneDescriptors = descriptors;

	if(tempDescriptors.size() != 0){
		// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<pcl::SHOT352> matching;
	matching.setInputCloud(tempDescriptors);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
 
	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < sceneDescriptors->size(); ++i)
	{
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (pcl_isfinite(sceneDescriptors->at(i).descriptor[0]))
		{
			// Find the nearest neighbor (in descriptor space)...
			int neighborCount = matching.nearestKSearch(sceneDescriptors->at(i), 1, neighbors, squaredDistances);
			// ...and add a new correspondence if the distance is less than a threshold
			// (SHOT distances are between 0 and 1, other descriptors use different metrics).
			if (neighborCount == 1 && squaredDistances[0] < 0.25f)
			{
				pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
				correspondences->push_back(correspondence);
			}
		}
	}
	std::cout << "Found " << correspondences->size() << " correspondences." << std::endl;
}
	else
	tempDescriptors = sceneDescriptors;


}



int main(int argc, char** argv)
{
  init (argc, argv, "my_pcl_tutorial");
  NodeHandle nh;
  cout << "Node nh Started"<< std::endl;
  // Create a ROS subscriber for the input point cloud

//  Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb); //for XYZRGB
  Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);   // for depth data
  cout << "input done" <<std::endl;
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  // Spin
  spin ();
}
