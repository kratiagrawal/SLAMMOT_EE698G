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

//SHOT
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include "correspondence_types.hpp"

//Function Definitions
/*To calculate the descriptors of a given point cloud*/
//PointCloud<SHOT352>::Ptr descriptors_estimation(PointCloud<PointXYZ>::Ptr);
/*ROS Call back function*/
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr&);

using namespace std;
using namespace ros;
using namespace pcl;
using namespace registration;

PointCloud<PointXYZ>::Ptr scene0;
PointCloud<PointXYZ>::Ptr scene1;
int counter = 0;	

Publisher pub;

PointCloud<SHOT352>::Ptr descriptors_estimation(PointCloud<PointXYZ>::Ptr cloud_ori)
{
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
  PointCloud<SHOT352>::Ptr descriptors(new PointCloud<SHOT352>());
  

  //DownSampling
  VoxelGrid<PointXYZ> filter;
  filter.setInputCloud(cloud_ori);
  filter.setLeafSize(0.2f, 0.2f, 0.2f);
  filter.filter(*cloud);
  cout << "Total points: " << cloud_ori->size () << "; Keypoints: " << cloud->size () << endl;
  

  // Estimate the normals.
  NormalEstimationOMP<PointXYZ, Normal> normalEstimation;
  normalEstimation.setInputCloud(cloud_ori);
  normalEstimation.setKSearch(10);
  normalEstimation.compute(*normals);
 
  
  // SHOT estimation object.
  SHOTEstimationOMP<PointXYZ, Normal, SHOT352> shot;
  shot.setInputCloud(cloud);
  shot.setInputNormals(normals);
  shot.setRadiusSearch(0.02);
  shot.setSearchSurface(cloud_ori);
  shot.compute(*descriptors);

  return descriptors;
}


void feature_matching(PointCloud<SHOT352>::Ptr sceneDescriptors, PointCloud<SHOT352>::Ptr modelDescriptors)
{
	// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	KdTreeFLANN<SHOT352> matching;
	matching.setInputCloud(modelDescriptors);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	CorrespondencesPtr correspondences(new Correspondences());
 	vector<int> indices_q;
	vector<int> indices_m;
	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < sceneDescriptors->size(); ++i)
	{
		vector<int> neighbors(1);
		vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (pcl_isfinite(sceneDescriptors->at(i).descriptor[0]))
		{
			// Find the nearest neighbor (in descriptor space)...
			int neighborCount = matching.nearestKSearch(sceneDescriptors->at(i), 1, neighbors, squaredDistances);
			// ...and add a new correspondence if the distance is less than a threshold
			// (SHOT distances are between 0 and 1, other descriptors use different metrics).
			if (neighborCount == 1 && squaredDistances[0] < 0.25f)
			{
				Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
				correspondences->push_back(correspondence);
        cout << correspondence << endl;
 			}
		}
	}

	cout << "Found " << correspondences->size() << " correspondences." << endl;
	registration::getQueryIndices(*correspondences,indices_q); 
	registration::getMatchIndices(*correspondences,indices_m); 
  for (int k = 0; k < indices_m.size(); ++k)
  {
    cout << "q = " << indices_q.at(k) << " " << "m = " << indices_m.at(k) << endl;  
  }
  cout << "query: " << indices_q.size() << endl;
	cout << "match: " << indices_m.size() << endl;
  //cout << correspondences << endl;
}


void cloud_cb( const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{ 
  PointCloud<PointXYZ>::Ptr cloud_ori(new PointCloud<PointXYZ>);
  PointCloud<SHOT352>::Ptr descriptors0(new PointCloud<SHOT352>());
  PointCloud<SHOT352>::Ptr descriptors1(new PointCloud<SHOT352>());
  
  fromROSMsg(*cloud_msg, *cloud_ori);
  if(counter == 0)
  {  	
  	scene0 = cloud_ori;
  	cout << "loop entered" << endl;
  	counter++;
  }
  else if(counter == 1)
  {
  	cout << "value put in scene1" << endl;
  	scene1 = cloud_ori;
  	counter++;
  }
  else
  {
  	scene0 = scene1;
  	scene1 = cloud_ori;
  	counter++;
  }
  if(counter > 1)
  {  	
  	descriptors0 = descriptors_estimation(scene0);
  	descriptors1 = descriptors_estimation(scene1);
  	feature_matching(descriptors0, descriptors1);
	
	//cout << "before pushback" << descriptors0->size() << endl;  
	//descriptors0->operator+=(*descriptors1);
	//cout << "after pushback" << descriptors0->size() << endl;  
    // Convert to ROS data type and Publish the data
/*    sensor_msgs::PointCloud2 output;
    toROSMsg(*cloud_ori, output);  
    pub.publish(output);
*/ }

    cout << counter << " msg recieved" << endl;

}


int main(int argc, char** argv)
{
  // Create a ROS subscriber for the input point cloud
  init (argc, argv, "my_pcl_tutorial");
  NodeHandle nh;
  cout << "Node nh Started"<< endl;
  

  //Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb); //for XYZRGB
  Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);   // for depth data
  cout << "input done" << endl;
 

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  spin ();
}
