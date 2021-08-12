#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>


using namespace cv;
pcl::PassThrough<pcl::PointXYZ> passx;


int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile <pcl::PointXYZ> ("volumetry-background/backgroundcloud4.pcd", *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputcloud (new pcl::PointCloud<pcl::PointXYZ>);

	//std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
	   //<< " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered);

	pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (cloud_filtered);
	normal_estimator.setKSearch (50);
	normal_estimator.compute (*normals);

	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-0.35, 0.65);
	pass.filter (*indices);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize (500);
	reg.setMaxClusterSize (1000000);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (50);
	reg.setInputCloud (cloud_filtered);
	reg.setIndices (indices);
	reg.setInputNormals (normals);
	reg.setSmoothnessThreshold (5.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold (50.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	//std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
	//std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
	//std::cout << "These are the indices of the points of the initial" <<
	//std::endl << "cloud that belong to the first cluster:" << std::endl;
	int counter = 0;
	//while (counter < clusters[0].indices.size ())
	//{
	//std::cout << clusters[0].indices[counter] << ", ";
	//counter++;
	//if (counter % 10 == 0)
	  //std::cout << std::endl;
	//}
	//std::cout << std::endl;
	
	std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
	std::cout << "First cluster has " << clusters[1].indices.size () << " points." << std::endl;
	std::cout << "These are the indices of the points of the initial" <<
	std::endl << "cloud that belong to the first cluster:" << std::endl;
	
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	pcl::visualization::CloudViewer viewer ("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped ())
	{
	}

	return (0);



	//std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
	   //<< " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;


	//cv::projectPoints(objectPoints, rvecR, T, K, distCoeffs, projectedPoints);


    
    //Mat image;
    //image = imread("download.jpeg",0);

    //namedWindow("Display Image", WINDOW_AUTOSIZE );
    //imshow("Display Image", image);
    //waitKey(0);
    //return 0;



 
	//pcl::visualization::CloudViewer viewer ("Cluster viewer");
	//viewer.showCloud(outputcloud);
	//while (!viewer.wasStopped ())
	//{
	//}
	//return (0);	  
  	
}
