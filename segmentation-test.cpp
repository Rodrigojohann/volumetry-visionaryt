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

#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;
pcl::PassThrough<pcl::PointXYZ> passx;


int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile <pcl::PointXYZ> ("volumetry-background/backgroundcloud.pcd", *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
	   << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
	   << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;


	//cv::projectPoints(objectPoints, rvecR, T, K, distCoeffs, projectedPoints);


    
    //Mat image;
    //image = imread("download.jpeg",0);

    //namedWindow("Display Image", WINDOW_AUTOSIZE );
    //imshow("Display Image", image);
    //waitKey(0);
    //return 0;



 
	pcl::visualization::CloudViewer viewer ("Cluster viewer");
	viewer.showCloud(cloud_filtered);
	while (!viewer.wasStopped ())
	{
	}
	return (0);	  
  	
}
