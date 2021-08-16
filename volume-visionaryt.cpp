#include <algorithm>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <termio.h>
#include <unistd.h>
#include <iostream>
#include <tuple>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "VisionaryTData.h"
#include "VisionaryDataStream.h"
#include "VisionaryControl.h"
#include "PointXYZ.h"
#include "PointCloudPlyWriter.h"
#include "VisionaryAutoIPScan.h"
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_background (new pcl::PointCloud<pcl::PointXYZ>);
////////////////////////////////////////////////////

bool kbhit(void)
{
// var	
	int    characters_buffered;
	bool   pressed;
	struct termios original;
	struct termios term;
////
    tcgetattr(STDIN_FILENO, &original);
    memcpy(&term, &original, sizeof(term));
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    characters_buffered = 0;
    ioctl(STDIN_FILENO, FIONREAD, &characters_buffered);
    tcsetattr(STDIN_FILENO, TCSANOW, &original);
    pressed = (characters_buffered != 0);
    return pressed;
}

double findMedian(double inputarray[], int size)
{
    // First we sort the array
    std::sort(inputarray, inputarray + size);
 
    // check for even case
    if (size % 2 != 0)
        return inputarray[size/2];
 
    return ((inputarray[(size - 1) / 2] + inputarray[size / 2])/2.0);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr erasebackground(pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud)
{
//var
	std::vector<int>					newPointIdxVector;
	float 								resolution;
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputcloud (new pcl::PointCloud<pcl::PointXYZ>);
////
	resolution = 0.15;
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

	if ((inputcloud->size()) > 10)
	{
		octree.setInputCloud(cloud_background);
		octree.addPointsFromInputCloud();
		octree.switchBuffers();

		octree.setInputCloud(inputcloud);
		octree.addPointsFromInputCloud();
		octree.getPointIndicesFromNewVoxels(newPointIdxVector);
		
		outputcloud->points.resize(newPointIdxVector.size());
		
		for (size_t i = 0; i < newPointIdxVector.size(); ++i)
		{
			outputcloud->points[i].x = (*inputcloud)[newPointIdxVector[i]].x;
			outputcloud->points[i].y = (*inputcloud)[newPointIdxVector[i]].y;
			outputcloud->points[i].z = (*inputcloud)[newPointIdxVector[i]].z;
		}
	}
	else
	{
		outputcloud = inputcloud;
	}	
	return outputcloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filtercloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud)
{
// var
	pcl::PassThrough<pcl::PointXYZ> 			  pass_x;
	pcl::PassThrough<pcl::PointXYZ> 			  pass_y;
	pcl::PassThrough<pcl::PointXYZ> 			  pass_z;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	pcl::PointCloud<pcl::PointXYZ>::Ptr			  outputcloud (new pcl::PointCloud<pcl::PointXYZ>);
////
	pass_x.setInputCloud(inputcloud);
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits(-0.5, 0.5);
	pass_x.filter(*outputcloud);
	
	pass_y.setInputCloud(outputcloud);
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits(-0.4, 0.4);
	pass_y.filter(*outputcloud);
	
	pass_z.setInputCloud(outputcloud);
	pass_z.setFilterFieldName("z");
	pass_z.setFilterLimits(0, 2.2);
	pass_z.filter(*outputcloud);
	
	if (outputcloud->size() > 10)
	{
		sor.setInputCloud(outputcloud);
		sor.setMeanK(5);
		sor.setStddevMulThresh(3.5);
		sor.filter(*outputcloud);
	}
	return outputcloud;
}

void calculatevolume(std::vector<PointXYZ> inputcloud)
{
// var
	pcl::PointCloud<pcl::PointXYZ>::Ptr     		  cloud_raw          (new pcl::PointCloud<pcl::PointXYZ>);
	
   	pcl::VoxelGrid<pcl::PointXYZ> 		    		  sor;
	pcl::PointCloud<pcl::PointXYZ>::Ptr     		  cloud_filtered     (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr     		  cloud_nobackground (new pcl::PointCloud<pcl::PointXYZ>);
	size_t                               			  cloud_size;
	pcl::PointXYZ                        		      minPt, maxPt;
   	double 									  		  max_z;
   	pcl::search::Search<pcl::PointXYZ>::Ptr 		  tree (new pcl::search::KdTree<pcl::PointXYZ>);
   	pcl::PointCloud <pcl::Normal>::Ptr      		  normals (new pcl::PointCloud <pcl::Normal>);
   	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
   	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal>    reg;
   	pcl::IndicesPtr 								  indices (new std::vector <int>);
   	std::vector <pcl::PointIndices> 				  clusters;
   	pcl::PointCloud<pcl::PointXYZ>::Ptr     		  segmented_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	double 									  		  median_z;   	
	pcl::PassThrough<pcl::PointXYZ> 				  passz;
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> 	  feature_extractor;
	pcl::PointXYZ 									  min_point_OBB;
	pcl::PointXYZ									  max_point_OBB;
	pcl::PointXYZ 									  position_OBB;
	Eigen::Matrix3f 								  rotational_matrix_OBB;
////
	cloud_raw->points.resize(inputcloud.size());
	
	for(size_t i=0; i<cloud_raw->points.size(); ++i)
	{
		cloud_raw->points[i].x = inputcloud[i].x;
		cloud_raw->points[i].y = inputcloud[i].y;
		cloud_raw->points[i].z = inputcloud[i].z;
	}

	//sor.setInputCloud (cloud_raw);
	//sor.setLeafSize (0.01f, 0.01f, 0.01f);
	//sor.filter (*cloud_raw);
	
 	cloud_filtered     = filtercloud(cloud_raw);
	cloud_nobackground = erasebackground(cloud_filtered);
	cloud_size         = cloud_nobackground->size();
	
	if (cloud_size > 10)
	{
		pcl::getMinMax3D(*cloud_nobackground, minPt, maxPt);
		max_z = maxPt.z;
		
		normal_estimator.setSearchMethod (tree);
		normal_estimator.setInputCloud (cloud_filtered);
		normal_estimator.setKSearch (50);
		normal_estimator.compute (*normals);
		
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (cloud_filtered);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (-0.35, 0.65);
		pass.filter (*indices);
		
		reg.setMinClusterSize (500);
		reg.setMaxClusterSize (1000000);
		reg.setSearchMethod (tree);
		reg.setNumberOfNeighbours (50);
		reg.setInputCloud (cloud_filtered);
		reg.setIndices (indices);
		reg.setInputNormals (normals);
		reg.setSmoothnessThreshold (5.0 / 180.0*M_PI);
		reg.setCurvatureThreshold (50.0);

		reg.extract (clusters);		
		printf("-------------\n\n");
		for (int number=0; number<clusters.size(); ++number)
		{
			segmented_cloud->points.resize(clusters[number].indices.size());
			double z_array[segmented_cloud->size()];
			for(size_t i=0; i<clusters[number].indices.size(); ++i)
			{
				segmented_cloud->points[i].x = (*cloud_filtered)[clusters[number].indices[i]].x;
				segmented_cloud->points[i].y = (*cloud_filtered)[clusters[number].indices[i]].y;
				segmented_cloud->points[i].z = (*cloud_filtered)[clusters[number].indices[i]].z;
				z_array[i] = segmented_cloud->points[i].z;
			}
			median_z = findMedian(z_array, clusters[number].indices.size());
			
			pcl::getMinMax3D(*segmented_cloud, minPt, maxPt);

			passz.setInputCloud(segmented_cloud);
			passz.setFilterFieldName ("z");
			passz.setFilterLimits ((minPt.z-0.1), (minPt.z+0.1));
			passz.filter(*segmented_cloud);

			feature_extractor.setInputCloud(segmented_cloud);
			feature_extractor.compute();

			feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);		
			
			
			printf("Box %d:  \n\n", number);
			printf("x: %f cm \n", (max_point_OBB.x - min_point_OBB.x));
			printf("y: %f cn \n", (max_point_OBB.y - min_point_OBB.y));
			printf("z: %f cm \n\n", (max_z - median_z));
		}
		printf("----------\n");
	}
}

void runStreamingDemo(char* ipAddress, unsigned short port)
{
// var
	int 							  counter;
	std::vector<PointXYZ>			  pointCloud;
	boost::shared_ptr<VisionaryTData> pDataHandler;
	double 							  volumearray[10], Xarray[10], Yarray[10], Zarray[10];
	pcl::VoxelGrid<pcl::PointXYZ>	  sor;
////
	// Generate Visionary instance
	pDataHandler = boost::make_shared<VisionaryTData>();
	VisionaryDataStream dataStream(pDataHandler, inet_addr(ipAddress), htons(port));
	VisionaryControl control(inet_addr(ipAddress), htons(2112));
	
	pcl::io::loadPCDFile<pcl::PointXYZ> ("volumetry-background/backgroundcloud.pcd", *cloud_background);
	sor.setInputCloud (cloud_background);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_background);
	
	//-----------------------------------------------
	// Connect to devices data stream 
	if (!dataStream.openConnection())
	{
		printf("Failed to open data stream connection to device.\n");
	}
	//-----------------------------------------------
	// Connect to devices control channel
	if (!control.openConnection())
	{
		printf("Failed to open control connection to device.\n");
	}
	control.stopAcquisition();
	control.startAcquisition();
	
	while (!kbhit())
	{
		if (dataStream.getNextFrame())
		{
			// Convert data to a point cloud
			pDataHandler->generatePointCloud(pointCloud);
		}
		// Calculate volume
		calculatevolume(pointCloud); 
	}
	control.stopAcquisition();
	control.closeConnection();
	dataStream.closeConnection();
}

int main()
{
	runStreamingDemo((char*)"192.168.140.34", 2114);
}
