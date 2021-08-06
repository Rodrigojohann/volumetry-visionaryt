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
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>



pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered        (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_background      (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nobackground    (new pcl::PointCloud<pcl::PointXYZ>);

////////////////////////////////////////////////////

bool kbhit(void)
{
// var	
	int characters_buffered;
	bool pressed;
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

void erasebackground(pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud)
{
//var
	std::vector<int> newPointIdxVector;
	float resolution;
////
	resolution = 0.02f;

	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

	octree.setInputCloud(cloud_background);
	octree.addPointsFromInputCloud();
	octree.switchBuffers();

	octree.setInputCloud(inputcloud);
	octree.addPointsFromInputCloud();
	octree.getPointIndicesFromNewVoxels(newPointIdxVector);
	
	cloud_nobackground->points.resize(newPointIdxVector.size());
	
	for (size_t i = 0; i < newPointIdxVector.size(); ++i)
	{
		cloud_nobackground->points[i].x = (*inputcloud)[newPointIdxVector[i]].x;
		cloud_nobackground->points[i].y = (*inputcloud)[newPointIdxVector[i]].y;
		cloud_nobackground->points[i].z = (*inputcloud)[newPointIdxVector[i]].z;
	}
}

void filtercloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud)
{
// var
	pcl::PassThrough<pcl::PointXYZ> passx;
	pcl::PassThrough<pcl::PointXYZ> passy;
	pcl::PassThrough<pcl::PointXYZ> passz;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
////
	passx.setInputCloud(inputcloud);
	passx.setFilterFieldName ("x");
	passx.setFilterLimits (-0.23, 0.25);
	passx.filter (*cloud_filtered);
	
	passy.setInputCloud(cloud_filtered);
	passy.setFilterFieldName ("y");
	passy.setFilterLimits (-0.15, 0.26);
	passy.filter (*cloud_filtered);
	
	passz.setInputCloud(cloud_filtered);
	passz.setFilterFieldName ("z");
	passz.setFilterLimits (0, 0.758);
	passz.filter (*cloud_filtered);
	
	sor.setInputCloud(cloud_filtered);
	sor.setMeanK(5);
	sor.setStddevMulThresh(3.5);
	sor.filter(*cloud_filtered);
}

std::tuple<double, double, double, double> calculatevolume(std::vector<PointXYZ> inputcloud)
{
// var
   	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>);
	size_t cloud_size;
	pcl::ConvexHull<pcl::PointXYZ> chull;
	std::vector<pcl::Vertices> polygons;
	double volume, dimensionX, dimensionY, dimensionZ;	
	pcl::PointXYZ minPt, maxPt;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw 	          (new pcl::PointCloud<pcl::PointXYZ>);
////
	cloud_raw->points.resize(inputcloud.size());
	
	for(size_t i=0; i<cloud_raw->points.size(); ++i)
	{
		cloud_raw->points[i].x = inputcloud[i].x;
		cloud_raw->points[i].y = inputcloud[i].y;
		cloud_raw->points[i].z = inputcloud[i].z;
	}

	filtercloud(cloud_raw);
	erasebackground(cloud_filtered);
	cloud_size = cloud_nobackground->size();
	
	if (cloud_size > 10)
	{
	chull.setInputCloud(cloud_nobackground);
	chull.setDimension(3);
	chull.setComputeAreaVolume(true);
	chull.reconstruct(*surface_hull, polygons);
	
	volume = chull.getTotalVolume();
	pcl::getMinMax3D(*surface_hull, minPt, maxPt);
	
	dimensionX = maxPt.x - minPt.x;
	dimensionY = maxPt.y - minPt.y;
	dimensionZ = maxPt.z - minPt.z;
	}
	else
	{
	volume     = 0.0;
	dimensionX = 0.0;
	dimensionY = 0.0;
	dimensionZ = 0.0;
	}
	
	return std::make_tuple(volume, dimensionX, dimensionY, dimensionZ);
}

void runStreamingDemo(char* ipAddress, unsigned short port)
{
// var
	int counter;
	double volumemean, X_mean, Y_mean, Z_mean;	
	std::vector<PointXYZ> pointCloud;
	boost::shared_ptr<VisionaryTData> pDataHandler;
	double volume, dimensionX, dimensionY, dimensionZ;
////
	pcl::io::loadPCDFile<pcl::PointXYZ> ("volumetry-background/backgroundcloud.pcd", *cloud_background);
	// Generate Visionary instance
	pDataHandler = boost::make_shared<VisionaryTData>();
	VisionaryDataStream dataStream(pDataHandler, inet_addr(ipAddress), htons(port));
	VisionaryControl control(inet_addr(ipAddress), htons(2112));
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
	
	volumemean = 0.0;
	counter    = 0;
	while (!kbhit())
	{
		counter = counter+1; 
		if (dataStream.getNextFrame())
		{
			// Convert data to a point cloud
			pDataHandler->generatePointCloud(pointCloud);
			// Calculate volume
			std::tie(volume, dimensionX, dimensionY, dimensionZ) = calculatevolume(pointCloud); 
			volumemean = volumemean + volume;
			X_mean = X_mean + dimensionX;
			Y_mean = Y_mean + dimensionY;
			Z_mean = Z_mean + dimensionZ;			
		}

		if (counter==9)
		{
			counter    = 0;
			volumemean = volumemean/10;
			X_mean     = X_mean/10;
			Y_mean     = Y_mean/10;
			Z_mean     = Z_mean/10;
			
			printf("---------------------\n\n");
			printf("volume:\n");
			printf("%f cm³\n\n", volumemean*1000000);
			
			printf("dimensions:\n");
			printf("%f cm (x)\n", X_mean*100);
			printf("%f cm (y)\n", Y_mean*100);
			printf("%f cm (z)\n\n", Z_mean*100);
			volumemean = 0.0;			
		}
	}
	control.stopAcquisition();
	control.closeConnection();
	dataStream.closeConnection();
}


int main()
{
	runStreamingDemo((char*)"192.168.140.29", 2114);
}
