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

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_background (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_concat     (new pcl::PointCloud<pcl::PointXYZ>);
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

double calculate_new_mean (double data[], double meanvalue, double stdvalue)
{
//var	
	double sum     = 0.0;
	int    counter = 1;
	double mean;
////	
	for (int i = 0; i < 10; i++)
	{
		if ((data[i] <= meanvalue+stdvalue) and (data[i] >= meanvalue-stdvalue))
		{
			sum     += data[i];
			counter += 1;
		}
	}
	mean = sum/counter;
	
	return mean;	
}

double calculate_std (double data[], double mean)
{
//var	
	double sum = 0.0;
	double std;
////
	for(int i = 0; i < 10; ++i)
	{
		sum += pow(data[i] - mean, 2);
	}
	
	std = sqrt(sum/10);
	
	return std;
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

std::tuple<double, double, double, double> calculatevolume(std::vector<PointXYZ> inputcloud)
{
// var
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw          (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered     (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nobackground (new pcl::PointCloud<pcl::PointXYZ>);
   	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull       (new pcl::PointCloud<pcl::PointXYZ>);
	size_t                              cloud_size;
	pcl::ConvexHull<pcl::PointXYZ>      chull;
	std::vector<pcl::Vertices>          polygons;
	double                              volume, dimensionX, dimensionY, dimensionZ;	
	pcl::PointXYZ                       minPt, maxPt;
	double 						        cutvalue;
	pcl::PassThrough<pcl::PointXYZ>     pass_groundnoise;
////
	cutvalue = 0.15;
	cloud_raw->points.resize(inputcloud.size());
	
	for(size_t i=0; i<cloud_raw->points.size(); ++i)
	{
		cloud_raw->points[i].x = inputcloud[i].x;
		cloud_raw->points[i].y = inputcloud[i].y;
		cloud_raw->points[i].z = inputcloud[i].z;
	}

 	cloud_filtered     = filtercloud(cloud_raw);
	cloud_nobackground = erasebackground(cloud_filtered);
	cloud_size         = cloud_nobackground->size();
	
	*cloud_concat = (*cloud_concat) + (*cloud_nobackground);
	
	if (cloud_size > 10)
	{
		pcl::getMinMax3D(*cloud_nobackground, minPt, maxPt);
		
		pass_groundnoise.setInputCloud(cloud_nobackground);
		pass_groundnoise.setFilterFieldName("z");
		pass_groundnoise.setFilterLimits(0, (maxPt.z - cutvalue));
		pass_groundnoise.filter(*cloud_nobackground);		
		
		//chull.setInputCloud(cloud_nobackground);
		//chull.setDimension(3);
		//chull.setComputeAreaVolume(true);
		//chull.reconstruct(*surface_hull, polygons);
		
		//volume = chull.getTotalVolume();
		
		pcl::getMinMax3D(*cloud_nobackground, minPt, maxPt);
		
		dimensionX = maxPt.x - minPt.x;
		dimensionY = maxPt.y - minPt.y;
		dimensionZ = maxPt.z - minPt.z + cutvalue;
		
		volume = dimensionX*dimensionY*dimensionZ;
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
	int 							  counter;
	double 							  volumemean, X_mean, Y_mean, Z_mean;
	double 							  volumestd, X_std, Y_std, Z_std;	
	double 							  volumemean_new, X_mean_new, Y_mean_new, Z_mean_new;
	std::vector<PointXYZ>			  pointCloud;
	boost::shared_ptr<VisionaryTData> pDataHandler;
	double 							  volumearray[10], Xarray[10], Yarray[10], Zarray[10];
	pcl::PassThrough<pcl::PointXYZ>   pass_remove;
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
	X_mean     = 0.0;
	Y_mean     = 0.0;
	Z_mean     = 0.0;
	counter    = 0;
	while (!kbhit())
	{
		counter = counter + 1; 
		if (dataStream.getNextFrame())
		{
			// Convert data to a point cloud
			pDataHandler->generatePointCloud(pointCloud);
		}
		// Calculate volume
		std::tie(volumearray[counter], Xarray[counter], Yarray[counter], Zarray[counter]) = calculatevolume(pointCloud); 

		volumemean = volumemean + volumearray[counter];
		X_mean     = X_mean + Xarray[counter];
		Y_mean     = Y_mean + Yarray[counter];
		Z_mean     = Z_mean + Zarray[counter];
		
		if (counter==9)
		{
			counter    = 0;
			volumemean = volumemean/10;
			X_mean     = X_mean/10;
			Y_mean     = Y_mean/10;
			Z_mean     = Z_mean/10;
			
			volumestd = calculate_std(volumearray, volumemean);
			X_std	  = calculate_std(Xarray, X_mean);
			Y_std	  = calculate_std(Yarray, Y_mean);
			Z_std 	  = calculate_std(Zarray, Z_mean);
			
			volumemean_new = calculate_new_mean(volumearray, volumemean, volumestd);
			X_mean_new 	   = calculate_new_mean(Xarray, X_mean, X_std);
			Y_mean_new     = calculate_new_mean(Yarray, Y_mean, Y_std);
			Z_mean_new     = calculate_new_mean(Zarray, Z_mean, Z_std);
			
			pcl::io::savePLYFile("outputcloud.ply", *cloud_concat);
			
			pass_remove.setInputCloud(cloud_concat);
			pass_remove.setFilterFieldName("x");
			pass_remove.setFilterLimits(-5000, -4000);
			pass_remove.filter(*cloud_concat);
			
			printf("---------------------\n\n");
			printf("volume:\n");
			printf("%f cm³\n\n", volumemean_new*1000000);
			
			printf("dimensions:\n");
			printf("%f cm (x)\n", X_mean_new*100);
			printf("%f cm (y)\n", Y_mean_new*100);
			printf("%f cm (z)\n\n", Z_mean_new*100);
			volumemean = 0.0;
			X_mean 	   = 0.0;
			Y_mean     = 0.0;
			Z_mean     = 0.0;
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
