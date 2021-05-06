//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: November 2016
// 
// @author:  Andreas Richert
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de
// 
// Last commit: $Date: 2018-02-15 13:51:24 +0100 (Do, 15 Feb 2018) $
// Last editor: $Author: richean $
// 
// Version "$Revision: 16012 $"
//

#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "VisionaryTData.h"    // Header specific for the Time of Flight data
#include "VisionaryDataStream.h"
#include "VisionaryControl.h"
#include "PointXYZ.h"
#include "PointCloudPlyWriter.h"
#include "CoLaBCommandBuilder.h"
#include "CoLaBCommandReader.h"
#include <vector>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/io/ply_io.h>

void calculatevolume(std::vector<PointXYZ> inputcloud)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize (inputcloud.size());

	for(size_t i=0;i<cloud->points.size();++i)
	{
		cloud->points[i].x = inputcloud[i].x;
		cloud->points[i].y = inputcloud[i].y;
		cloud->points[i].z = inputcloud[i].z;
	}

	size_t original_size = cloud->size();
	printf("original size: %d \n", original_size);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PassThrough<pcl::PointXYZ> passx;
	passx.setInputCloud (cloud);
	passx.setFilterFieldName ("x");
	passx.setFilterLimits (-0.250, 0.250);
	passx.filter (*cloud_filtered_x);
	
	pcl::PassThrough<pcl::PointXYZ> passy;
	passy.setInputCloud (cloud_filtered_x);
	passy.setFilterFieldName ("y");
	passy.setFilterLimits (0, 0.819);
	passy.filter (*cloud_filtered_y);
	
	pcl::PassThrough<pcl::PointXYZ> passz;
	passz.setInputCloud (cloud_filtered_y);
	passz.setFilterFieldName ("z");
	passz.setFilterLimits (-0.810, -0.400);
	passz.filter (*cloud_filtered_z);
	size_t filtered_size = cloud_filtered_z->size();
	printf("pointcloud after filtering size: %d \n", filtered_size);

	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud_filtered_z);
	chull.setDimension(3);
	chull.setComputeAreaVolume(true);

	std::vector<pcl::Vertices> polygons;
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
	chull.reconstruct(*surface_hull, polygons);

	double volume= chull.getTotalVolume();
	printf("volume: %f \n\n", volume);
}



bool runStreamingDemo(char* ipAddress, unsigned short port)
{
	// Generate Visionary instance
	boost::shared_ptr<VisionaryTData> pDataHandler = boost::make_shared<VisionaryTData>();
	VisionaryDataStream dataStream(pDataHandler, inet_addr(ipAddress), htons(port));
	VisionaryControl control(inet_addr(ipAddress), htons(2112));
  
	//-----------------------------------------------
	// Connect to devices data stream 
	if (!dataStream.openConnection())
	{
		printf("Failed to open data stream connection to device.\n");
		return false;   // connection failed
	}

	//-----------------------------------------------
	// Connect to devices control channel
	if (!control.openConnection())
	{
		printf("Failed to open control connection to device.\n");
		return false;   // connection failed
	}

	//-----------------------------------------------
	// Stop image acquisition (works always, also when already stopped)
	control.stopAcquisition();

	//-----------------------------------------------
	// Capture a single image
	//control.stepAcquisition();
	control.startAcquisition();
	for (int i = 0; i < 10; i++)
	{
		if (dataStream.getNextFrame())
		{
			printf("Frame received through step called, frame #%d, timestamp: %I64u \n", pDataHandler->getFrameNum(), pDataHandler->getTimestampMS());

			//-----------------------------------------------
			// Convert data to a point cloud
			std::vector<PointXYZ> pointCloud;
			pDataHandler->generatePointCloud(pointCloud);

			calculatevolume(pointCloud);
			
			if (i == 9)
			{
				char* plyFilePath = "/home/rodrigo/Volumetry/volumetry-visionaryt/testcloud_original.ply";
				printf("Writing frame to %s\n", plyFilePath);
				PointCloudPlyWriter::WriteFormatPLY(plyFilePath, pointCloud, true);
				printf("Finished writing frame to %s\n", plyFilePath);
			}
		}
	}

	control.stopAcquisition();
	control.closeConnection();
	dataStream.closeConnection();
	return true;
}


int main()
{
	// Insert IP of your camera and the API port
	/// Default values:
	/// IP:        "192.168.1.10"
	/// API-port:  2114
	runStreamingDemo("192.168.15.40", 2114);
}
