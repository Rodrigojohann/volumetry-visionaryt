#include <stdio.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "VisionaryTData.h"
#include "VisionaryDataStream.h"
#include "VisionaryControl.h"
#include "PointXYZ.h"
#include "PointCloudPlyWriter.h"
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/passthrough.h>

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

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PassThrough<pcl::PointXYZ> passx;
	passx.setInputCloud (cloud);
	passx.setFilterFieldName ("x");
	passx.setFilterLimits (-5000, 5000);
	passx.filter (*cloud_filtered);

	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud_filtered);
	chull.setDimension(3);
	chull.setComputeAreaVolume(true);

	std::vector<pcl::Vertices> polygons;
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
	chull.reconstruct(*surface_hull, polygons);

	double volume = chull.getTotalVolume();
	double volumecm = volume*1000000;
	printf("volume: %f m³\n\n", volume);
	printf("volume: %f cm³\n\n", volumecm);
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

	control.stopAcquisition();
	control.startAcquisition();
	for (int i = 0; i < 10; i++)
	{
		if (dataStream.getNextFrame())
		{
			// Convert data to a point cloud
			std::vector<PointXYZ> pointCloud;
			pDataHandler->generatePointCloud(pointCloud);

			// Calculate volume
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
}

int main()
{
	// Insert IP of your camera and the API port
	/// Default values:
	/// IP:        "192.168.1.10"
	/// API-port:  2114
	runStreamingDemo("192.168.15.40", 2114);
}
