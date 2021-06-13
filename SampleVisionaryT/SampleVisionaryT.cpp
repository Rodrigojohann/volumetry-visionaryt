#include <stdio.h>
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

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
size_t original_size;
pcl::PassThrough<pcl::PointXYZ> passx;
pcl::ConvexHull<pcl::PointXYZ> chull;
std::vector<pcl::Vertices> polygons;
pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
double volume;
double volumecm;
std::vector<PointXYZ> pointCloud;
boost::shared_ptr<VisionaryTData> pDataHandler;


void calculatevolume(std::vector<PointXYZ> inputcloud)
{
	cloud->points.resize (inputcloud.size());

	for(size_t i=0;i<cloud->points.size();++i)
	{
		cloud->points[i].x = inputcloud[i].x;
		cloud->points[i].y = inputcloud[i].y;
		cloud->points[i].z = inputcloud[i].z;
	}

	original_size = cloud->size();
	printf("original size: %d \n", original_size);

	passx.setInputCloud (cloud);
	passx.setFilterFieldName ("x");
	passx.setFilterLimits (-5000, 5000);
	passx.filter (*cloud_filtered);

	chull.setInputCloud(cloud_filtered);
	chull.setDimension(3);
	chull.setComputeAreaVolume(true);
	chull.reconstruct(*surface_hull, polygons);

	volume = chull.getTotalVolume();
	volumecm = volume*1000000;
	printf("volume: %f m³\n\n", volume);
	printf("volume: %f cm³\n\n", volumecm);
}

bool runStreamingDemo(char* ipAddress, unsigned short port)
{
	// Generate Visionary instance
	pDataHandler = boost::make_shared<VisionaryTData>();
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
	while (true)
	{
		if (dataStream.getNextFrame())
		{
			// Convert data to a point cloud
			pDataHandler->generatePointCloud(pointCloud);
			// Calculate volume
			calculatevolume(pointCloud);
		}
		if (getchar() == 'q')
		{
			break;
		}	
	}
	control.stopAcquisition();
	control.closeConnection();
	dataStream.closeConnection();
}

int main()
{
	//unsigned int timeout = 5000;
	//VisionaryAutoIPScan ipScan;
	//std::vector<VisionaryAutoIPScan::DeviceInfo> deviceList = ipScan.doScan(timeout);
	//for(auto it : deviceList)
	//{
		//printf("DT: %s \n", it.DeviceName.c_str());
		//printf("MAC Address: %s \n", it.MacAddress.c_str());
		//printf("IP Address: %s \n", it.IpAddress.c_str());
		//printf("Subnet: %s \n", it.SubNet.c_str());
		//printf("Port %s \n", it.Port.c_str());
	//}
	runStreamingDemo((char*)"192.168.15.40", 2114);
}
