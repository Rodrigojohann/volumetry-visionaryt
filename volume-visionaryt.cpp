#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <termio.h>
#include <unistd.h>
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
pcl::PassThrough<pcl::PointXYZ> passy;
pcl::PassThrough<pcl::PointXYZ> passz;
pcl::ConvexHull<pcl::PointXYZ> chull;
std::vector<pcl::Vertices> polygons;
pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
double volume;
double volumecm;
double volumemean;
std::vector<PointXYZ> pointCloud;
boost::shared_ptr<VisionaryTData> pDataHandler;
int i;

bool kbhit(void)
{
    struct termios original;
    tcgetattr(STDIN_FILENO, &original);
    struct termios term;
    memcpy(&term, &original, sizeof(term));
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    int characters_buffered = 0;
    ioctl(STDIN_FILENO, FIONREAD, &characters_buffered);
    tcsetattr(STDIN_FILENO, TCSANOW, &original);
    bool pressed = (characters_buffered != 0);
    return pressed;
}

double calculatevolume(std::vector<PointXYZ> inputcloud)
{
	cloud->points.resize (inputcloud.size());

	for(size_t i=0;i<cloud->points.size();++i)
	{
		cloud->points[i].x = inputcloud[i].x;
		cloud->points[i].y = inputcloud[i].y;
		cloud->points[i].z = inputcloud[i].z;
	}

	original_size = cloud->size();
	//printf("original size: %d \n", original_size);

	passx.setInputCloud (cloud);
	passx.setFilterFieldName ("x");
	passx.setFilterLimits (-0.24, 0.2);
	passx.filter (*cloud_filtered);
	
	passy.setInputCloud (cloud_filtered);
	passy.setFilterFieldName ("y");
	passy.setFilterLimits (-0.220, 0.220);
	passy.filter (*cloud_filtered);
	
	passz.setInputCloud (cloud_filtered);
	passz.setFilterFieldName ("z");
	passz.setFilterLimits (0, 0.80);
	passz.filter (*cloud_filtered);

	chull.setInputCloud(cloud_filtered);
	chull.setDimension(3);
	chull.setComputeAreaVolume(true);
	chull.reconstruct(*surface_hull, polygons);

	volume = chull.getTotalVolume();
	//volumecm = volume*1000000;
	//printf("volume: %f m³\n\n", volume);
	//printf("volume: %f cm³\n\n", volumecm);
	return volume;
}

void runStreamingDemo(char* ipAddress, unsigned short port)
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
	while (!kbhit())
	{
		i = i+1; 
		if (dataStream.getNextFrame())
		{
			// Convert data to a point cloud
			pDataHandler->generatePointCloud(pointCloud);
			// Calculate volume
			volumemean = volumemean + calculatevolume(pointCloud);
		}

		if (i==9)
		{
			i = 0;
			volumemean = volumemean/10;
			printf("volume: %f m³\n\n", volumemean);
			printf("volume: %f cm³\n\n", volumemean*1000000);
			volumemean = 0.0;			
		}
	}
	control.stopAcquisition();
	control.closeConnection();
	dataStream.closeConnection();
}


int main()
{
	runStreamingDemo((char*)"192.168.15.40", 2114);
}
