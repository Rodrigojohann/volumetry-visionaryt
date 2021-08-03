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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
std::vector<PointXYZ> pointCloud;
boost::shared_ptr<VisionaryTData> pDataHandler;
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;


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
	passx.setFilterLimits (-0.230, 0.25);
	passx.filter (*cloud_filtered);
	
	passy.setInputCloud (cloud_filtered);
	passy.setFilterFieldName ("y");
	passy.setFilterLimits (-0.15, 0.28);
	passy.filter (*cloud_filtered);
	
	passz.setInputCloud (cloud_filtered);
	passz.setFilterFieldName ("z");
	passz.setFilterLimits (-5000, 0.76);
	passz.filter (*cloud_filtered);
	
//	sor.setInputCloud (cloud_filtered);
//	sor.setMeanK (3);
//	sor.setStddevMulThresh (3.0);
//	sor.filter (*cloud_filtered);
	

	// ---------------------- Visualizer -------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	viewer->setBackgroundColor(0, 0, 0);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 0, 0);
	viewer->addPointCloud(cloud_filtered, color_handler, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud");
	while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
		}
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

	if (dataStream.getNextFrame())
		{
			// Convert data to a point cloud
			pDataHandler->generatePointCloud(pointCloud);
			// Calculate volume
			calculatevolume(pointCloud);
		}
	control.stopAcquisition();
	control.closeConnection();
	dataStream.closeConnection();
}


int main()
{
	runStreamingDemo((char*)"192.168.140.29", 2114);
}
