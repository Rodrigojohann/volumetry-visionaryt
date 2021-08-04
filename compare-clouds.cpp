#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <termio.h>
#include <unistd.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

int main()
{
	pcl::io::loadPLYFile<pcl::PointXYZ> ("volumetry-background/backgroundcloud.ply", *cloud_original);
	pcl::io::loadPLYFile<pcl::PointXYZ> ("volumetry-background/cloud_nobackground.ply", *cloud_filtered);

	sor.setInputCloud (cloud_filtered);
	sor.setMeanK (50);
	sor.setStddevMulThresh (3.5);
	sor.filter (*cloud_filtered);
	
	
	// ---------------------- Visualizer -------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	viewer->setBackgroundColor(0, 0, 0);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler1(cloud_original, 255, 0, 0);
	viewer->addPointCloud(cloud_original, color_handler1, "sample cloud1");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler2(cloud_filtered, 0, 0, 255);
	viewer->addPointCloud(cloud_filtered, color_handler2, "sample cloud2");

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud2");
	while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
		}

}
