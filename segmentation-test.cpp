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
#include <pcl/segmentation/sac_segmentation.h>

#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/moment_of_inertia_estimation.h>

using namespace cv;

pcl::PointXYZ                       minPt, maxPt;


double findMedian(double a[], int n)
{
    // First we sort the array
    std::sort(a, a + n);
 
    // check for even case
    if (n % 2 != 0)
        return a[n/2];
 
    return ((a[(n - 1) / 2] + a[n / 2])/2.0);
}

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile <pcl::PointXYZ> ("volumetry-background/backgroundcloud5.pcd", *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputcloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	//std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
	   //<< " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered);

	float max_z, min_z;
	pcl::getMinMax3D(*cloud_filtered, minPt, maxPt);
	max_z = maxPt.z;
	min_z = minPt.z;

	pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (cloud_filtered);
	normal_estimator.setKSearch (50);
	normal_estimator.compute (*normals);

	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-0.35, 0.65);
	pass.filter (*indices);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize (500);
	reg.setMaxClusterSize (1000000);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (50);
	reg.setInputCloud (cloud_filtered);
	reg.setIndices (indices);
	reg.setInputNormals (normals);
	reg.setSmoothnessThreshold (5.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold (50.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	
	std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
	std::cout << "First cluster has " << clusters[1].indices.size () << " points." << std::endl;
	std::cout << "These are the indices of the points of the initial" <<
	std::endl << "cloud that belong to the first cluster:" << std::endl;
	
	//pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	//pcl::visualization::CloudViewer viewer ("Cluster viewer");
	//viewer.showCloud(colored_cloud);
	//while (!viewer.wasStopped ())
	//{
	//}

	//return (0);

	int number = 4;
	segmented_cloud->points.resize(clusters[number].indices.size());
	
	double z_array[clusters[number].indices.size()];
	for(size_t i=0; i<clusters[number].indices.size(); ++i)
	{
		
		//printf("\n\n %d", clusters[1].indices[i]);
		
		segmented_cloud->points[i].x = (*cloud_filtered)[clusters[number].indices[i]].x;
		segmented_cloud->points[i].y = (*cloud_filtered)[clusters[number].indices[i]].y;
		segmented_cloud->points[i].z = (*cloud_filtered)[clusters[number].indices[i]].z;
		z_array[i] = segmented_cloud->points[i].z;
	}

	pcl::getMinMax3D(*segmented_cloud, minPt, maxPt);
	
	pcl::PassThrough<pcl::PointXYZ> passz;
	passz.setInputCloud (segmented_cloud);
	passz.setFilterFieldName ("z");
	passz.setFilterLimits ((minPt.z-0.1), (minPt.z+0.1));
	passz.filter (*segmented_cloud);
	
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud (segmented_cloud);
	feature_extractor.compute ();
	
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;	

	feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	
	printf("x: %f \n", (max_point_OBB.x - min_point_OBB.x));
	printf("y: %f \n", (max_point_OBB.y - min_point_OBB.y));
	
	pcl::getMinMax3D(*segmented_cloud, minPt, maxPt);
	
	double median_z;
	median_z = findMedian(z_array,clusters[number].indices.size());
	printf("z: %f \n", (2.02 - median_z));
	
	
	//pcl::io::savePCDFile("outputcloud.pcd", *segmented_cloud);

    //normal_estimator.setSearchMethod (tree);
	//normal_estimator.setInputCloud (segmented_cloud);
	//normal_estimator.setKSearch (segmented_cloud->points.size());
	//normal_estimator.compute (*normals);
	
	//std::cout << normals;
	
	// ---------------------- Visualizer -------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	viewer->setBackgroundColor(0, 0, 0);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler1(cloud, 255, 0, 0);
	viewer->addPointCloud(cloud_filtered, color_handler1, "sample cloud1");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler2(cloud, 0, 0, 255);
	viewer->addPointCloud(segmented_cloud, color_handler2, "sample cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud2");
	Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
	Eigen::Quaternionf quat (rotational_matrix_OBB);
	viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
	while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
		}
  	
}
