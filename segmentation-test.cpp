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
#include <pcl/features/integral_image_normal.h>

using namespace cv;
pcl::PassThrough<pcl::PointXYZ> passx;


int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile <pcl::PointXYZ> ("volumetry-background/backgroundcloud4.pcd", *cloud);
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

	//int counter = 0;
	//while (counter < clusters[1].indices.size ())
	//{
	//std::cout << clusters[1].indices[counter] << ", ";
	//counter++;
	//if (counter % 10 == 0)make
	  //std::cout << std::endl;
	//}
	//std::cout << std::endl;
	
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

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	
	//for(size_t i=0; i<clusters[0].indices.size(); ++i)
	//{
		
		//segmented_cloud->points[i].x = (*colored_cloud)[i].x;
		//segmented_cloud->points[i].y = (*colored_cloud)[i].y;
		//segmented_cloud->points[i].z = (*colored_cloud)[i].z;
	//}

	pcl::io::savePCDFile("outputcloud.pcd", *colored_cloud);

    //// estimate normals
    //pcl::PointCloud<pcl::Normal>::Ptr newnormals (new pcl::PointCloud<pcl::Normal>);

    //pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    //ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    //ne.setMaxDepthChangeFactor(0.02f);
    //ne.setNormalSmoothingSize(10.0f);
    //ne.setInputCloud(segmented_cloud);
    //ne.compute(*newnormals);

    //// visualize normals
    //pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    //viewer.setBackgroundColor (0.0, 0.0, 0.5);
    //viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(segmented_cloud, newnormals);
    
    //while (!viewer.wasStopped ())
    //{
      //viewer.spinOnce ();
    //}
    //return 0;
  	
}
