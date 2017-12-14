
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include "normalEstimation.h"
#include "timer.h"

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int  loadPointCloud(const std::string& source_path, const PointCloud::Ptr& source);
void PointCloudToEigenMatrix(const PointCloud::Ptr& source, Eigen::Matrix<float, Eigen::Dynamic, 3>& destination);

int main()
{
	//const std::string source_path = "../dataset/cloud.pcd";
	const std::string source_path = "../dataset/boat.ply";
	//const std::string target_path = "../dataset/cloud0.pcd";
	//const std::string source_path = "../dataset/cloud20.pcd";
	//const std::string target_path = "../dataset/yuanlin18SourcePoints.xyz";
	//const std::string source_path = "../dataset/yuanlin18modelPoints.xyz";

	PointCloud::Ptr source(new PointCloud());

	if (loadPointCloud(source_path, source) < 0) { std::cerr << "ERROR: load source " << source_path << " error." << std::endl; return -1; }

	Eigen::Matrix<float, Eigen::Dynamic, 3> sourcePoints;
	sourcePoints.resize(source->points.size(), 3);
	PointCloudToEigenMatrix(source, sourcePoints);

	std::vector<int> indices;
	indices.push_back(100);
	indices.push_back(72);
	std::vector<PointNormal> result;

	timer NormalEstimationTime;
	NormalEstimation ne;
	ne.setInputCloud(sourcePoints);
	ne.setKSearch(10);
	ne.setIndices(indices);
	ne.compute(result);
	printf("NormalEstimation cost time = %f ms.\n", NormalEstimationTime.getClock());
	for (int i = 0; i < result.size(); ++i)
		printf("point[%d] = [%f, %f, %f], normal = [%f, %f, %f], curvature = %f\n", 
														result[i].index, 
														result[i].x, result[i].y, result[i].z,
														result[i].normal_x, result[i].normal_y, result[i].normal_z, result[i].curvature);
	system("pause");
	/*
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<PointT>(source, "source");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	*/
	return 0;

}

int loadPointCloud(const std::string& source_path, const PointCloud::Ptr& source)
{
	const std::string source_ext = source_path.substr(source_path.size() - 3, source_path.size());
	if (source_ext.compare("pcd") == 0)
	{
		if (pcl::io::loadPCDFile(source_path, *source) < 0)
		{
			std::cerr << "Error loading " << source_path << std::endl;
			return -1;
		}
	}
	else if (source_ext.compare("ply") == 0)
	{
		if (pcl::io::loadPLYFile(source_path, *source) < 0)
		{
			std::cerr << "Error loading " << source_path << std::endl;
			return -1;
		}
	}
	else if (source_ext.compare("xyz") == 0 || source_ext.compare("txt") == 0)
	{
		ifstream infile;
		infile.open(source_path);
		if (!infile) { cout << "failed to open " + source_path << endl; return -1; }
		float x, y, z;
		int npts = 0;
		while (infile >> x >> y >> z)
		{
			PointT point;
			point.x = x;
			point.y = y;
			point.z = z;
			//point.b = 255;
			//point.g = 255;
			//point.r = 0;
			source->points.push_back(point);
			npts++;
		}
		infile.close();
		source->height = 1;
		source->width = npts;
		source->points.resize(npts);
	}
	else
	{
		std::cerr << "we don't support this file format right now, please use ply/pcd format!" << std::endl;
		return -1;
	}
	return 0;
}

void PointCloudToEigenMatrix(const PointCloud::Ptr& source, Eigen::Matrix<float, Eigen::Dynamic, 3>& destination)
{
	for (int i = 0; i < source->points.size(); ++i)
	{
		destination(i, 0) = source->points[i].x;
		destination(i, 1) = source->points[i].y;
		destination(i, 2) = source->points[i].z;
	}
}