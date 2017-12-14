#ifndef _NORMAL_ESTIMATION_H_
#define _NORMAL_ESTIMATION_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include "nanoflann.hpp"
using namespace nanoflann;

typedef KDTreeEigenMatrixAdaptor<Eigen::Matrix<float, Eigen::Dynamic, 3>, nanoflann::metric_L2_Simple> KDTree;

struct Point3f {
	float x;
	float y;
	float z;
};

struct PointNormal {
	int index;
	float x, y, z;
	float normal_x, normal_y, normal_z;
	float curvature;
	//PointNormal(int index_, float x_, float y_, float z_, float normal_x_, float normal_y_, float normal_z_, float curvature_) :
	//	index(index_), x(x_), y(y_), z(z_), normal_x(normal_x_), normal_y(normal_y_), normal_z(normal_z_), curvature(curvature_){}
};

class NormalEstimation {
public:
	NormalEstimation();
	void setInputCloud(const Eigen::Matrix<float, Eigen::Dynamic, 3>& source_);
	void setQueryPoint();
	void setIndices(const std::vector<int>& indices_);
	void findNeighbors(const Eigen::Matrix<float, 1, 3>&  queryPoint, Eigen::Matrix<float, Eigen::Dynamic, 3>& neighbors);
	void setKSearch(int K_);
	void compute3DCentroid(const Eigen::Matrix<float, Eigen::Dynamic, 3>& neighbors, Eigen::Matrix<float, 1, 3>& centroid);
	void demeanPointCloud(const Eigen::Matrix<float, Eigen::Dynamic, 3>& source_, const Eigen::Matrix<float, 1, 3>& centroid, Eigen::Matrix<float, Eigen::Dynamic, 3>& source_demean);
	void computeCovarianceMatrix(const Eigen::Matrix<float, Eigen::Dynamic, 3>& neighbors, Eigen::Matrix<float, 3, 3>&  covariance_matrix);
	void compute(std::vector<PointNormal>& pointnormal);
	void compute(int index, Eigen::Matrix<float, 1, 3> points, PointNormal& pn);
	void solvePlaneParameters(const Eigen::Matrix<float, 3, 3>& covariance_matrix, PointNormal& pn);  //solve the eigenvalues and eigenvectors of a given 3*3 covariance matrix, and estimate the least-squares plane normal and surface curvature. <features/impl/feature.hpp> 48ÐÐ
	//palne_parameters: the resultant plane parameters as :a, b, c, d (ax + by +cz +d = 0)
	//curvature: the estimated surface curvature as a measure of lambda_0 / (lambda_0 + lambda_1 + lambda_2)
	//void solvePlaneParameters(const Eigen::Matrix<double, 3, 3>& covariance_matrix, Point3f& normal, float& curvature); //<features/impl/feature.hpp> 61ÐÐ
	void flipNormalTowardsViewPoint(const Eigen::Matrix<float, 1, 3>& points, PointNormal& pn);
	void setViewPoint(const Point3f& viewpoint_);

private:
	Eigen::Matrix<float, Eigen::Dynamic, 3> source;
	int K;		//K nearest neighbors
	KDTree* treeIndex;
	Point3f viewpoint;
	std::vector<int> indices;
};

#endif //_NORMAL_ESTIMATION_H_