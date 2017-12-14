#include "normalEstimation.h"
#include <iostream>

NormalEstimation::NormalEstimation() {
	K = 5;
	viewpoint.x = 0;
	viewpoint.y = 0;
	viewpoint.z = 0;
	indices.clear();
}

void NormalEstimation::setInputCloud(const Eigen::Matrix<float, Eigen::Dynamic, 3>& source_) {
	source = source_;
}
void NormalEstimation::findNeighbors(const Eigen::Matrix<float, 1, 3>& queryPoint, Eigen::Matrix<float, Eigen::Dynamic, 3>& neighbors) {
	
	neighbors.resize(K, 3);
	std::vector<size_t> indexResult(K);               //存储index
	std::vector<float> squaredistResult(K);           //存储和查询点的平方距离。
	nanoflann::KNNResultSet<float> resultSet(K);
	resultSet.init(&indexResult[0], &squaredistResult[0]);

	float query_pt[3] = {queryPoint(0, 0), queryPoint(0, 1), queryPoint(0, 2) };
	treeIndex->index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
	
	for (size_t i = 0; i < K; i++)
	{
		neighbors(i, 0) = source(indexResult[i], 0);
		neighbors(i, 1) = source(indexResult[i], 1);
		neighbors(i, 2) = source(indexResult[i], 2);
	}
	//std::cout << "knnSearch(nn=" << K << "): \n";
	//for (size_t i = 0; i < K; i++)       //输出查询结果
	//	printf("sourcePoints[%d] = (%f, %f, %f), SquareDist = %f\n", indexResult[i], source(indexResult[i], 0), source(indexResult[i], 1), source(indexResult[i], 2), squaredistResult[i]);
}
void NormalEstimation::setKSearch(int K_) {
	K = K_;
}
void NormalEstimation::compute3DCentroid(const Eigen::Matrix<float, Eigen::Dynamic, 3>& neighbors, Eigen::Matrix<float, 1, 3>& centroid) {
	centroid = neighbors.colwise().sum() / neighbors.rows();
}
void NormalEstimation::demeanPointCloud(const Eigen::Matrix<float, Eigen::Dynamic, 3>& source_, const Eigen::Matrix<float, 1, 3>& centroid, Eigen::Matrix<float, Eigen::Dynamic, 3>& source_demean) {
	source_demean.resize(source.rows(), 3);

	for (int i = 0; i < source_.rows(); ++i)
	{
		source_demean(i, 0) = source_(i, 0) - centroid(0, 0);
		source_demean(i, 1) = source_(i, 1) - centroid(0, 1);
		source_demean(i, 2) = source_(i, 2) - centroid(0, 2);
	}
}
void NormalEstimation::computeCovarianceMatrix(const Eigen::Matrix<float, Eigen::Dynamic, 3>& neighbors, Eigen::Matrix<float, 3, 3>&  covariance_matrix) {
	
	Eigen::Matrix<float, 1, 3> centroid;
	compute3DCentroid(neighbors, centroid);
	Eigen::Matrix<float, Eigen::Dynamic, 3> neighbors_demean;
	demeanPointCloud(neighbors, centroid, neighbors_demean);

	covariance_matrix.setZero();
	for (size_t i = 0; i < neighbors.size(); ++i)
	{
		covariance_matrix(0, 0) += neighbors_demean(i, 0) * neighbors_demean(i, 0);
		covariance_matrix(0, 1) += neighbors_demean(i, 0) * neighbors_demean(i, 1);
		covariance_matrix(0, 2) += neighbors_demean(i, 0) * neighbors_demean(i, 2);
		covariance_matrix(1, 1) += neighbors_demean(i, 1) * neighbors_demean(i, 1);
		covariance_matrix(1, 2) += neighbors_demean(i, 1) * neighbors_demean(i, 2);
		covariance_matrix(2, 2) += neighbors_demean(i, 2) * neighbors_demean(i, 2);
	}
	covariance_matrix(1, 0) = covariance_matrix(0, 1);
	covariance_matrix(2, 0) = covariance_matrix(0, 2);
	covariance_matrix(2, 1) = covariance_matrix(1, 2);
}

void NormalEstimation::solvePlaneParameters(const Eigen::Matrix<float, 3, 3>& covariance_matrix, PointNormal& pn) {

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 3, 3>> eigensolver(covariance_matrix);

	pn.normal_x = eigensolver.eigenvectors()(0, 0);
	pn.normal_y = eigensolver.eigenvectors()(1, 0);
	pn.normal_z = eigensolver.eigenvectors()(2, 0);

	float eigen_sum = covariance_matrix(0, 0) + covariance_matrix(1, 1) + covariance_matrix(2, 2);
	if (eigen_sum != 0)
		pn.curvature = fabs(eigensolver.eigenvalues()(0) / eigen_sum);
	else
		pn.curvature = 0;
}

void NormalEstimation::flipNormalTowardsViewPoint(const Eigen::Matrix<float, 1, 3>& points, PointNormal& pn)
{
	// See if we need to flip any plane normals
	viewpoint.x -= points(0, 0);
	viewpoint.y -= points(0, 1);
	viewpoint.z -= points(0, 2);
	// Dot product between the (viewpoint - point) and the plane normal
	float cos_theta = (viewpoint.x * pn.normal_x + viewpoint.y * pn.normal_y + viewpoint.z * pn.normal_z);

	// Flip the plane normal
	if (cos_theta < 0)
	{
		pn.normal_x *= -1;
		pn.normal_y *= -1;
		pn.normal_z *= -1;
	}

	//printf("points[%d] = [%f, %f, %f], normal = [%f, %f, %f], curvature = %f\n", pn.index, pn.x, pn.y, pn.z, pn.normal_x, pn.normal_y, pn.normal_z, pn.curvature);
}

void NormalEstimation::setIndices(const std::vector<int>& indices_) {
	indices = indices_;
}
void NormalEstimation::setViewPoint(const Point3f& viewpoint_)
{
	viewpoint = viewpoint_;
}

void NormalEstimation::compute(std::vector<PointNormal>& pointnormal) {
	
	treeIndex = new KDTree(source, 5);
	//printf("indices size = %d\n", indices.size());
	//printf("source size = %d\n", source.size());
	pointnormal.clear();
	
	
	if (indices.size() == 0)
	{
		pointnormal.resize(source.size());
		#pragma omp parallel for
		for (int i = 0; i < source.size(); ++i)
		{
			compute(i, source.row(i), pointnormal[i]);
		}			
	}
	else
	{
		pointnormal.resize(indices.size());
		#pragma omp parallel for
		for (int i = 0; i < indices.size(); ++i)
		{
			compute(indices[i], source.row(indices[i]), pointnormal[i]);
		}
	}	
}

void NormalEstimation::compute(int index, Eigen::Matrix<float, 1, 3> points, PointNormal& pn) {
	
		pn.index = index;
		pn.x = points(0, 0);
		pn.y = points(0, 1);
		pn.z = points(0, 2);
		Eigen::Matrix<float, Eigen::Dynamic, 3> neighbors;
		neighbors.resize(K, 3);
		Eigen::Matrix<float, 3, 3> covariance_matrix;
		
		findNeighbors(points, neighbors);
		computeCovarianceMatrix(neighbors, covariance_matrix);

		solvePlaneParameters(covariance_matrix, pn);
		flipNormalTowardsViewPoint(points, pn);
}