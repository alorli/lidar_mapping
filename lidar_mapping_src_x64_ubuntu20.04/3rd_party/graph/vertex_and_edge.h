
#ifndef VERTEX_AND_EDGE_H_
#define VERTEX_AND_EDGE_H_


#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include "g2o/core/robust_kernel_impl.h"
#include <sophus/se3.h>
#include <sophus/so3.h>

namespace g2o_graph
{

class VertexSE3LieAlgebra: public g2o::BaseVertex<6, Sophus::SE3>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	bool read(std::istream& is);
	bool write(std::ostream& os) const;
	bool read(Eigen::Vector3f t, 
	          Eigen::Quaternionf q);

	bool write(std::vector<double> id_time, 
	           std::ostream& os) const;
	
	virtual void setToOriginImpl();
	virtual void oplusImpl(const double* update);

	// added by lichunjing 2021-02-17
	Eigen::Vector3d translation();
	Eigen::Quaterniond quaternion();
};

class EdgeSE3LieAlgebra: public g2o::BaseBinaryEdge<6, Sophus::SE3, VertexSE3LieAlgebra, VertexSE3LieAlgebra>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	bool read(std::istream& is);

	bool read(Eigen::Vector3f t0, 
	          Eigen::Quaternionf q0, 
			  double score);
			  
	bool write(std::ostream& os) const;

	virtual void computeError();
	virtual void linearizeOplus();

	// added by lichunjing 2021-02-17
	VertexSE3LieAlgebra* getFirstVertexSE3LieAlgebra() const;
	VertexSE3LieAlgebra* getSecondVertexSE3LieAlgebra() const;
	// added by lichunjing 2021-02-17
	Eigen::Vector3d translation();
	Eigen::Quaterniond quaternion();
};

}    //namespace g2o_graph

#endif
