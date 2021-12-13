
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include "g2o/core/robust_kernel_impl.h"
#include <sophus/se3.h>
#include <sophus/so3.h>

#include "vertex_and_edge.h"

namespace g2o_graph
{

typedef Eigen::Matrix<double,6,6> Matrix6d;

Matrix6d JRInv(Sophus::SE3 e)
{
    Matrix6d J;

    J.block(0,0,3,3) = Sophus::SO3::hat(e.so3().log());
    J.block(0,3,3,3) = Sophus::SO3::hat(e.translation());
    J.block(3,0,3,3) = Eigen::Matrix3d::Zero(3,3);
    J.block(3,3,3,3) = Sophus::SO3::hat(e.so3().log());
    J = J*0.5 + Matrix6d::Identity();

    return J;
}

typedef Eigen::Matrix<double, 6, 1> Vector6d;

bool VertexSE3LieAlgebra::read(std::istream& is){}
bool VertexSE3LieAlgebra::write(std::ostream& os) const {}

bool VertexSE3LieAlgebra::read(Eigen::Vector3f t, 
                               Eigen::Quaternionf q) 
{
	setEstimate(Sophus::SE3(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()), 
	                        Eigen::Vector3d(t[0], t[1], t[2])));
}

bool VertexSE3LieAlgebra::write(std::vector<double> id_time, 
                                std::ostream& os) const
{
	double time = id_time[id()];
	Eigen::Quaterniond q = _estimate.unit_quaternion();
	Eigen::Vector3d t = _estimate.translation();
	Eigen::Matrix4d Twv = Eigen::Matrix4d::Identity();
	Twv.block<3,3>(0,0) = q.toRotationMatrix();
	Twv.block<3,1>(0,3) = t;
	os << std::setprecision(15) << time << " " << Twv(0,0) << " " <<  Twv(0,1) << " " << Twv(0,2) << " " << Twv(0,3) 
										<< " " << Twv(1,0) << " " <<  Twv(1,1) << " " << Twv(1,2) << " " << Twv(1,3) 
										<< " " << Twv(2,0) << " " <<  Twv(2,1) << " " << Twv(2,2) << " " << Twv(2,3) 
										<< " " << Twv(3,0) << " " <<  Twv(3,1) << " " << Twv(3,2) << " " << Twv(3,3) 
										<< std::endl;
	return true;
}

void VertexSE3LieAlgebra::setToOriginImpl()
{
	_estimate = Sophus::SE3();
}

void VertexSE3LieAlgebra::oplusImpl(const double* update)
{
	Sophus::SE3 up(
			Sophus::SO3(update[3], update[4], update[5]),
			Eigen::Vector3d(update[0], update[1], update[2])
	);
	_estimate = up*_estimate;
}


// added by lichunjing 2021-02-17
Eigen::Vector3d VertexSE3LieAlgebra::translation()
{
	return _estimate.translation();
}

// added by lichunjing 2021-02-17
Eigen::Quaterniond VertexSE3LieAlgebra::quaternion()
{
	return _estimate.unit_quaternion();
}










bool EdgeSE3LieAlgebra::read(std::istream& is) {}

bool EdgeSE3LieAlgebra::read(Eigen::Vector3f t0, 
                             Eigen::Quaternionf q0, 
							 double score)
{
	Eigen::Quaterniond q(q0.w(), q0.x(), q0.y(), q0.z());

	q.normalize();

	setMeasurement(
			Sophus::SE3(q, Eigen::Vector3d(t0[0], t0[1], t0[2])) 
	);

	for(int i=0; i<information().rows(); i++)
	{
		for( int j=i; j<information().cols(); j++)
		{
			if(i==j)
			{
				information()(i,j) = score;
			}
				
			else 
			{
				information()(i,j) = 0;
				information()(j,i) = information()(i,j);
			}
		}
	}

	return true;
}
bool EdgeSE3LieAlgebra::write(std::ostream& os) const
{
	VertexSE3LieAlgebra* v1 = static_cast<VertexSE3LieAlgebra*>(_vertices[0]);
	VertexSE3LieAlgebra* v2 = static_cast<VertexSE3LieAlgebra*>(_vertices[1]);

	os << v1->id() << " " << v2->id() << " ";

	Sophus::SE3 m = _measurement;

	Eigen::Quaterniond q = m.unit_quaternion();
	os << m.translation().transpose() << " ";

	os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

	// information matrix 
	for(int i=0; i<information().rows(); i++)
	{
		for(int j=i; j<information().cols(); j++)
		{
			os << information()(i,j) << " ";
		}
	}

	os << std::endl;
	return true;
}

void EdgeSE3LieAlgebra::computeError()
{
	Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
	Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*> (_vertices[1]))->estimate();

	_error = (_measurement.inverse()*v1.inverse()*v2).log();
}

//     virtual void linearizeOplus()
//     {
//         Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
//         Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*> (_vertices[1]))->estimate();
//         Matrix6d J = JRInv(SE3::exp(_error));
//         // 尝试把J近似为I？
//         _jacobianOplusXi = - J* v2.inverse().Adj();
//         _jacobianOplusXj = J*v2.inverse().Adj();
//     }

void EdgeSE3LieAlgebra::linearizeOplus()
{
	Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
	Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*> (_vertices[1]))->estimate();

	Matrix6d J = JRInv(Sophus::SE3::exp(_error));
	
	// 尝试把J近似为I？
	J = Matrix6d::Identity();
	_jacobianOplusXi = - J* v2.inverse().Adj();
	_jacobianOplusXj = J*v2.inverse().Adj();
}


// added by lichunjing 2021-02-17
VertexSE3LieAlgebra* EdgeSE3LieAlgebra::getFirstVertexSE3LieAlgebra() const
{
	return static_cast<VertexSE3LieAlgebra*>(_vertices[0]);
}


// added by lichunjing 2021-02-17
VertexSE3LieAlgebra* EdgeSE3LieAlgebra::getSecondVertexSE3LieAlgebra() const
{
	return static_cast<VertexSE3LieAlgebra*>(_vertices[1]);
}


// added by lichunjing 2021-02-17
Eigen::Vector3d EdgeSE3LieAlgebra::translation()
{
	return _measurement.translation();
}

// added by lichunjing 2021-02-17
Eigen::Quaterniond EdgeSE3LieAlgebra::quaternion()
{
	return _measurement.unit_quaternion();
}


}    //namespace g2o_graph

