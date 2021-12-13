/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

Authors: Andreas Geiger

libicp is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libicp is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libicp; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

// #ifndef ICP_H
// #define ICP_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

#include "matrix.h"
#include "kdtree.h"
#include <iostream>
#include <chrono>
#include "icpPointToPoint.h"
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "omp.h"
#include "icp.h"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ICP_OMP {

public:

	// constructor
	// input: M ....... pointer to first model point
	//        M_num ... number of model points
	//        dim   ... dimensionality of model points (2 or 3)
	ICP_OMP ();

	// deconstructor
	~ICP_OMP ();
		
	void release();
	
	void releaseT();

	void setInputTarget(const PointCloudT::Ptr cloud_target);
			
	void setInputSource(const PointCloudT::Ptr cloud_source);

	void setTransform(Eigen::Matrix4d& transform);

	void align(Eigen::Matrix4f transformation_matrix, double neighbor_threshold);
	
	void eigen2matrix(Eigen::Matrix4f& transform, Matrix& R, Matrix& t);

	void matrix2eigen(Eigen::Matrix4f& transform, Matrix& R, Matrix& t);
	
	void eigen2matrix(Eigen::Matrix4d& transform, Matrix& R, Matrix& t);

	void matrix2eigen(Eigen::Matrix4d& transform, Matrix& R, Matrix& t);
	
	Eigen::Matrix4f getFinalTransformation();
	
	double getFitnessScore();
	
	double getResidual(double dist_threshold);
  
protected:
    
	double* M;
	double* T;
	IcpPointToPoint* icp;

	Matrix R;
	Matrix t;
	Eigen::Matrix4f transform;
	int target_size, source_size;
	double fitness_score;
};

