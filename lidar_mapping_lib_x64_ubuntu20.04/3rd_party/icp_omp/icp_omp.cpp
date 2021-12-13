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

#include "icp_omp.h"
using namespace std;

ICP_OMP::ICP_OMP () {
    R = Matrix::eye(3);
    t = Matrix::ones(3,1);
    transform = Eigen::Matrix4f::Identity();
    M = NULL;
    T = NULL;
    icp = NULL;
}

ICP_OMP::~ICP_OMP () {
    if(M)
        free(M);
    if(T)
        free(T);
    if(icp)
        delete icp;
}

void ICP_OMP::release() {
    if(M)
        free(M);
    if(T)
        free(T);
    if(icp)
        delete icp;
}
void ICP_OMP::releaseT() {
	if(T)
		free(T);
}
void ICP_OMP::eigen2matrix(Eigen::Matrix4f& transform, Matrix& R, Matrix& t) {
    R.setVal(transform(0,0), 0, 0, 0, 0); R.setVal(transform(0,1), 0, 1, 0, 1); R.setVal(transform(0,2), 0, 2, 0, 2);
    R.setVal(transform(1,0), 1, 0, 1, 0); R.setVal(transform(1,1), 1, 1, 1, 1); R.setVal(transform(1,2), 1, 2, 1, 2);
    R.setVal(transform(2,0), 2, 0, 2, 0); R.setVal(transform(2,1), 2, 1, 2, 1); R.setVal(transform(2,2), 2, 2, 2, 2);
    t.setVal(transform(0,3), 0, 0, 0, 0);
    t.setVal(transform(1,3), 1, 0, 1, 0);
    t.setVal(transform(2,3), 2, 0, 2, 0);
}

void ICP_OMP::matrix2eigen(Eigen::Matrix4f& transform, Matrix& R, Matrix& t) {
    transform.block<3,4>(0,0) << R.val[0][0], R.val[0][1], R.val[0][2], t.val[0][0],
                                R.val[1][0], R.val[1][1], R.val[1][2], t.val[1][0],
                                R.val[2][0], R.val[2][1], R.val[2][2], t.val[2][0];
}
void ICP_OMP::eigen2matrix(Eigen::Matrix4d& transform, Matrix& R, Matrix& t) {
    R.setVal(transform(0,0), 0, 0, 0, 0); R.setVal(transform(0,1), 0, 1, 0, 1); R.setVal(transform(0,2), 0, 2, 0, 2);
    R.setVal(transform(1,0), 1, 0, 1, 0); R.setVal(transform(1,1), 1, 1, 1, 1); R.setVal(transform(1,2), 1, 2, 1, 2);
    R.setVal(transform(2,0), 2, 0, 2, 0); R.setVal(transform(2,1), 2, 1, 2, 1); R.setVal(transform(2,2), 2, 2, 2, 2);
    t.setVal(transform(0,3), 0, 0, 0, 0);
    t.setVal(transform(1,3), 1, 0, 1, 0);
    t.setVal(transform(2,3), 2, 0, 2, 0);
}

void ICP_OMP::matrix2eigen(Eigen::Matrix4d& transform, Matrix& R, Matrix& t) {
    transform.block<3,4>(0,0) << R.val[0][0], R.val[0][1], R.val[0][2], t.val[0][0],
                                R.val[1][0], R.val[1][1], R.val[1][2], t.val[1][0],
                                R.val[2][0], R.val[2][1], R.val[2][2], t.val[2][0];
}
void ICP_OMP::setInputTarget(const PointCloudT::Ptr cloud_target) {
    target_size = cloud_target->size();
    M = (double*)calloc(3*target_size, sizeof(double));
    for(int i=0; i<cloud_target->size(); i++) {
        M[i*3+0] = cloud_target->points[i].x;
        M[i*3+1] = cloud_target->points[i].y;
        M[i*3+2] = cloud_target->points[i].z;
    }
    
    icp = new IcpPointToPoint(M, target_size, 3);
}

void ICP_OMP::setInputSource(const PointCloudT::Ptr cloud_source) {
    source_size = cloud_source->size();
    T = (double*)calloc(3*source_size, sizeof(double));
    for(int i=0; i<cloud_source->size(); i++) {
        T[i*3+0] = cloud_source->points[i].x;
        T[i*3+1] = cloud_source->points[i].y;
        T[i*3+2] = cloud_source->points[i].z;
    }
}


void ICP_OMP::align(Eigen::Matrix4f transformation_matrix, double neighbor_threshold) {
    eigen2matrix(transformation_matrix, R, t);
//     icp = new IcpPointToPoint(M, target_size, 3);
    
    fitness_score = icp->fit(T, source_size, R, t, neighbor_threshold);
    matrix2eigen(transform, R, t);
    free(T);
    T = NULL;
}

Eigen::Matrix4f ICP_OMP::getFinalTransformation() {
    return transform;
}

double ICP_OMP::getFitnessScore() {
    return fitness_score;
}

double ICP_OMP::getResidual(double dist_threshold) {
    return icp->getError(T, source_size, R, t, dist_threshold);
}

void ICP_OMP::setTransform(Eigen::Matrix4d& transform) {
	eigen2matrix(transform, R, t);
}
