///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_01_26
///////////////////////////////////////////////////////////////////////////////
#ifndef GNSS_OPTIMIZATION_H_
#define GNSS_OPTIMIZATION_H_

// #include "src/common/time.h"
// #include "src/common/time_conversion.h"
// #include "src/map_generator.h"
#include "src/constraints/gnss_constraints_builder.h"

// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/registration/ndt.h>

// // #include<Eigen/Core>
// #include <Eigen/Geometry>
// #include <vector>
#include <string>
#include <fstream>
#include "yaml-cpp/yaml.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>


namespace optimization
{

struct GnssOptimizationParameter
{
    double lidar_keyframe_distance;
    double use_distance_lidar_pose_estimate_init_angle;
    double distance_interval_add_prior_gnss_pose;
    int max_iterations;
};

struct LeverArmParameter
{
    bool use_lever_arm;
    double lever_arm_x;
    double lever_arm_y;
    double lever_arm_z;
};

// G2O_USE_TYPE_GROUP 保证能够进graph的load 和save, 进行类型注册, 也就是optimizer.load 和optimizer.save 在执行时有相对的类型对应. 
// slam3d 和slam2d是对应不同的顶点和边的类型
G2O_USE_TYPE_GROUP(slam3d);

class EdgeSE3PriorXYZ : public g2o::BaseUnaryEdge<3, g2o::Vector3D, g2o::VertexSE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3PriorXYZ()
      : g2o::BaseUnaryEdge<3, g2o::Vector3D, g2o::VertexSE3>()
    {}

    void computeError() override
    {
        const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

        Eigen::Vector3d estimate = v1->estimate().translation();
        _error = estimate - _measurement;
    }

    void setMeasurement(const g2o::Vector3D& m) override
    {
        _measurement = m;
    }

    virtual bool read(std::istream& is) override
    {
        g2o::Vector3D v;
        is >> v(0) >> v(1) >> v(2);

        setMeasurement(g2o::Vector3D(v));

        for(int i = 0; i < information().rows(); ++i)
        {
            for(int j = i; j < information().cols(); ++j)
            {
              is >> information()(i, j);

              if(i != j)
              {
                  information()(j, i) = information()(i, j);
              }
                
            }
        }
        return true;
    }
    
    virtual bool write(std::ostream& os) const override
    {
        g2o::Vector3D v = _measurement;
        os << v(0) << " " << v(1) << " " << v(2) << " ";

        for(int i = 0; i < information().rows(); ++i)
        {
            for(int j = i; j < information().cols(); ++j)
            {
                os << " " << information()(i, j);
            }              
        }
        return os.good();
    }
};


class GnssOptimization
{
public:
    GnssOptimization(std::string cfg_file_path);
    ~GnssOptimization();

    void RunOptimization(std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists,
                         std::vector<constraints::GnssPoseConstraint> gnss_pose_constraints);


private:
    void LoadConfig();

    bool Initialize(std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists,
                    std::vector<constraints::GnssPoseConstraint> gnss_pose_constraints);

    void SelectOptimizationFrame();
    void AddVertexes();
    void AddLidarOdomVertex(Eigen::Vector3f translation,
                            Eigen::Matrix3f rotation, 
                            long long vertex_id);
    void AddGnssVertex(Eigen::Vector3d translation,
                       long long vertex_id);

    void AddEdges();
    void AddLidarOdomEdges();
    void AddLidarGnssEdges();

    void AddVertexesAndEdgesToOptimizer(g2o::SparseOptimizer& optimizer);

    void EstimateInitAngle();

    void AlignToGnss();

    void OutputG2oFile(std::ofstream& g2o_file);
    void OutputOptimizationResultsFile();
    void OutputNotOptimizationResultsFile();


private:
    YAML::Node cfg_file_;
    std::string cfg_file_path_;

    std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists_;
    std::vector<mapping::RegistrationResultsFilelist> registration_optimization_results_filelists_;
    std::vector<constraints::GnssPoseConstraint> gnss_pose_constraints_;
    std::vector<constraints::GnssPoseConstraint> gnss_optimization_pose_constraints_;

    GnssOptimizationParameter gnss_optimization_parameter_;
    LeverArmParameter lever_arm_parameter_;

    std::vector<g2o::VertexSE3*> lidar_keyframe_vertices_;
    std::vector<g2o::VertexPointXYZ*> gnss_vertices_;
    std::vector<g2o::EdgeSE3*> lidar_keyframe_edges_;
    std::vector<EdgeSE3PriorXYZ*> lidar_keyframe_gnss_edges_;

    std::deque<Eigen::Vector3d> current_segment_gnss_pose_arrary_;


    Eigen::Matrix<double, 6, 6> lidar_odom_information_;
    Eigen::Matrix<double, 3, 3> lidar_gnss_information_;

    Eigen::Vector3d gnss_position_init_;
    Eigen::Vector3f lidar_position_init_;
    bool is_get_gnss_position_init_;

    // 通过 ICP算法估计出来的初始雷达轨迹和 gnss轨迹的初始偏差角
    Eigen::Matrix3d initial_rotation_matrix_;

    std::ofstream g2o_file_before_optimization_;
    std::ofstream g2o_file_after_optimization_;
    std::ofstream gnss_optimization_results_filelist_;
    std::ofstream gnss_init_pose_file_;
};

} // namespace io

#endif