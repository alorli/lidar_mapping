///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_02_17
///////////////////////////////////////////////////////////////////////////////
#ifndef CLOSELOOP_OPTIMIZATION_H_
#define CLOSELOOP_OPTIMIZATION_H_

// #include "src/common/time.h"
// #include "src/common/time_conversion.h"
// #include "src/map_generator.h"
#include "src/constraints/closeloop_constraints_builder.h"
#include "3rd_party/graph/vertex_and_edge.h"

// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/registration/ndt.h>

// // #include<Eigen/Core>
// #include <Eigen/Geometry>
// #include <vector>
#include <string>
#include <fstream>
#include "yaml-cpp/yaml.h"

// #include "g2o/core/sparse_optimizer.h"
// #include "g2o/core/block_solver.h"
// #include "g2o/core/factory.h"
// #include "g2o/core/optimization_algorithm_levenberg.h"
// #include "g2o/solvers/csparse/linear_solver_csparse.h"

// #include "g2o/types/slam3d/vertex_se3.h"
// #include "g2o/types/slam3d/edge_se3.h"

// #include <g2o/types/slam3d/types_slam3d.h>
// #include <g2o/types/slam3d_addons/types_slam3d_addons.h>


namespace optimization
{

struct CloseloopOptimizationParameter
{
    int num_fixed_vertex;
    int max_iterations;
    double lidar_odom_translation_weight_xy;
    double lidar_odom_translation_weight_z;
    double lidar_odom_rotation_weight;
};

class CloseloopOptimization
{
public:
    CloseloopOptimization(std::string cfg_file_path, 
                          std::string project_directory_name);
    ~CloseloopOptimization();

    void RunOptimization(std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists,
                         std::vector<constraints::CloseloopConstraint> closeloop_constraints);


private:
    void LoadConfig();

    void AddVertexes(std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists);
    void SetFixedVertexes(std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists,
                          std::vector<constraints::CloseloopConstraint> closeloop_constraints);
    mapping::RegistrationResultsFilelist SearchRegistrationResults(long long target_allframe_id);
    void AddLidarOdomEdges(std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists);
    void AddCloseloopEdges(std::vector<constraints::CloseloopConstraint> closeloop_constraints);

    g2o_graph::VertexSE3LieAlgebra* SearchVertex(long long target_allframe_id) const;

    void OutputG2oFile(std::ofstream& g2o_file);
    void OutputOptimizationResultsFile();

    void Optimize();


private:
    YAML::Node cfg_file_;
    std::string cfg_file_path_;
    std::string project_directory_name_;

    g2o::SparseOptimizer optimizer_;

    std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists_;

    CloseloopOptimizationParameter closeloop_optimization_parameter_;

    std::vector<g2o_graph::VertexSE3LieAlgebra*> lidar_vertices_;
    std::vector<g2o_graph::EdgeSE3LieAlgebra*> lidar_odom_edges_;
	std::vector<g2o_graph::EdgeSE3LieAlgebra*> closeloop_edges_;

    std::ofstream g2o_file_before_optimization_;
    std::ofstream g2o_file_after_optimization_;
    std::ofstream closeloop_optimization_results_filelist_;
};

} // namespace optimization

#endif