///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_02_17
///////////////////////////////////////////////////////////////////////////////

// #include <iostream>
// #include <sstream>
// #include <fstream>
#include "src/optimization/closeloop_optimization.h"



namespace optimization
{
CloseloopOptimization::CloseloopOptimization(std::string cfg_file_path,
                                             std::string project_directory_name)
    : cfg_file_path_(cfg_file_path),
      project_directory_name_(project_directory_name)
    //   initial_rotation_matrix_(Eigen::Matrix3d::Identity()),
    //   gnss_position_init_(Eigen::Vector3d::Zero()),
    //   is_get_gnss_position_init_(false)
{
    // // 雷达里程计信息矩阵
    // lidar_odom_information_ << 320.0,          0.0,         0.0,          0.0,          0.0,           0.0,
    //                            0.0,            320.0,       0.0,          0.0,          0.0,           0.0,
    //                            0.0,            0.0,         500.0,        0.0,          0.0,           0.0,
    //                            0.0,            0.0,         0.0,          20000.0,      0.0,           0.0,
    //                            0.0,            0.0,         0.0,          0.0,          20000.0,       0.0,
    //                            0.0,            0.0,         0.0,          0.0,          0.0,           20000.0;

    // // 雷达-gnss信息矩阵
    // lidar_gnss_information_ << 25.0,          0.0,           0.0,
    //                            0.0,            25.0,         0.0,
    //                            0.0,            0.0,           10.0;

}

CloseloopOptimization::~CloseloopOptimization()
{
    g2o_file_before_optimization_.close();
    g2o_file_after_optimization_.close();
    closeloop_optimization_results_filelist_.close();
}

void CloseloopOptimization::LoadConfig()
{
    cfg_file_ = YAML::LoadFile(cfg_file_path_ + "/cfg.yaml");

    std::string project_directory = cfg_file_["directory"]["project_directory"].as<std::string>();
    project_directory = project_directory + project_directory_name_;
    
    std::string optimization_main_directory = cfg_file_["directory"]["optimization"]["main_directory"].as<std::string>();
    
    std::string closeloop_before_optimization_file = cfg_file_["directory"]["optimization"]["closeloop_before_optimization_file"].as<std::string>();
    std::string closeloop_after_optimization_file = cfg_file_["directory"]["optimization"]["closeloop_after_optimization_file"].as<std::string>();
    std::string closeloop_optimization_results_file = cfg_file_["directory"]["optimization"]["closeloop_optimization_results_file"].as<std::string>();

    closeloop_optimization_parameter_.num_fixed_vertex = cfg_file_["closeloop_optimization"]["closeloop_optimization_parameter"]["num_fixed_vertex"].as<int>();
    closeloop_optimization_parameter_.max_iterations = cfg_file_["closeloop_optimization"]["closeloop_optimization_parameter"]["max_iterations"].as<int>();
    closeloop_optimization_parameter_.lidar_odom_translation_weight_xy = cfg_file_["closeloop_optimization"]["closeloop_optimization_parameter"]["lidar_odom_translation_weight_xy"].as<double>();
    closeloop_optimization_parameter_.lidar_odom_translation_weight_z = cfg_file_["closeloop_optimization"]["closeloop_optimization_parameter"]["lidar_odom_translation_weight_z"].as<double>();
    closeloop_optimization_parameter_.lidar_odom_rotation_weight = cfg_file_["closeloop_optimization"]["closeloop_optimization_parameter"]["lidar_odom_rotation_weight"].as<double>();


    std::string g2o_file_before_optimization_path = project_directory + optimization_main_directory + closeloop_before_optimization_file;
    std::string g2o_file_after_optimization_path = project_directory + optimization_main_directory + closeloop_after_optimization_file;
    std::string closeloop_optimization_results_file_path = project_directory + optimization_main_directory + closeloop_optimization_results_file;

    std::cout << std::endl;
    std::cout << "\033[32m" << "----------CloseloopOptimizationParameter parameter:----------" << "\033[0m" << std::endl;
    std::cout << "g2o_file_before_optimization_path:" << "\033[33m"  << g2o_file_before_optimization_path  << "\033[0m" << std::endl;
    std::cout << "g2o_file_after_optimization_path:" << "\033[33m"  << g2o_file_after_optimization_path  << "\033[0m" << std::endl;
    std::cout << "closeloop_optimization_results_file_path:" << "\033[33m"  << closeloop_optimization_results_file_path  << "\033[0m" << std::endl;
    std::cout << "num_fixed_vertex:" << "\033[33m"  << closeloop_optimization_parameter_.num_fixed_vertex  << "\033[0m" << std::endl;
    std::cout << "max_iterations:" << "\033[33m"  << closeloop_optimization_parameter_.max_iterations  << "\033[0m" << std::endl;
    std::cout << "lidar_odom_translation_weight_xy:" << "\033[33m"  << closeloop_optimization_parameter_.lidar_odom_translation_weight_xy  << "\033[0m" << std::endl;
    std::cout << "lidar_odom_translation_weight_z:" << "\033[33m"  << closeloop_optimization_parameter_.lidar_odom_translation_weight_z  << "\033[0m" << std::endl;
    std::cout << "lidar_odom_rotation_weight:" << "\033[33m"  << closeloop_optimization_parameter_.lidar_odom_rotation_weight  << "\033[0m" << std::endl;
    

    g2o_file_before_optimization_.open(g2o_file_before_optimization_path.c_str());
    g2o_file_after_optimization_.open(g2o_file_after_optimization_path.c_str());
    closeloop_optimization_results_filelist_.open(closeloop_optimization_results_file_path); 
}

void CloseloopOptimization::RunOptimization(std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists,
                                            std::vector<constraints::CloseloopConstraint> closeloop_constraints)
{
    registration_results_filelists_ = registration_results_filelists;

    LoadConfig();
    AddVertexes(registration_results_filelists);

    SetFixedVertexes(registration_results_filelists,
                     closeloop_constraints);

    AddLidarOdomEdges(registration_results_filelists);
    AddCloseloopEdges(closeloop_constraints);

    OutputG2oFile(g2o_file_before_optimization_);

    Optimize();

    OutputG2oFile(g2o_file_after_optimization_);

    OutputOptimizationResultsFile();
}

void CloseloopOptimization::AddVertexes(std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists)
{
    for(std::vector<mapping::RegistrationResultsFilelist>::iterator item = registration_results_filelists.begin();
                                                                    item != registration_results_filelists.end();
                                                                    item++)
    {
        g2o_graph::VertexSE3LieAlgebra* vertex = new g2o_graph::VertexSE3LieAlgebra();

        Eigen::Matrix4f transform = item->alignment_result.final_transform;
		Eigen::Vector3f translation = transform.block<3,1>(0,3);
		Eigen::Quaternionf quat = Eigen::Quaternionf(transform.block<3,3>(0,0));
		vertex->setId(item->allframe_id);
		vertex->read(translation, quat);

		lidar_vertices_.push_back(vertex);
    }

    for(int i=0; i< closeloop_optimization_parameter_.num_fixed_vertex; i++)
    {
        lidar_vertices_.at(i)->setFixed(true); 
    }
}

void CloseloopOptimization::SetFixedVertexes(std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists,
                                             std::vector<constraints::CloseloopConstraint> closeloop_constraints)
{
    for(std::vector<mapping::RegistrationResultsFilelist>::iterator item = registration_results_filelists.begin();
                                                                    item != registration_results_filelists.end();
                                                                    item++)
    {
        long long allframe_id = item->allframe_id;
        g2o_graph::VertexSE3LieAlgebra* vertex = SearchVertex(allframe_id);

        if(item->is_gnss_constraint)
        {
            vertex->setFixed(true);
        }
    }

    // 闭环约束中，将时间戳较早的位姿节点设置成 fixed 
    for(int i=0; i< closeloop_constraints.size(); i++)
    {
        long long allframe_id_from = closeloop_constraints.at(i).allframe_id_from;
        long long allframe_id_to = closeloop_constraints.at(i).allframe_id_to;

        mapping::RegistrationResultsFilelist registration_result_from = SearchRegistrationResults(allframe_id_from);
        mapping::RegistrationResultsFilelist registration_result_to = SearchRegistrationResults(allframe_id_to);

        if((registration_result_from.is_gnss_constraint) && (registration_result_to.is_gnss_constraint))
        {
            double time_from = common::ToUniversalSeconds(registration_result_from.time);
            double time_to = common::ToUniversalSeconds(registration_result_to.time);

            g2o_graph::VertexSE3LieAlgebra* vertex_from = SearchVertex(allframe_id_from);
            g2o_graph::VertexSE3LieAlgebra* vertex_to = SearchVertex(allframe_id_to);

            if(time_from < time_to)
            {
                vertex_to->setFixed(false);

                // added by lichunjing 2021-08-16
                for(int j=0; j<20; j++)
                {
                    g2o_graph::VertexSE3LieAlgebra* vertex_near = SearchVertex(allframe_id_to-j);
                    if(vertex_near)
                    {
                        vertex_near->setFixed(false);
                    }
                }
                for(int j=0; j<20; j++)
                {
                    g2o_graph::VertexSE3LieAlgebra* vertex_near = SearchVertex(allframe_id_to+j);
                    if(vertex_near)
                    {
                        vertex_near->setFixed(false);
                    }
                }
            }
            else if(time_from > time_to)
            {
                vertex_from->setFixed(false);

                // added by lichunjing 2021-08-16
                for(int j=0; j<20; j++)
                {
                    g2o_graph::VertexSE3LieAlgebra* vertex_near = SearchVertex(allframe_id_from-j);
                    if(vertex_near)
                    {
                        vertex_near->setFixed(false);
                    }
                }
                for(int j=0; j<20; j++)
                {
                    g2o_graph::VertexSE3LieAlgebra* vertex_near = SearchVertex(allframe_id_from+j);
                    if(vertex_near)
                    {
                        vertex_near->setFixed(false);
                    }
                }
            }
        }


    }

}

mapping::RegistrationResultsFilelist CloseloopOptimization::SearchRegistrationResults(long long target_allframe_id)
{
    for(std::vector<mapping::RegistrationResultsFilelist>::iterator item_lidar_pose = registration_results_filelists_.begin();
                                                                    item_lidar_pose != registration_results_filelists_.end();
                                                                    item_lidar_pose++)
    {
        if(item_lidar_pose->allframe_id == target_allframe_id)
        {
            return *item_lidar_pose;
        }
    }
}


void CloseloopOptimization::AddLidarOdomEdges(std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists)
{
    int edges_cnt = 0;
    for(int i=1; i< registration_results_filelists.size(); i++)
    {
        g2o_graph::EdgeSE3LieAlgebra* edge = new g2o_graph::EdgeSE3LieAlgebra();

        Eigen::Matrix4f relative_transform = registration_results_filelists.at(i-1).alignment_result.final_transform.inverse() 
                                           * registration_results_filelists.at(i).alignment_result.final_transform;
		Eigen::Vector3f translation = relative_transform.block<3,1>(0,3);
		Eigen::Quaternionf quat = Eigen::Quaternionf(relative_transform.block<3,3>(0,0));

        double score = closeloop_optimization_parameter_.lidar_odom_translation_weight_xy;
        edge->setId(edges_cnt++);
        edge->setVertex(0, lidar_vertices_.at(i-1));
        edge->setVertex(1, lidar_vertices_.at(i));
        edge->read(translation, quat, score);

		lidar_odom_edges_.push_back(edge);
    }
}

void CloseloopOptimization::AddCloseloopEdges(std::vector<constraints::CloseloopConstraint> closeloop_constraints)
{
    int edges_cnt = lidar_odom_edges_.size();
    for(int i=0; i< closeloop_constraints.size(); i++)
    {
        long long allframe_id_from = closeloop_constraints.at(i).allframe_id_from;
        long long allframe_id_to = closeloop_constraints.at(i).allframe_id_to;

        double score = closeloop_constraints.at(i).score;
        Eigen::Matrix4f transform = closeloop_constraints.at(i).relative_matrix;
        Eigen::Vector3f translation = transform.block<3,1>(0,3);
        Eigen::Quaternionf quat = Eigen::Quaternionf(transform.block<3,3>(0,0));

        g2o_graph::EdgeSE3LieAlgebra* edge = new g2o_graph::EdgeSE3LieAlgebra();
        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        rk->setDelta(0.1);
        edge->setRobustKernel(rk);
        edge->setId(edges_cnt++);
        edge->setVertex(0, SearchVertex(allframe_id_from));
        edge->setVertex(1, SearchVertex(allframe_id_to));
        edge->read(translation, quat, score);
        closeloop_edges_.push_back(edge);
    }
}

g2o_graph::VertexSE3LieAlgebra* CloseloopOptimization::SearchVertex(long long target_allframe_id) const
{
    g2o_graph::VertexSE3LieAlgebra* lidar_vertices_ptr = nullptr;

    for(int i=0; i< lidar_vertices_.size(); i++)
    {
        if(lidar_vertices_.at(i)->id() == target_allframe_id)
        {
            lidar_vertices_ptr = lidar_vertices_.at(i);
            
        }
    }

    return lidar_vertices_ptr;
}

void CloseloopOptimization::Optimize()
{
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> Block;
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>();
	Block* solver_ptr = new Block(linearSolver);
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
	g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
	// g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton ( solver_ptr );

	optimizer_.setAlgorithm(solver);


    for(int i=0; i<lidar_vertices_.size(); i++)
    {
        optimizer_.addVertex(lidar_vertices_.at(i));
    }

    for(int i=0; i<lidar_odom_edges_.size(); i++)
    {
        optimizer_.addEdge(lidar_odom_edges_.at(i));
    }

    for(int i=0; i<closeloop_edges_.size(); i++)
    {
        optimizer_.addEdge(closeloop_edges_.at(i));
    }

    optimizer_.setVerbose(true);
	optimizer_.initializeOptimization();
	std::cout << "starting closeloop Optimize! " << std::endl;

	optimizer_.optimize(closeloop_optimization_parameter_.max_iterations);
}

void CloseloopOptimization::OutputG2oFile(std::ofstream& g2o_file)
{
    std::string lidar_vertex_tag = "VERTEX_SE3:QUAT";
    std::string lidar_odom_edge_tag = "EDGE_SE3:QUAT";

    for(size_t i = 0; i < lidar_vertices_.size(); i++)
    {
        g2o_graph::VertexSE3LieAlgebra* vertex = lidar_vertices_.at(i);
        Eigen::Quaterniond quat = vertex->quaternion();
	    Eigen::Vector3d translation = vertex->translation();

        g2o_file << lidar_vertex_tag << " " << vertex->id() << " "
                 << translation(0) << " " << translation(1) << " " << translation(2) << " "
                 << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
                 << std::endl;
    }


    for(size_t i = 0; i < lidar_odom_edges_.size(); ++i)
    {
        g2o_graph::EdgeSE3LieAlgebra* edge = lidar_odom_edges_.at(i);

        g2o_graph::VertexSE3LieAlgebra* from = static_cast<g2o_graph::VertexSE3LieAlgebra*>(edge->getFirstVertexSE3LieAlgebra());
        g2o_graph::VertexSE3LieAlgebra* to = static_cast<g2o_graph::VertexSE3LieAlgebra*>(edge->getSecondVertexSE3LieAlgebra());

        Eigen::Quaterniond quat = edge->quaternion();
	    Eigen::Vector3d translation = edge->translation();

        g2o_file << lidar_odom_edge_tag << " " << from->id() << " " << to->id() << " "
                 << translation(0) << " " << translation(1) << " " << translation(2) << " "
                 << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
                 << std::endl;
    }

    for(size_t i = 0; i < closeloop_edges_.size(); ++i)
    {
        g2o_graph::EdgeSE3LieAlgebra* edge = closeloop_edges_.at(i);

        g2o_graph::VertexSE3LieAlgebra* from = static_cast<g2o_graph::VertexSE3LieAlgebra*>(edge->getFirstVertexSE3LieAlgebra());
        g2o_graph::VertexSE3LieAlgebra* to = static_cast<g2o_graph::VertexSE3LieAlgebra*>(edge->getSecondVertexSE3LieAlgebra());
        Eigen::Quaterniond quat = edge->quaternion();
	    Eigen::Vector3d translation = edge->translation();

        g2o_file << lidar_odom_edge_tag << " " << from->id() << " " << to->id() << " "
                 << translation(0) << " " << translation(1) << " " << translation(2) << " "
                 << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
                 << std::endl;
    }

}

void CloseloopOptimization::OutputOptimizationResultsFile()
{
    std::cout << "registration_results_filelists_.size:" << registration_results_filelists_.size() << std::endl;
    std::cout << "lidar_vertices_.size:" << lidar_vertices_.size() << std::endl;

    for(int i=0; i<registration_results_filelists_.size(); i++)
    {
        mapping::RegistrationResultsFilelist& registration_results_filelist = registration_results_filelists_.at(i);

        g2o_graph::VertexSE3LieAlgebra* vertex = lidar_vertices_.at(i);
        Eigen::Quaterniond quat = vertex->quaternion();
	    Eigen::Vector3d translation = vertex->translation();

        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3,3>(0,0) = quat.toRotationMatrix();
        transform.block<3,1>(0,3) = translation;

        Eigen::Matrix4f transform_f;
        transform_f(0,0) = transform(0,0); transform_f(0,1) = transform(0,1); transform_f(0,2) = transform(0,2); transform_f(0,3) = transform(0,3);
        transform_f(1,0) = transform(1,0); transform_f(1,1) = transform(1,1); transform_f(1,2) = transform(1,2); transform_f(1,3) = transform(1,3);
        transform_f(2,0) = transform(2,0); transform_f(2,1) = transform(2,1); transform_f(2,2) = transform(2,2); transform_f(2,3) = transform(2,3);
        transform_f(3,0) = transform(3,0); transform_f(3,1) = transform(3,1); transform_f(3,2) = transform(3,2); transform_f(3,3) = transform(3,3);


        registration_results_filelist.alignment_result.final_transform = transform_f;

        registration_results_filelist.is_gnss_constraint = false;

        closeloop_optimization_results_filelist_ << std::setprecision(15)
                                                 << registration_results_filelist.allframe_id
                                                 << " " << registration_results_filelist.time
                                                 << " " << registration_results_filelist.is_gnss_constraint
                                                 << " " << registration_results_filelist.pcd_file_path
                                                 << " " << registration_results_filelist.alignment_result.is_converged
                                                 << " " << registration_results_filelist.alignment_result.fitness_score
                                                 << " " << registration_results_filelist.alignment_result.time_duration_ms
                                                 << " " << registration_results_filelist.alignment_result.final_num_iteration
                                                 << " " << registration_results_filelist.alignment_result.final_transform(0,0) << " " << registration_results_filelist.alignment_result.final_transform(0,1) << " " << registration_results_filelist.alignment_result.final_transform(0,2) << " " << registration_results_filelist.alignment_result.final_transform(0,3)
                                                 << " " << registration_results_filelist.alignment_result.final_transform(1,0) << " " << registration_results_filelist.alignment_result.final_transform(1,1) << " " << registration_results_filelist.alignment_result.final_transform(1,2) << " " << registration_results_filelist.alignment_result.final_transform(1,3)
                                                 << " " << registration_results_filelist.alignment_result.final_transform(2,0) << " " << registration_results_filelist.alignment_result.final_transform(2,1) << " " << registration_results_filelist.alignment_result.final_transform(2,2) << " " << registration_results_filelist.alignment_result.final_transform(2,3)
                                                 << " " << registration_results_filelist.alignment_result.final_transform(3,0) << " " << registration_results_filelist.alignment_result.final_transform(3,1) << " " << registration_results_filelist.alignment_result.final_transform(3,2) << " " << registration_results_filelist.alignment_result.final_transform(3,3)
                                                 << std::endl;
    }

}




}  // namespace optimization


