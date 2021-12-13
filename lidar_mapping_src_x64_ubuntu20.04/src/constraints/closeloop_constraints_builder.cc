///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_02_08
///////////////////////////////////////////////////////////////////////////////


#include "src/common/common.h"
#include "src/constraints/closeloop_constraints_builder.h"
#include "3rd_party/icp_omp/icp_omp.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>


#include <iostream>
// #include <sstream>
#include <fstream>
#include <chrono>
#include "omp.h"


namespace constraints
{

CloseloopConstraintsBuilder::CloseloopConstraintsBuilder(std::string cfg_file_path)
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");

    std::string project_directory = cfg_file_["directory"]["project_directory"].as<std::string>();
    std::string optimization_main_directory = cfg_file_["directory"]["optimization"]["main_directory"].as<std::string>();
    std::string gnss_optimization_results_file = cfg_file_["directory"]["optimization"]["gnss_optimization_results_file"].as<std::string>();

    closeloop_constraints_builder_parameter_.num_threads = cfg_file_["closeloop_constraints_builder"]["closeloop_constraints_builder_parameter"]["num_threads"].as<int>();
    closeloop_constraints_builder_parameter_.min_score = cfg_file_["closeloop_constraints_builder"]["closeloop_constraints_builder_parameter"]["min_score"].as<double>();
    closeloop_constraints_builder_parameter_.select_interval = cfg_file_["closeloop_constraints_builder"]["closeloop_constraints_builder_parameter"]["select_interval"].as<int>();
    closeloop_constraints_builder_parameter_.max_constraints_distance = cfg_file_["closeloop_constraints_builder"]["closeloop_constraints_builder_parameter"]["max_constraints_distance"].as<double>();
    closeloop_constraints_builder_parameter_.min_constraints_distance = cfg_file_["closeloop_constraints_builder"]["closeloop_constraints_builder_parameter"]["min_constraints_distance"].as<double>();
    closeloop_constraints_builder_parameter_.max_constraints_distance_z = cfg_file_["closeloop_constraints_builder"]["closeloop_constraints_builder_parameter"]["max_constraints_distance_z"].as<double>();
    closeloop_constraints_builder_parameter_.min_constraints_time = cfg_file_["closeloop_constraints_builder"]["closeloop_constraints_builder_parameter"]["min_constraints_time"].as<double>();
    
    
    // std::string velodyne_main_directory = cfg_file_["directory"]["velodyne"]["main_directory"].as<std::string>();
    // std::string gnss_main_directory = cfg_file_["directory"]["gnss"]["main_directory"].as<std::string>();
    // std::string compensation_directory = cfg_file_["directory"]["velodyne"]["compensation_directory"].as<std::string>();


    lidar_odom_pose_path_ = project_directory + optimization_main_directory + gnss_optimization_results_file;

    std::cout << std::endl;
    std::cout << "\033[32m" << "----------CloseloopConstraintsBuilderParameter parameter:----------" << "\033[0m" << std::endl;
    std::cout << "project_directory:" << "\033[33m" << project_directory << "\033[0m" << std::endl;
    std::cout << "optimization_main_directory:" << "\033[33m" << optimization_main_directory << "\033[0m"  << std::endl;
    std::cout << "gnss_optimization_results_file:" << "\033[33m" << gnss_optimization_results_file << "\033[0m" << std::endl;
    std::cout << "lidar_odom_pose_path_:" << "\033[33m" << lidar_odom_pose_path_ << "\033[0m" << std::endl;

    std::cout << "closeloop_constraints_builder_parameter_.num_threads:" << "\033[33m" << closeloop_constraints_builder_parameter_.num_threads << "\033[0m" << std::endl;
    std::cout << "closeloop_constraints_builder_parameter_.min_score:" << "\033[33m" << closeloop_constraints_builder_parameter_.min_score << "\033[0m" << std::endl;
    std::cout << "closeloop_constraints_builder_parameter_.select_interval:" << "\033[33m" << closeloop_constraints_builder_parameter_.select_interval << "\033[0m" << std::endl;
    std::cout << "closeloop_constraints_builder_parameter_.max_constraints_distance_z:" << "\033[33m" << closeloop_constraints_builder_parameter_.max_constraints_distance_z << "\033[0m" << std::endl;
    std::cout << "closeloop_constraints_builder_parameter_.min_constraints_time:" << "\033[33m" << closeloop_constraints_builder_parameter_.min_constraints_time << "\033[0m" << std::endl;
    std::cout << "closeloop_constraints_builder_parameter_.min_constraints_distance:" << "\033[33m" << closeloop_constraints_builder_parameter_.min_constraints_distance << "\033[0m" << std::endl;
    std::cout << "closeloop_constraints_builder_parameter_.max_constraints_distance:" << "\033[33m" << closeloop_constraints_builder_parameter_.max_constraints_distance << "\033[0m" << std::endl;
}

CloseloopConstraintsBuilder::~CloseloopConstraintsBuilder()
{
}

void CloseloopConstraintsBuilder::BuildConstraints()
{
    LoadLidarOdomPoses();
    SelectPairs();
    ComputeConstraint();
    ErasePairs();
    
    std::cout << "closeloop_constraints_.size:" << "\033[32m" << closeloop_constraints_.size() << "\033[0m" 
              << std::endl;
}


void CloseloopConstraintsBuilder::LoadLidarOdomPoses()
{
    std::ifstream filelists;
    filelists.open(lidar_odom_pose_path_);
    mapping::RegistrationResultsFilelist registration_results_filelist;
    long long time;
    while(filelists >> registration_results_filelist.allframe_id 
                    >> time
                    >> registration_results_filelist.is_gnss_constraint
                    >> registration_results_filelist.pcd_file_path
                    >> registration_results_filelist.alignment_result.is_converged
                    >> registration_results_filelist.alignment_result.fitness_score
                    >> registration_results_filelist.alignment_result.time_duration_ms
                    >> registration_results_filelist.alignment_result.final_num_iteration
                    >> registration_results_filelist.alignment_result.final_transform(0,0) >> registration_results_filelist.alignment_result.final_transform(0,1) >> registration_results_filelist.alignment_result.final_transform(0,2) >> registration_results_filelist.alignment_result.final_transform(0,3)
                    >> registration_results_filelist.alignment_result.final_transform(1,0) >> registration_results_filelist.alignment_result.final_transform(1,1) >> registration_results_filelist.alignment_result.final_transform(1,2) >> registration_results_filelist.alignment_result.final_transform(1,3)
                    >> registration_results_filelist.alignment_result.final_transform(2,0) >> registration_results_filelist.alignment_result.final_transform(2,1) >> registration_results_filelist.alignment_result.final_transform(2,2) >> registration_results_filelist.alignment_result.final_transform(2,3)
                    >> registration_results_filelist.alignment_result.final_transform(3,0) >> registration_results_filelist.alignment_result.final_transform(3,1) >> registration_results_filelist.alignment_result.final_transform(3,2) >> registration_results_filelist.alignment_result.final_transform(3,3)
    )
    {
        registration_results_filelist.time = common::FromUniversal(time);
        registration_results_filelists_.push_back(registration_results_filelist);

        // std::cout << "size:" << "\033[32m" << registration_results_filelists_.size() << "\033[0m" 
        //           << " allframe_id: " << "\033[32m" << registration_results_filelist.allframe_id << "\033[0m" 
        //           << " time: " << "\033[32m" << registration_results_filelist.time << "\033[0m" 
        //           << " pcd_file_path: " << "\033[32m" << registration_results_filelist.pcd_file_path << "\033[0m" 
        //         //   << " final_transform: " << "\033[32m" << registration_results_filelist.alignment_result.final_transform
        //           << std::endl;
    }
    filelists.close();
}


void CloseloopConstraintsBuilder::SelectPairs()
{
	for(int i=0; i<registration_results_filelists_.size(); i=i+closeloop_constraints_builder_parameter_.select_interval)
	{
		for(int j=i+closeloop_constraints_builder_parameter_.select_interval; 
            j<registration_results_filelists_.size(); 
            j=j+closeloop_constraints_builder_parameter_.select_interval)
		{
			//TODO：闭环约束在非 gnss约束优化的点进行搜索
            if((registration_results_filelists_.at(i).is_gnss_constraint) || (registration_results_filelists_.at(j).is_gnss_constraint))
            {
                continue;
            }


            Eigen::Vector3f translation_i, translation_j;
            translation_i = registration_results_filelists_.at(i).alignment_result.final_transform.block<3,1>(0,3);
			translation_j = registration_results_filelists_.at(j).alignment_result.final_transform.block<3,1>(0,3);


			// 两个位姿之间的角度增量
            double yaw_i = registration_results_filelists_.at(i).alignment_result.final_transform.block<3, 3>(0,0).eulerAngles(2,1,0).transpose()(0);
            double yaw_j = registration_results_filelists_.at(j).alignment_result.final_transform.block<3, 3>(0,0).eulerAngles(2,1,0).transpose()(0);
            
			double delta_angle = common::radian_to_degree(yaw_j) - common::radian_to_degree(yaw_i);

			// 两个位姿之间的平移量增量
            double delta_x = translation_j(0)-translation_i(0);
            double delta_y = translation_j(1)-translation_i(1);
            double delta_z = translation_j(2)-translation_i(2);
			double delta_distance = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);

			// 高程增量
			double distance_z = sqrt(delta_z*delta_z);


			if(distance_z > closeloop_constraints_builder_parameter_.max_constraints_distance_z)
			{
				continue;
			}

			// 时间挨的太近，不认为是约束关系
            double delta_time = common::ToUniversalSeconds(registration_results_filelists_.at(j).time) 
                              - common::ToUniversalSeconds(registration_results_filelists_.at(i).time);

			if(delta_time < closeloop_constraints_builder_parameter_.min_constraints_time)
			{
				continue;
			}


			// 如果距离太远，或者距离太近，不认为是约束关系
			if(delta_distance > closeloop_constraints_builder_parameter_.max_constraints_distance 
            || delta_distance < closeloop_constraints_builder_parameter_.min_constraints_distance 
            || j<0 
            || j>=registration_results_filelists_.size())
			{
				continue;
			}
			else
			{
                CloseloopConstraint closeloop_constraint;
                closeloop_constraint.allframe_id_from = registration_results_filelists_.at(i).allframe_id;
                closeloop_constraint.allframe_id_to = registration_results_filelists_.at(j).allframe_id;
                closeloop_constraint.score = -1;
                closeloop_constraint.relative_matrix = Eigen::Matrix4f::Identity();
				closeloop_constraints_.push_back(closeloop_constraint);
			}
		}
	}
}


mapping::RegistrationResultsFilelist CloseloopConstraintsBuilder::SearchRegistrationResults(long long target_allframe_id)
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

void CloseloopConstraintsBuilder::ExtractRegistrationResults(mapping::RegistrationResultsFilelist& registration_result_from, 
                                                             mapping::RegistrationResultsFilelist& registration_result_to,
                                                             int index)
{
    long long allframe_id_from = closeloop_constraints_.at(index).allframe_id_from;
    long long allframe_id_to = closeloop_constraints_.at(index).allframe_id_to;

    registration_result_from = SearchRegistrationResults(allframe_id_from);
    registration_result_to = SearchRegistrationResults(allframe_id_to);
}



// 计算相对位姿关系
void CloseloopConstraintsBuilder::ComputeConstraint()
{
    omp_set_num_threads(closeloop_constraints_builder_parameter_.num_threads);
#pragma omp parallel for
    for(int i=0; i<closeloop_constraints_.size(); i++)
	{
		// std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

		pcl::PointCloud<registration::PointType>::Ptr source(new pcl::PointCloud<registration::PointType>);
		pcl::PointCloud<registration::PointType>::Ptr target(new pcl::PointCloud<registration::PointType>);
		pcl::PointCloud<registration::PointType>::Ptr aligned(new pcl::PointCloud<registration::PointType>);
		pcl::PointCloud<registration::PointType>::Ptr transformed(new pcl::PointCloud<registration::PointType>);

        mapping::RegistrationResultsFilelist registration_result_from, registration_result_to;
        ExtractRegistrationResults(registration_result_from, 
                                   registration_result_to, 
                                   i);

        // 前面的节点是位姿点云是目标点云
        if(pcl::io::loadPCDFile<registration::PointType>(registration_result_from.pcd_file_path, *target) == -1)
		{
			PCL_ERROR("Couldn't load pcd file: ", registration_result_from.pcd_file_path);
		}

        // 后面的是源点云，将源点云匹配到目标点云上，源点云在目标点云下的相对位姿
        if(pcl::io::loadPCDFile<registration::PointType>(registration_result_to.pcd_file_path, *source) == -1)
		{
			PCL_ERROR("Couldn't load pcd file: ", registration_result_to.pcd_file_path);
		}

        FilterPointCloud(target);
        FilterPointCloud(source);


		// std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		// std::chrono::duration<double> time_cost = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
		// std::cout << "filter cost time is: " << "\033[32m" << time_cost.count() << "\033[0m" << std::endl;

        Eigen::Matrix4f transform_from = registration_result_from.alignment_result.final_transform;
        Eigen::Matrix4f transform_to = registration_result_to.alignment_result.final_transform;

		ICP_OMP icp_omp;
		icp_omp.setInputTarget(target);
		icp_omp.setInputSource(source);
		icp_omp.align(transform_from.inverse()*transform_to,
		              closeloop_constraints_builder_parameter_.max_constraints_distance_z);

		// Twv是匹配后的相对位姿
		Eigen::Matrix4f transform_relative = icp_omp.getFinalTransformation();
		// pcl::transformPointCloud(*source, *aligned, transform_relative);

        // std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
		// time_cost = std::chrono::duration_cast<std::chrono::duration<double>>(t3-t2);
		// std::cout << "align cost time is: " << "\033[32m" << time_cost.count() << "\033[0m" << std::endl;

		if(i < closeloop_constraints_.size()/8)
		{
			// printf("\033[1;32m  [%d/%d]--icp_omp fitness score:%.6f \033[0m \n", i, closeloop_constraints_.size()/8, icp_omp.getFitnessScore());
            std::cout << "\033[1;32m [" << i << "/" << closeloop_constraints_.size()/8 << "]" << "\033[0m" 
                      << "----icp_omp fitness score:" << "\033[1;32m" << icp_omp.getFitnessScore() <<  "\033[0m"
                      << std::endl;
		}
		

		if(icp_omp.getFitnessScore() < closeloop_constraints_builder_parameter_.min_score)
		{

			Eigen::Vector3f deltat = (transform_from.inverse()*transform_to).block<3,1>(0,3)
			                          - transform_relative.block<3,1>(0,3);

			if(sqrt(deltat(0)*deltat(0) + deltat(1)*deltat(1) + deltat(2)*deltat(2))==0)
			{

				continue;
			}

			double score = 1/icp_omp.getFitnessScore();
			Eigen::Matrix4f transform = icp_omp.getFinalTransformation();
			// Eigen::Vector3f t = transform.block<3,1>(0,3);
			// Eigen::Quaternionf q = Eigen::Quaternionf(transform.block<3,3>(0,0));
            closeloop_constraints_.at(i).score = score;
            closeloop_constraints_.at(i).relative_matrix = transform;
		}
	}    
}

// 删除匹配得分数不够的pairs
void CloseloopConstraintsBuilder::ErasePairs()
{
    for(std::vector<CloseloopConstraint>::iterator item = closeloop_constraints_.begin();
                                                   item != closeloop_constraints_.end();
                                                   item++)   
    {
        if(item->score == -1)
        {
            // erase函数的返回的是指向被删除元素的下一个元素的迭代器，所以执行erase（）后要把迭代器减1，指向前面一个
            item = closeloop_constraints_.erase(item);
            item--;
        }
    }
}

void CloseloopConstraintsBuilder::FilterPointCloud(pcl::PointCloud<registration::PointType>::Ptr pointcloud_ptr)
{
    pcl::PointCloud<registration::PointType>::Ptr pointcloud_filtered_ptr(new pcl::PointCloud<registration::PointType>());

    pcl::PassThrough<registration::PointType> pass_through;
	pcl::Filter<registration::PointType>::Ptr downsample_filter;
	boost::shared_ptr<pcl::VoxelGrid<registration::PointType>> voxel_grid(new pcl::VoxelGrid<registration::PointType>());

	voxel_grid->setLeafSize(0.1f, 0.1f, 0.1f);
	downsample_filter = voxel_grid;

    // 直通滤波器 PassThrough：实现对用户给定点云某个字段的限定下，对点云进行简单的基本过滤，例如限制过滤掉点云中所有 X 字段不在某个范围内的点
    pass_through.setInputCloud(pointcloud_ptr);
    pass_through.setFilterFieldName("z");

    // 滤出pcd点云中z轴坐标在 -2 ～ 50之间的点云
    pass_through.setFilterLimits(-2, 50);
    pass_through.filter(*pointcloud_filtered_ptr);

    *pointcloud_ptr = *pointcloud_filtered_ptr;


    downsample_filter->setInputCloud(pointcloud_ptr);
    downsample_filter->filter(*pointcloud_filtered_ptr);

    *pointcloud_ptr = *pointcloud_filtered_ptr;
}



}  // namespace constraints


