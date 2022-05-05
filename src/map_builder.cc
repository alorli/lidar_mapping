///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_23
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>
#include <fstream>
#include "src/map_builder.h"
#include "src/common/common.h"
#include "pcl/filters/voxel_grid.h"
#include "3rd_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <stdlib.h>   //system()系统调用函数头文件


namespace mapping
{

MapBuilder::MapBuilder(std::string cfg_file_path, 
                       std::string project_directory_name)
    :ndt_registration_(cfg_file_path, 0),
     ndt_registration_compensation_(cfg_file_path, 1),
     ekf_registration_(cfg_file_path),                       //added by lichunjing 2022-04-24
     motion_compensation_(0),
     laserscan_pose_interpolation_(cfg_file_path, project_directory_name),
     gnss_constraints_builder_(cfg_file_path, project_directory_name),
     closeloop_constraints_builder_(cfg_file_path, project_directory_name),
     gnss_optimization_(cfg_file_path, project_directory_name), 
     closeloop_optimization_(cfg_file_path, project_directory_name),
     to_grid_map_(cfg_file_path),
     map_partition_(cfg_file_path, project_directory_name),
     pointcloud_pap_arealist_(cfg_file_path, project_directory_name),
     is_save_vlp_raw_pcd_(false),
     pcd_map_previous_size_(0),
     allframe_id_(0),
     alignment_result_previous_({true, 0.0, 0.0, 0, Eigen::Matrix4f::Identity()})
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");

    msg_parameter_.pcd_map_voxel_leaf_size = cfg_file_["map_builder"]["msg_parameter"]["pcd_map_voxel_leaf_size"].as<double>();
    map_builder_parameter_.lidar_n_scans = cfg_file_["map_builder"]["map_builder_parameter"]["lidar_n_scans"].as<int>();

    vlp_mapping_parameter_.min_map_horizontal_radius = cfg_file_["map_builder"]["map_builder_parameter"]["vlp_mapping_parameter"]["min_map_horizontal_radius"].as<double>();
    vlp_mapping_parameter_.min_keyframe_distance = cfg_file_["map_builder"]["map_builder_parameter"]["vlp_mapping_parameter"]["min_keyframe_distance"].as<double>();
    vlp_mapping_parameter_.num_keyframe_submap = cfg_file_["map_builder"]["map_builder_parameter"]["vlp_mapping_parameter"]["num_keyframe_submap"].as<int>();
    vlp_mapping_parameter_.min_intensity = cfg_file_["map_builder"]["map_builder_parameter"]["vlp_mapping_parameter"]["min_intensity"].as<double>();
    vlp_mapping_parameter_.max_intensity = cfg_file_["map_builder"]["map_builder_parameter"]["vlp_mapping_parameter"]["max_intensity"].as<double>();

    sick_mapping_parameter_.min_map_horizontal_radius = cfg_file_["map_builder"]["map_builder_parameter"]["sick_mapping_parameter"]["min_map_horizontal_radius"].as<double>();
    sick_mapping_parameter_.min_keyframe_distance = cfg_file_["map_builder"]["map_builder_parameter"]["sick_mapping_parameter"]["min_keyframe_distance"].as<double>();
    sick_mapping_parameter_.num_keyframe_submap = cfg_file_["map_builder"]["map_builder_parameter"]["sick_mapping_parameter"]["num_keyframe_submap"].as<int>();
    sick_mapping_parameter_.min_intensity = cfg_file_["map_builder"]["map_builder_parameter"]["sick_mapping_parameter"]["min_intensity"].as<double>();
    sick_mapping_parameter_.max_intensity = cfg_file_["map_builder"]["map_builder_parameter"]["sick_mapping_parameter"]["max_intensity"].as<double>();
    sick_mapping_parameter_.z_offset = cfg_file_["map_builder"]["map_builder_parameter"]["sick_mapping_parameter"]["z_offset"].as<double>();
    

    dual_antenna_parameter_.is_use_dual_antenna = cfg_file_["map_builder"]["dual_antenna_parameter"]["is_use_dual_antenna"].as<bool>();
    dual_antenna_parameter_.ranges_main_antenna_to_lidar = cfg_file_["map_builder"]["dual_antenna_parameter"]["ranges_main_antenna_to_lidar"].as<double>();
    dual_antenna_parameter_.angle_dual_antenna_to_lidar = cfg_file_["map_builder"]["dual_antenna_parameter"]["angle_dual_antenna_to_lidar"].as<double>();

    // "beijing" 或者 "changsha" added by lichunjing 2021-08-12
    utm_localization_ = cfg_file_["map_builder"]["utm_localization"].as<std::string>();

    is_save_vlp_raw_pcd_ = cfg_file_["map_builder"]["save_vlp_raw_pcd"].as<bool>();
    is_save_sick_raw_pcd_ = cfg_file_["map_builder"]["save_sick_raw_pcd"].as<bool>();
    is_save_compensation_bag_= cfg_file_["map_builder"]["save_compensation_bag"].as<bool>();

    std::string project_directory = cfg_file_["directory"]["project_directory"].as<std::string>();
    std::string map_main_directory = cfg_file_["directory"]["map"]["main_directory"].as<std::string>();
    std::string velodyne_main_directory = cfg_file_["directory"]["velodyne"]["main_directory"].as<std::string>();
    std::string sick_main_directory = cfg_file_["directory"]["sick"]["main_directory"].as<std::string>();
    std::string gnss_main_directory = cfg_file_["directory"]["gnss"]["main_directory"].as<std::string>();
    std::string optimization_main_directory = cfg_file_["directory"]["optimization"]["main_directory"].as<std::string>();
    std::string compensation_bag_main_directory = cfg_file_["directory"]["compensation_bag"]["main_directory"].as<std::string>();
    std::string compensation_bag_file = cfg_file_["directory"]["compensation_bag"]["compensation_bag_file"].as<std::string>();
    std::string velodyne_raw_directory = cfg_file_["directory"]["velodyne"]["raw_directory"].as<std::string>();
    std::string velodyne_pcd_directory = cfg_file_["directory"]["velodyne"]["pcd_directory"].as<std::string>();
    std::string velodyne_compensation_directory = cfg_file_["directory"]["velodyne"]["compensation_directory"].as<std::string>();
    std::string sick_raw_directory = cfg_file_["directory"]["sick"]["raw_directory"].as<std::string>();
    std::string sick_pcd_directory = cfg_file_["directory"]["sick"]["pcd_directory"].as<std::string>();
    std::string sick_interpolation_results_filelist = cfg_file_["directory"]["sick"]["interpolation_results_filelist"].as<std::string>();
    std::string sick_compensation_directory = cfg_file_["directory"]["sick"]["compensation_directory"].as<std::string>();
    std::string velodyne_registration_results_filelist = cfg_file_["directory"]["velodyne"]["registration_results_filelist"].as<std::string>();
    std::string sick_registration_results_filelist = cfg_file_["directory"]["sick"]["registration_results_filelist"].as<std::string>();
    std::string gnss_fix_pose_file = cfg_file_["directory"]["gnss"]["gnss_fix_pose_file"].as<std::string>();
    std::string gnss_optimization_results_file = cfg_file_["directory"]["optimization"]["gnss_optimization_results_file"].as<std::string>();
    std::string closeloop_optimization_results_file = cfg_file_["directory"]["optimization"]["closeloop_optimization_results_file"].as<std::string>();
    std::string g2o_pose_file = cfg_file_["directory"]["velodyne"]["g2o_file"].as<std::string>();

    std::string raw_map_directory = cfg_file_["directory"]["map"]["raw_map"].as<std::string>();
    std::string compensation_map_directory = cfg_file_["directory"]["map"]["compensation_map"].as<std::string>();
    std::string gnss_optimization_map_directory = cfg_file_["directory"]["map"]["gnss_optimization_map"].as<std::string>();
    std::string closeloop_optimization_map_directory = cfg_file_["directory"]["map"]["closeloop_optimization_map"].as<std::string>();
    std::string laserscan_map_directory = cfg_file_["directory"]["map"]["laserscan_map_directory"].as<std::string>();
    std::string output_map_directory = cfg_file_["directory"]["map_partition"]["output_map_directory"].as<std::string>();
    save_map_full_directory_ = cfg_file_["directory"]["save_map_full_directory"].as<std::string>();

    project_directory_ = project_directory + project_directory_name;
    save_gnss_path_ = project_directory + project_directory_name + gnss_main_directory;
    save_velodyne_raw_pointcloud_path_ = project_directory + project_directory_name + velodyne_main_directory + velodyne_raw_directory + velodyne_pcd_directory;
    save_sick_raw_pointcloud_path_ = project_directory + project_directory_name + sick_main_directory + sick_raw_directory + sick_pcd_directory;
    save_velodyne_compensation_pointcloud_path_ = project_directory + project_directory_name + velodyne_main_directory + velodyne_compensation_directory + velodyne_pcd_directory;
    output_map_full_directory_ = project_directory + project_directory_name + map_main_directory + output_map_directory;
    save_compensation_bag_path_ = project_directory + project_directory_name + compensation_bag_main_directory;

    std::cout << std::endl;
    std::cout << "\033[32m" << "----------MapBuilder parameter:----------" << "\033[0m" << std::endl;
    std::cout << "msg_parameter_.pcd_map_voxel_leaf_size:"  << "\033[33m" << msg_parameter_.pcd_map_voxel_leaf_size  << "\033[0m" << std::endl;
    std::cout << "map_builder_parameter_.lidar_n_scans:" << "\033[33m"  << map_builder_parameter_.lidar_n_scans  << "\033[0m" << std::endl;
    std::cout << "vlp_mapping_parameter_.min_map_horizontal_radius:" << "\033[33m"  << vlp_mapping_parameter_.min_map_horizontal_radius  << "\033[0m" << std::endl;
    std::cout << "vlp_mapping_parameter_.min_keyframe_distance:" << "\033[33m"  << vlp_mapping_parameter_.min_keyframe_distance  << "\033[0m" << std::endl;
    std::cout << "vlp_mapping_parameter_.num_keyframe_submap:" << "\033[33m"  << vlp_mapping_parameter_.num_keyframe_submap  << "\033[0m" << std::endl;
    std::cout << "sick_mapping_parameter_.min_map_horizontal_radius:" << "\033[33m"  << sick_mapping_parameter_.min_map_horizontal_radius  << "\033[0m" << std::endl;
    std::cout << "sick_mapping_parameter_.min_keyframe_distance:" << "\033[33m"  << sick_mapping_parameter_.min_keyframe_distance  << "\033[0m" << std::endl;
    std::cout << "sick_mapping_parameter_.num_keyframe_submap:" << "\033[33m"  << sick_mapping_parameter_.num_keyframe_submap  << "\033[0m" << std::endl;
    std::cout << "save_velodyne_compensation_pointcloud_path_:" << "\033[33m"  << save_velodyne_compensation_pointcloud_path_  << "\033[0m" << std::endl;
    std::cout << "save_velodyne_raw_pointcloud_path_:" << "\033[33m"  << save_velodyne_raw_pointcloud_path_  << "\033[0m" << std::endl;
    std::cout << "save_gnss_path_:" << "\033[33m"  << save_gnss_path_  << "\033[0m" << std::endl;

    // 创建工程主目录
    if(!stlplus::folder_exists(project_directory))
	{
		stlplus::folder_create(project_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory << "\033[0m" << std::endl;
	}

    // 创建工程目录名
    if(!stlplus::folder_exists(project_directory+project_directory_name))
	{
		stlplus::folder_create(project_directory+project_directory_name);
        std::cout << "\033[34m" << "create directory:" << project_directory+project_directory_name << "\033[0m" << std::endl;
	}

    project_directory = project_directory + project_directory_name;


    // 创建 velodyne 目录
    if(!stlplus::folder_exists(project_directory + velodyne_main_directory))
	{
		stlplus::folder_create(project_directory + velodyne_main_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + velodyne_main_directory << "\033[0m" << std::endl;
	}

    // 创建 velodyne/raw 目录
    if(!stlplus::folder_exists(project_directory + velodyne_main_directory + velodyne_raw_directory))
	{
		stlplus::folder_create(project_directory + velodyne_main_directory + velodyne_raw_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + velodyne_main_directory + velodyne_raw_directory << "\033[0m" << std::endl;
	}

    // 创建 velodyne/compensation 目录
    if(!stlplus::folder_exists(project_directory + velodyne_main_directory + velodyne_compensation_directory))
	{
		stlplus::folder_create(project_directory + velodyne_main_directory + velodyne_compensation_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + velodyne_main_directory + velodyne_compensation_directory << "\033[0m" << std::endl;
	}

    // 创建 velodyne/raw/pcd 目录
    if(!stlplus::folder_exists(project_directory + velodyne_main_directory + velodyne_raw_directory + velodyne_pcd_directory))
	{
		stlplus::folder_create(project_directory + velodyne_main_directory + velodyne_raw_directory + velodyne_pcd_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + velodyne_main_directory + velodyne_raw_directory + velodyne_pcd_directory << "\033[0m" << std::endl;
	}

    // 创建 velodyne/compensation/pcd 目录
    if(!stlplus::folder_exists(project_directory + velodyne_main_directory + velodyne_compensation_directory + velodyne_pcd_directory))
	{
		stlplus::folder_create(project_directory + velodyne_main_directory + velodyne_compensation_directory + velodyne_pcd_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + velodyne_main_directory + velodyne_compensation_directory + velodyne_pcd_directory << "\033[0m" << std::endl;
	}

    // 创建 sick 目录
    if(!stlplus::folder_exists(project_directory + sick_main_directory))
	{
		stlplus::folder_create(project_directory + sick_main_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + sick_main_directory << "\033[0m" << std::endl;
	}

    // 创建 sick/raw 目录
    if(!stlplus::folder_exists(project_directory + sick_main_directory + sick_raw_directory))
	{
		stlplus::folder_create(project_directory + sick_main_directory + sick_raw_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + sick_main_directory + sick_raw_directory << "\033[0m" << std::endl;
	}

    // 创建 sick/compensation 目录
    if(!stlplus::folder_exists(project_directory + sick_main_directory + sick_compensation_directory))
	{
		stlplus::folder_create(project_directory + sick_main_directory + sick_compensation_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + sick_main_directory + sick_compensation_directory << "\033[0m" << std::endl;
	}

    // 创建 sick/raw/pcd 目录
    if(!stlplus::folder_exists(project_directory + sick_main_directory + sick_raw_directory + sick_pcd_directory))
	{
		stlplus::folder_create(project_directory + sick_main_directory + sick_raw_directory + sick_pcd_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + sick_main_directory + sick_raw_directory + sick_pcd_directory << "\033[0m" << std::endl;
	}

    // 创建 sick/compensation/pcd 目录
    if(!stlplus::folder_exists(project_directory + sick_main_directory + sick_compensation_directory + sick_pcd_directory))
	{
		stlplus::folder_create(project_directory + sick_main_directory + sick_compensation_directory + sick_pcd_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + sick_main_directory + sick_compensation_directory + sick_pcd_directory << "\033[0m" << std::endl;
	}

    // 创建 gnss 目录
    if(!stlplus::folder_exists(project_directory + gnss_main_directory))
	{
		stlplus::folder_create(project_directory + gnss_main_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + gnss_main_directory << "\033[0m" << std::endl;
	}

    // 创建 optimization 目录
    if(!stlplus::folder_exists(project_directory + optimization_main_directory))
	{
		stlplus::folder_create(project_directory + optimization_main_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + optimization_main_directory << "\033[0m" << std::endl;
	}

    // 创建 map 目录
    if(!stlplus::folder_exists(project_directory + map_main_directory))
	{
		stlplus::folder_create(project_directory + map_main_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + map_main_directory << "\033[0m" << std::endl;
	}

    // 创建 map/raw_map 目录
    if(!stlplus::folder_exists(project_directory + map_main_directory + raw_map_directory))
	{
		stlplus::folder_create(project_directory + map_main_directory + raw_map_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + map_main_directory + raw_map_directory << "\033[0m" << std::endl;
	}

    // 创建 map/compensation_map 目录
    if(!stlplus::folder_exists(project_directory + map_main_directory + compensation_map_directory))
	{
		stlplus::folder_create(project_directory + map_main_directory + compensation_map_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + map_main_directory + compensation_map_directory << "\033[0m" << std::endl;
	}

    // 创建 map/gnss_optimization_map 目录
    if(!stlplus::folder_exists(project_directory + map_main_directory + gnss_optimization_map_directory))
	{
		stlplus::folder_create(project_directory + map_main_directory + gnss_optimization_map_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + map_main_directory + gnss_optimization_map_directory << "\033[0m" << std::endl;
	}

    // 创建 map/closeloop_optimization_map 目录
    if(!stlplus::folder_exists(project_directory + map_main_directory + closeloop_optimization_map_directory))
	{
		stlplus::folder_create(project_directory + map_main_directory + closeloop_optimization_map_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + map_main_directory + closeloop_optimization_map_directory << "\033[0m" << std::endl;
	}

    // 创建 map/closeloop_optimization_map 目录
    if(!stlplus::folder_exists(project_directory + map_main_directory + laserscan_map_directory))
	{
		stlplus::folder_create(project_directory + map_main_directory + laserscan_map_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + map_main_directory + laserscan_map_directory << "\033[0m" << std::endl;
	}

    // 创建 map/output_map 目录
    if(!stlplus::folder_exists(project_directory + map_main_directory + output_map_directory))
	{
		stlplus::folder_create(project_directory + map_main_directory + output_map_directory);
        std::cout << "\033[34m" << "create directory:" << project_directory + map_main_directory + output_map_directory << "\033[0m" << std::endl;
	}

    // 创建 compensation_bag 目录
    if(!stlplus::folder_exists(save_compensation_bag_path_))
	{
		stlplus::folder_create(save_compensation_bag_path_);
        std::cout << "\033[34m" << "create directory:" << save_compensation_bag_path_ << "\033[0m" << std::endl;
	}


    // added by lichunjing 2021-08-12
    if(utm_localization_ == "beijing")
    {
        if(!(pj_utm_ = pj_init_plus("+proj=utm +zone=50 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs")))     //北京
        {
            exit(1);
        }
    }

    // added by lichunjing 2021-08-12
    if(utm_localization_ == "changsha")
    {
        if(!(pj_utm_ = pj_init_plus("+proj=utm +zone=49 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs")))        //长沙
        {
            exit(1);
        }
    }

    if(!(pj_latlong_ = pj_init_plus("+proj=longlat +ellps=WGS84")))
    {
        exit(1);
    }

    motion_compensation_.SetLidarNScans(map_builder_parameter_.lidar_n_scans);


    map_generator_raw_.SetPathName(save_velodyne_raw_pointcloud_path_ + "../" + velodyne_registration_results_filelist,
                                   project_directory + map_main_directory + raw_map_directory);

    map_generator_raw_.SetParameter(vlp_mapping_parameter_.min_keyframe_distance,
                                    vlp_mapping_parameter_.min_map_horizontal_radius,
                                    vlp_mapping_parameter_.num_keyframe_submap,
                                    vlp_mapping_parameter_.min_intensity,
                                    vlp_mapping_parameter_.max_intensity
                                    );

    map_generator_compensation_.SetPathName(save_velodyne_compensation_pointcloud_path_ + "../" + velodyne_registration_results_filelist,
                                            project_directory + map_main_directory + compensation_map_directory);
    map_generator_compensation_.SetParameter(vlp_mapping_parameter_.min_keyframe_distance,
                                             vlp_mapping_parameter_.min_map_horizontal_radius,
                                             vlp_mapping_parameter_.num_keyframe_submap,
                                             vlp_mapping_parameter_.min_intensity,
                                             vlp_mapping_parameter_.max_intensity
                                            );


    map_generator_gnss_optimization_.SetPathName(project_directory + optimization_main_directory + gnss_optimization_results_file,
                                                 project_directory + map_main_directory + gnss_optimization_map_directory);
    map_generator_gnss_optimization_.SetParameter(vlp_mapping_parameter_.min_keyframe_distance,
                                                  vlp_mapping_parameter_.min_map_horizontal_radius,
                                                  vlp_mapping_parameter_.num_keyframe_submap,
                                                  vlp_mapping_parameter_.min_intensity,
                                                  vlp_mapping_parameter_.max_intensity
                                                 );

    map_generator_closeloop_optimization_.SetPathName(project_directory + optimization_main_directory + closeloop_optimization_results_file,
                                                      project_directory + map_main_directory + closeloop_optimization_map_directory);
    map_generator_closeloop_optimization_.SetParameter(vlp_mapping_parameter_.min_keyframe_distance,
                                                       vlp_mapping_parameter_.min_map_horizontal_radius,
                                                       vlp_mapping_parameter_.num_keyframe_submap,
                                                       vlp_mapping_parameter_.min_intensity,
                                                       vlp_mapping_parameter_.max_intensity
                                                      );

    map_generator_laserscan_.SetPathName(project_directory + sick_main_directory + sick_raw_directory + sick_interpolation_results_filelist,
                                         project_directory + map_main_directory + laserscan_map_directory);
    map_generator_laserscan_.SetParameter(sick_mapping_parameter_.min_keyframe_distance,
                                          sick_mapping_parameter_.min_map_horizontal_radius,
                                          sick_mapping_parameter_.num_keyframe_submap,
                                          sick_mapping_parameter_.min_intensity,
                                          sick_mapping_parameter_.max_intensity
                                         );

    map_generator_laserscan_.SetZOffset(sick_mapping_parameter_.z_offset);

    velodyne_compensation_filelist_.open(save_velodyne_compensation_pointcloud_path_ + "../" + velodyne_registration_results_filelist,
                                         std::ios::app);

    velodyne_raw_filelist_.open(save_velodyne_raw_pointcloud_path_ + "../" + velodyne_registration_results_filelist,
                                std::ios::app);

    sick_raw_filelist_.open(save_sick_raw_pointcloud_path_ + "../" + sick_registration_results_filelist,
                                std::ios::app);


    gnss_fix_pose_file_.open(save_gnss_path_ + gnss_fix_pose_file,
                             std::ios::app);

    g2o_file_compensation_.open(save_velodyne_compensation_pointcloud_path_ + "../" + g2o_pose_file,
                                std::ios::app);

    g2o_file_raw_.open(save_velodyne_raw_pointcloud_path_ + "../" + g2o_pose_file,
                                std::ios::app);

    if(is_save_compensation_bag_)
    {
        compensation_bag_.open(save_compensation_bag_path_ + compensation_bag_file, rosbag::bagmode::Write);
    }
}

MapBuilder::~MapBuilder()
{
    velodyne_compensation_filelist_.close();
    velodyne_raw_filelist_.close();
    sick_raw_filelist_.close();
    gnss_fix_pose_file_.close();
    g2o_file_compensation_.close();
    g2o_file_raw_.close();

    if(is_save_compensation_bag_)
    {
        compensation_bag_.close();
    }
}

/*
void MapBuilder::AddVlpPointCloudData(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    registration::TimedIdPointCloud timed_id_pointcloud;
    pcl::fromROSMsg(*msg, timed_id_pointcloud.pointcloud);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(timed_id_pointcloud.pointcloud,
                                 timed_id_pointcloud.pointcloud,
                                 indices);

    timed_id_pointcloud.time = common::FromRos(msg->header.stamp);
    timed_id_pointcloud.allframe_id = allframe_id_;

    // added by lichunjing 2021-04-14
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // registration::TimedIdPointCloud timed_id_pointcloud_compensation;
    // timed_id_pointcloud_compensation.time = timed_id_pointcloud.time;
    // timed_id_pointcloud_compensation.allframe_id = timed_id_pointcloud.allframe_id;

    // if(!motion_compensation_.CompensationUsingPoseExtrapolater(timed_id_pointcloud.time,
    //                                                            timed_id_pointcloud.pointcloud,
    //                                                            timed_id_pointcloud_compensation.pointcloud,
    //                                                            extrapolator_ptr_.get()
    //                                                            ))
    // {
    //     return;
    // }

    // transform::Rigid3f pose_predict = extrapolator_ptr_->ExtrapolatePose(timed_id_pointcloud_compensation.time).cast<float>();
    // Eigen::Matrix4f  predict_matrix = Eigen::Matrix4f::Identity();
    // predict_matrix.block<3,1>(0,3) = pose_predict.translation();
    // predict_matrix.block<3,3>(0,0) = pose_predict.rotation().toRotationMatrix();

    // // 调用时提供预测位姿
    // ndt_registration_.AddSensorData(timed_id_pointcloud_compensation, predict_matrix);

    // registration::AlignmentResult alignment_result = ndt_registration_.GetAlignmentResult();

    // Eigen::Vector3d translation_alignment_result = alignment_result.final_transform.block<3,1>(0,3).cast<double>();
    // Eigen::Quaterniond rotation_alignment_result = Eigen::Quaterniond(alignment_result.final_transform.block<3,3>(0,0).cast<double>());

    // transform::Rigid3d pose_estimate = transform::Rigid3d(translation_alignment_result,
    //                                                       rotation_alignment_result
    //                                                       );
    // extrapolator_ptr_->AddPose(timed_id_pointcloud_compensation.time, pose_estimate);
    // // end ----added by lichunjing 2021-04-14
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if(is_save_vlp_raw_pcd_)
    {
        SaveVlpRawPcd(timed_id_pointcloud);

        // added by lichunjing 2021-04-14
        // SaveVlpRawPcd(timed_id_pointcloud_compensation);
    }


    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ndt_registration_.AddSensorData(timed_id_pointcloud);

    if(allframe_id_ >= 1)
    {
        // registration::TimedIdPointCloud timed_id_pointcloud_compensation;
        // timed_id_pointcloud_compensation.time = timed_id_pointcloud_previous_.time;
        // timed_id_pointcloud_compensation.allframe_id = timed_id_pointcloud_previous_.allframe_id;
        // motion_compensation_.LinearInterpolation(timed_id_pointcloud_previous_.pointcloud,
        //                                          timed_id_pointcloud_compensation.pointcloud,
        //                                          alignment_result_previous_.final_transform,
        //                                          ndt_registration_.GetAlignmentResult().final_transform
        //                                         );

        // added by lichunjing 2021-08-12
        registration::TimedIdPointCloud timed_id_pointcloud_compensation;
        timed_id_pointcloud_compensation.time = timed_id_pointcloud.time;
        timed_id_pointcloud_compensation.allframe_id = timed_id_pointcloud.allframe_id;
        motion_compensation_.LinearInterpolation(timed_id_pointcloud.pointcloud,
                                                 timed_id_pointcloud_compensation.pointcloud,
                                                 alignment_result_previous_.final_transform,
                                                 ndt_registration_.GetAlignmentResult().final_transform
                                                );


        // 这里不加预测位姿，实际使用的是线性递推得到的位姿作为预测位姿
        ndt_registration_compensation_.AddSensorData(timed_id_pointcloud_compensation);

        SaveVlpCompensationPcd(timed_id_pointcloud_compensation);

        // added by lichunjing 2022-03-22
        if(is_save_compensation_bag_)
        {
            sensor_msgs::PointCloud2 msg_pointcloud_compensation;
            pcl::toROSMsg(timed_id_pointcloud_compensation.pointcloud, msg_pointcloud_compensation);
            msg_pointcloud_compensation.header.stamp = common::ToRos(timed_id_pointcloud_compensation.time);

            compensation_bag_.write("/points_raw", msg_pointcloud_compensation.header.stamp, msg_pointcloud_compensation);
        }
    }
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    allframe_id_++;
    alignment_result_previous_ = ndt_registration_.GetAlignmentResult();
    timed_id_pointcloud_previous_ = timed_id_pointcloud;
}
*/

void MapBuilder::AddVlpPointCloudData(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // registration::TimedIdPointCloud timed_id_pointcloud;
    // pcl::fromROSMsg(*msg, timed_id_pointcloud.pointcloud);
// 
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(timed_id_pointcloud.pointcloud,
                                //  timed_id_pointcloud.pointcloud,
                                //  indices);
// 
    // timed_id_pointcloud.time = common::FromRos(msg->header.stamp);
    // timed_id_pointcloud.allframe_id = allframe_id_;
// 
    ekf_registration_.AddSensorData(msg);
}


void MapBuilder::AddGnssFixData(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    if(is_save_compensation_bag_)
    {   
        compensation_bag_.write("/gps/fix", msg->header.stamp, *msg);
    }
    
    sensor_msgs::NavSatFix gnss_data = *msg;

    static long long msg_id = 0;
    double utm_x, utm_y, utm_z;

    utm_x = common::degree_to_radian(gnss_data.longitude);
    utm_y = common::degree_to_radian(gnss_data.latitude);

    // 高程直接取自gga报文中的高程
    utm_z = gnss_data.altitude;

    pj_transform(pj_latlong_, pj_utm_, 1, 1, &utm_x, &utm_y, NULL);

    common::Time time_current = common::FromRos(msg->header.stamp);

    // common::Time time_current = common::FromRosAddOffset(msg->header.stamp, -0.045);

    // 配置是否使用双天线建图
    if(dual_antenna_parameter_.is_use_dual_antenna)
    {
        double heading_dual_antenna = gnss_data.position_covariance[1]; 

        double det_angle = (heading_dual_antenna - dual_antenna_parameter_.angle_dual_antenna_to_lidar)/180.0*M_PI;
        utm_x += dual_antenna_parameter_.ranges_main_antenna_to_lidar*sin(det_angle);
        utm_y += dual_antenna_parameter_.ranges_main_antenna_to_lidar*cos(det_angle);
    }



    // std::cout << "\033[32m"
    //           << std::setprecision(15)
    //           << "  utm_x:" << utm_x
    //           << "  utm_y:" << utm_y
    //           << "  utm_z:" << utm_z
    //           << "\033[0m"
    //           << std::endl;

    gnss_fix_pose_file_ << std::setprecision(15)
                        << msg_id++
                        << " " << time_current
                        << " " << (int)gnss_data.status.status
                        << " " << gnss_data.status.service
                        << " " << utm_x
                        << " " << utm_y
                        << " " << utm_z
                        << std::endl;
}



void MapBuilder::AddSickData(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if(!is_save_sick_raw_pcd_)
    {
        return;
    }

    static long long sick_msg_cnt = 0;
    registration::TimedIdPointCloud timed_id_pointcloud;

    timed_id_pointcloud.time = common::FromRos(msg->header.stamp);
    timed_id_pointcloud.allframe_id = sick_msg_cnt++;

    registration::PointType new_point;
    new_point.z = 0.0;
    double new_point_angle;

    int beam_num = msg->ranges.size();
    for(int i = 0; i < beam_num; i++)
    {
        new_point_angle = msg->angle_min + msg->angle_increment * i;
        new_point.x = msg->ranges[i] * cos(new_point_angle);
        new_point.y = msg->ranges[i] * sin(new_point_angle);
        new_point.intensity = msg->intensities[i];
        timed_id_pointcloud.pointcloud.push_back(new_point);
    }

	std::stringstream file_name;
    file_name << save_sick_raw_pointcloud_path_ << timed_id_pointcloud.allframe_id << ".pcd";

	pcl::io::savePCDFileBinaryCompressed(file_name.str().c_str(),
										 timed_id_pointcloud.pointcloud);


	registration::AlignmentResult alignment_result_raw;
    alignment_result_raw.is_converged = 0;
    alignment_result_raw.fitness_score = 0;
    alignment_result_raw.time_duration_ms = 0;
    alignment_result_raw.final_num_iteration = 0;
    alignment_result_raw.final_transform = Eigen::Matrix4f::Identity();

    sick_raw_filelist_ << std::setprecision(15)
                       << timed_id_pointcloud.allframe_id
                       << " " << timed_id_pointcloud.time
                       << " " << 0
                       << " " << file_name.str().c_str()
                       << " " << alignment_result_raw.is_converged
                       << " " << alignment_result_raw.fitness_score
                       << " " << alignment_result_raw.time_duration_ms
                       << " " << alignment_result_raw.final_num_iteration
                       << " " << alignment_result_raw.final_transform(0,0) << " " << alignment_result_raw.final_transform(0,1) << " " << alignment_result_raw.final_transform(0,2) << " " << alignment_result_raw.final_transform(0,3)
                       << " " << alignment_result_raw.final_transform(1,0) << " " << alignment_result_raw.final_transform(1,1) << " " << alignment_result_raw.final_transform(1,2) << " " << alignment_result_raw.final_transform(1,3)
                       << " " << alignment_result_raw.final_transform(2,0) << " " << alignment_result_raw.final_transform(2,1) << " " << alignment_result_raw.final_transform(2,2) << " " << alignment_result_raw.final_transform(2,3)
                       << " " << alignment_result_raw.final_transform(3,0) << " " << alignment_result_raw.final_transform(3,1) << " " << alignment_result_raw.final_transform(3,2) << " " << alignment_result_raw.final_transform(3,3)
                       << std::endl;
}


/*
void MapBuilder::AddSensorData(const sensor::ImuData& imu_data)
{
    if(extrapolator_ptr_ != nullptr)
    {
        extrapolator_ptr_->AddImuData(imu_data);
        return;
    }

    constexpr double kExtrapolationEstimationTimeSec = 0.001;

    extrapolator_ptr_ = processing::PoseExtrapolator::InitializeWithImu(
        common::FromSeconds(kExtrapolationEstimationTimeSec),
        9.8,
        imu_data);
}
*/

void MapBuilder::AddSensorData(const sensor::ImuData& imu_data)
{
    ekf_registration_.AddSensorData(imu_data);
}



void MapBuilder::AddSensorData(const sensor::OdometryData& odometry_data)
{
  if(extrapolator_ptr_ == nullptr)
  {
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }

  extrapolator_ptr_->AddOdometryData(odometry_data);
}





void MapBuilder::GnssAidedOptimization()
{
    gnss_constraints_builder_.BuildConstraints();

    gnss_optimization_.RunOptimization(gnss_constraints_builder_.GetLidarOdomPoses(),
                                       gnss_constraints_builder_.GetGnssPoseConstraints());
}

void MapBuilder::CloseloopOptimization()
{
    closeloop_constraints_builder_.BuildConstraints();

    closeloop_optimization_.RunOptimization(closeloop_constraints_builder_.GetLidarOdomPoses(),
                                            closeloop_constraints_builder_.GetCloseloopConstraints());
}

void MapBuilder::SaveVlpRawPcd(registration::TimedIdPointCloud& timed_id_pointcloud)
{
	std::stringstream file_name;
    file_name << save_velodyne_raw_pointcloud_path_ << timed_id_pointcloud.allframe_id << ".pcd";

	pcl::io::savePCDFileBinaryCompressed(file_name.str().c_str(),
										 timed_id_pointcloud.pointcloud);

	registration::AlignmentResult alignment_result_raw
        = ndt_registration_.GetAlignmentResult();

    velodyne_raw_filelist_ << std::setprecision(15)
                           << timed_id_pointcloud.allframe_id
                           << " " << timed_id_pointcloud.time
                           << " " << 0
                           << " " << file_name.str().c_str()
                           << " " << alignment_result_raw.is_converged
                           << " " << alignment_result_raw.fitness_score
                           << " " << alignment_result_raw.time_duration_ms
                           << " " << alignment_result_raw.final_num_iteration
                           << " " << alignment_result_raw.final_transform(0,0) << " " << alignment_result_raw.final_transform(0,1) << " " << alignment_result_raw.final_transform(0,2) << " " << alignment_result_raw.final_transform(0,3)
                           << " " << alignment_result_raw.final_transform(1,0) << " " << alignment_result_raw.final_transform(1,1) << " " << alignment_result_raw.final_transform(1,2) << " " << alignment_result_raw.final_transform(1,3)
                           << " " << alignment_result_raw.final_transform(2,0) << " " << alignment_result_raw.final_transform(2,1) << " " << alignment_result_raw.final_transform(2,2) << " " << alignment_result_raw.final_transform(2,3)
                           << " " << alignment_result_raw.final_transform(3,0) << " " << alignment_result_raw.final_transform(3,1) << " " << alignment_result_raw.final_transform(3,2) << " " << alignment_result_raw.final_transform(3,3)
                           << std::endl;


    Eigen::Matrix4f transform = alignment_result_raw.final_transform;
    Eigen::Vector3f translation = transform.block<3,1>(0,3);
    Eigen::Quaternionf quat = Eigen::Quaternionf(transform.block<3,3>(0,0));

    g2o_file_raw_ << "VERTEX_SE3:QUAT" << " "
                  << timed_id_pointcloud.allframe_id << " "
                  << translation(0) << " " << translation(1) << " " << translation(2) << " "
                  << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " "
                  << std::endl;
}

void MapBuilder::SaveVlpCompensationPcd(registration::TimedIdPointCloud& timed_id_pointcloud_compensation)
{
	std::stringstream file_name;
    file_name << save_velodyne_compensation_pointcloud_path_ << timed_id_pointcloud_compensation.allframe_id << ".pcd";

	pcl::io::savePCDFileBinaryCompressed(file_name.str().c_str(),
										 timed_id_pointcloud_compensation.pointcloud);

	registration::AlignmentResult alignment_result_compensation
        = ndt_registration_compensation_.GetAlignmentResult();

    velodyne_compensation_filelist_ << std::setprecision(15)
                                    << timed_id_pointcloud_compensation.allframe_id
                                    << " " << timed_id_pointcloud_compensation.time
                                    << " " << 0
                                    << " " << file_name.str().c_str()
                                    << " " << alignment_result_compensation.is_converged
                                    << " " << alignment_result_compensation.fitness_score
                                    << " " << alignment_result_compensation.time_duration_ms
                                    << " " << alignment_result_compensation.final_num_iteration
                                    << " " << alignment_result_compensation.final_transform(0,0) << " " << alignment_result_compensation.final_transform(0,1) << " " << alignment_result_compensation.final_transform(0,2) << " " << alignment_result_compensation.final_transform(0,3)
                                    << " " << alignment_result_compensation.final_transform(1,0) << " " << alignment_result_compensation.final_transform(1,1) << " " << alignment_result_compensation.final_transform(1,2) << " " << alignment_result_compensation.final_transform(1,3)
                                    << " " << alignment_result_compensation.final_transform(2,0) << " " << alignment_result_compensation.final_transform(2,1) << " " << alignment_result_compensation.final_transform(2,2) << " " << alignment_result_compensation.final_transform(2,3)
                                    << " " << alignment_result_compensation.final_transform(3,0) << " " << alignment_result_compensation.final_transform(3,1) << " " << alignment_result_compensation.final_transform(3,2) << " " << alignment_result_compensation.final_transform(3,3)
                                    << std::endl;


    Eigen::Matrix4f transform = alignment_result_compensation.final_transform;
    Eigen::Vector3f translation = transform.block<3,1>(0,3);
    Eigen::Quaternionf quat = Eigen::Quaternionf(transform.block<3,3>(0,0));

    g2o_file_compensation_ << "VERTEX_SE3:QUAT" << " "
                  << timed_id_pointcloud_compensation.allframe_id << " "
                  << translation(0) << " " << translation(1) << " " << translation(2) << " "
                  << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " "
                  << std::endl;
}

void MapBuilder::GenerateCompensationMap()
{
    map_generator_compensation_.GeneratePointCloudMap();
}

void MapBuilder::GenerateRawMap()
{
    map_generator_raw_.GeneratePointCloudMap();
}

void MapBuilder::GenerateGnssOptimizationMap()
{
    map_generator_gnss_optimization_.GeneratePointCloudMap();
}


void MapBuilder::GenerateCloseloopOptimizationMap()
{
    map_generator_closeloop_optimization_.GeneratePointCloudMap();
    // map_generator_closeloop_optimization_.GeneratePointCloudRGBAMap();

}

void MapBuilder::GenerateLaserScanMap()
{
    laserscan_pose_interpolation_.RunIterpolation();
    // map_generator_laserscan_.GeneratePointCloudMap();

    // map_generator_laserscan_.GeneratePointCloudRGBAMap();

    // added by lichunjing 2021-05-19
    map_generator_laserscan_.GeneratePointCloudRGBMap();

}


void MapBuilder::GetPcdMapMsg(sensor_msgs::PointCloud2& pcd_map_msg)
{
    pcl::PointCloud<registration::PointType>::ConstPtr pcd_map_ptr
        = ndt_registration_.GetMapConstPtr();

    if(pcd_map_ptr->points.size() != pcd_map_previous_size_)
    {
        pcd_map_previous_size_ = pcd_map_ptr->points.size();
        pcl::PointCloud<registration::PointType> pcd_map_filtered;
        pcl::VoxelGrid<registration::PointType> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(msg_parameter_.pcd_map_voxel_leaf_size,
                                      msg_parameter_.pcd_map_voxel_leaf_size,
                                      msg_parameter_.pcd_map_voxel_leaf_size);
        voxel_grid_filter.setInputCloud(pcd_map_ptr);
        voxel_grid_filter.filter(pcd_map_filtered);
        std::cout << "Original: " << pcd_map_ptr->points.size() << " points." << std::endl;
        std::cout << "Filtered: " << pcd_map_filtered.points.size() << " points." << std::endl;

        pcl::toROSMsg(pcd_map_filtered, pcd_map_msg);
        pcd_map_msg.header.stamp = ros::Time::now();
        pcd_map_msg.header.frame_id = "map";
    }
}

void MapBuilder::RunMapPartition()
{
    map_partition_.RunMapPartition();
}

void MapBuilder::GenerateArealistFile()
{
    pointcloud_pap_arealist_.GenerateArealistFile();
}

std::vector<geometry_msgs::TransformStamped> MapBuilder::GetTfMsg()
{
    std::vector<geometry_msgs::TransformStamped> stamped_transforms;
    geometry_msgs::TransformStamped stamped_transform;
    geometry_msgs::Transform transform;
    registration::AlignmentResult alignment_result_current
        = ndt_registration_.GetAlignmentResult();

    stamped_transform.header.stamp = ros::Time::now();
    stamped_transform.header.frame_id = "/map";
    stamped_transform.child_frame_id = "/velodyne";

    transform.translation.x = alignment_result_current.final_transform(0,3);
    transform.translation.y = alignment_result_current.final_transform(1,3);
    transform.translation.z = alignment_result_current.final_transform(2,3);

    Eigen::Quaternionf quat(alignment_result_current.final_transform.block<3,3>(0, 0));

    transform.rotation.w = quat.w();
    transform.rotation.x = quat.x();
    transform.rotation.y = quat.y();
    transform.rotation.z = quat.z();

    stamped_transform.transform = transform;
    stamped_transforms.push_back(stamped_transform);

    return stamped_transforms;
}

bool MapBuilder::GetOccupancyGridMsg(nav_msgs::OccupancyGrid& occupancy_grid_map_msg,
                                     double update_distance)
{
    registration::AlignmentResult alignment_result_current
        = ndt_registration_.GetAlignmentResult();

    static Eigen::Vector3f previous_translation = alignment_result_current.final_transform.block<3,1>(0, 3);
    Eigen::Vector3f current_translation = alignment_result_current.final_transform.block<3,1>(0, 3);
    double delta_distance = sqrt((previous_translation[0]-current_translation[0])*(previous_translation[0]-current_translation[0])
                                +(previous_translation[1]-current_translation[1])*(previous_translation[1]-current_translation[1])
                                +(previous_translation[2]-current_translation[2])*(previous_translation[2]-current_translation[2]));

    if(delta_distance > update_distance)
    {
        previous_translation = current_translation;

        pcl::PointCloud<pcl::PointXYZI> pcd_map = *ndt_registration_.GetMapConstPtr();

        to_grid_map_.PointCloudToOccupancyGridMsg(pcd_map, occupancy_grid_map_msg);

        return true;
    }

    return false;
}



bool MapBuilder::GetOccupancyGridMsg(nav_msgs::OccupancyGrid& occupancy_grid_map_msg)
{
    pcl::PointCloud<pcl::PointXYZI> pointcloud_map;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());

    std::vector<std::string> pointcloud_file_list = common::read_file_list(output_map_full_directory_.data());

    // 读入目录中所有点云文件
    for(int i = 0; i < pointcloud_file_list.size(); i++)
    {
        // Loading input_cloud.
        std::string file_name = pointcloud_file_list.at(i);

        std::cout << "output_map_full_directory_ + file_name:" << output_map_full_directory_ + file_name << std::endl;

        if(file_name.find(".pcd") == -1)
        {
            continue;
        }

        if(pcl::io::loadPCDFile<pcl::PointXYZI>(output_map_full_directory_ + file_name,
                                                *cloud_ptr) == -1)          //* load the file
        {
            PCL_ERROR("Couldn't read file: \n",  output_map_full_directory_ + file_name);
        }

        pointcloud_map += *cloud_ptr;
	}    


    to_grid_map_.PointCloudToOccupancyGridMsg(pointcloud_map, occupancy_grid_map_msg);

    return true;
}

bool MapBuilder::SaveMap(std::string map_name)
{
    // 创建 save_map 地图存放目录
    if(!stlplus::folder_exists(save_map_full_directory_ + map_name))
	{
		stlplus::folder_create(save_map_full_directory_ + map_name);
        std::cout << "\033[34m" << "create directory:" << save_map_full_directory_ + map_name << "\033[0m" << std::endl;
	}

    std::string copy_map_cmd = "cp " + output_map_full_directory_ + "* " + save_map_full_directory_ + map_name;

    // std::cout << "-------------copy_map_cmd:" << copy_map_cmd << std::endl;

    if(system(copy_map_cmd.c_str()) != 0)
    {
        std::cout << "can't copy map to " << save_map_full_directory_ + map_name << std::endl;
        return false;
    }

    return true;
}

void MapBuilder::DeleteProjectDirectory()
{
    std::string delete_project_directory_cmd = "rm -r " + project_directory_;

    std::cout << "--------------delete_project_directory_cmd:" << delete_project_directory_cmd << std::endl;

    if(system(delete_project_directory_cmd.c_str()) != 0)
    {
        std::cout << "delete project directory:" << project_directory_ << std::endl;
    }
}

geometry_msgs::PoseWithCovarianceStamped MapBuilder::GetPoseRegistration()
{
    geometry_msgs::PoseWithCovarianceStamped pose_registration_message;
    registration::AlignmentResult alignment_result_current
        = ndt_registration_.GetAlignmentResult();

    pose_registration_message.header.stamp = ros::Time::now();
    pose_registration_message.header.frame_id = "/map";
    pose_registration_message.pose.pose.position.x = alignment_result_current.final_transform(0,3);
    pose_registration_message.pose.pose.position.y = alignment_result_current.final_transform(1,3);
    pose_registration_message.pose.pose.position.z = alignment_result_current.final_transform(2,3);

    Eigen::Quaternionf quat(alignment_result_current.final_transform.block<3,3>(0, 0));

    pose_registration_message.pose.pose.orientation.x = quat.x();
    pose_registration_message.pose.pose.orientation.y = quat.y();
    pose_registration_message.pose.pose.orientation.z = quat.z();
    pose_registration_message.pose.pose.orientation.w = quat.w();

    return pose_registration_message;
}

Eigen::Matrix4f MapBuilder::PredictPose()
{
    registration::AlignmentResult alignment_result_current = ndt_registration_.GetAlignmentResult();

    Eigen::Matrix4f  delta_matrix
        = alignment_result_previous_.final_transform.inverse() * alignment_result_current.final_transform;
    Eigen::Matrix4f  predict_matrix = alignment_result_current.final_transform * delta_matrix;

    alignment_result_previous_ = alignment_result_current;

    return predict_matrix;
}

}  // namespace mapping
