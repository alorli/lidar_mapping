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


namespace mapping
{

MapBuilder::MapBuilder(std::string cfg_file_path)
    :ndt_registration_(cfg_file_path, 0),
     ndt_registration_compensation_(cfg_file_path, 1),
     motion_compensation_(0),
     laserscan_pose_interpolation_(cfg_file_path),
     gnss_constraints_builder_(cfg_file_path),
     closeloop_constraints_builder_(cfg_file_path),
     gnss_optimization_(cfg_file_path),
     closeloop_optimization_(cfg_file_path),
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

    is_save_vlp_raw_pcd_ = cfg_file_["map_builder"]["save_vlp_raw_pcd"].as<bool>();
    is_save_sick_raw_pcd_ = cfg_file_["map_builder"]["save_sick_raw_pcd"].as<bool>();

    std::string project_directory = cfg_file_["directory"]["project_directory"].as<std::string>();
    std::string map_main_directory = cfg_file_["directory"]["map"]["main_directory"].as<std::string>();
    std::string velodyne_main_directory = cfg_file_["directory"]["velodyne"]["main_directory"].as<std::string>();
    std::string sick_main_directory = cfg_file_["directory"]["sick"]["main_directory"].as<std::string>();
    std::string gnss_main_directory = cfg_file_["directory"]["gnss"]["main_directory"].as<std::string>();
    std::string optimization_main_directory = cfg_file_["directory"]["optimization"]["main_directory"].as<std::string>();
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

    save_gnss_path_ = project_directory + gnss_main_directory;
    save_velodyne_raw_pointcloud_path_ = project_directory + velodyne_main_directory + velodyne_raw_directory + velodyne_pcd_directory;
    save_sick_raw_pointcloud_path_ = project_directory + sick_main_directory + sick_raw_directory + sick_pcd_directory;
    save_velodyne_compensation_pointcloud_path_ = project_directory + velodyne_main_directory + velodyne_compensation_directory + velodyne_pcd_directory;
    
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


    // UTM 投影参数初始化
    if(!(pj_utm_ = pj_init_plus("+proj=utm +zone=50 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs")))
    {
        exit(1);
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
}

MapBuilder::~MapBuilder()
{
    velodyne_compensation_filelist_.close();
    velodyne_raw_filelist_.close();
    sick_raw_filelist_.close();
    gnss_fix_pose_file_.close();
    g2o_file_compensation_.close();
    g2o_file_raw_.close();
}

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

    // 调用时提供预测位姿
    // ndt_registration_.AddSensorData(timed_id_pointcloud, PredictPose());

    // 调用时没有提供预测位姿，使用内部线性外推预测位姿进行配准
    ndt_registration_.AddSensorData(timed_id_pointcloud);

    if(is_save_vlp_raw_pcd_)
    {
        SaveVlpRawPcd(timed_id_pointcloud);
    }
    
    
    if(allframe_id_ >= 1)
    {
        registration::TimedIdPointCloud timed_id_pointcloud_compensation;
        timed_id_pointcloud_compensation.time = timed_id_pointcloud_previous_.time;
        timed_id_pointcloud_compensation.allframe_id = timed_id_pointcloud_previous_.allframe_id;
        motion_compensation_.LinearInterpolation(timed_id_pointcloud_previous_.pointcloud,
                                                 timed_id_pointcloud_compensation.pointcloud,
                                                 alignment_result_previous_.final_transform,
                                                 ndt_registration_.GetAlignmentResult().final_transform
                                                );

        // 离线运动畸变补偿
        ndt_registration_compensation_.AddSensorData(timed_id_pointcloud_compensation);
        SaveVlpCompensationPcd(timed_id_pointcloud_compensation);
    }


    allframe_id_++;
    alignment_result_previous_ = ndt_registration_.GetAlignmentResult();
    timed_id_pointcloud_previous_ = timed_id_pointcloud;
}
 
void MapBuilder::AddGnssFixData(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    sensor_msgs::NavSatFix gnss_data = *msg;

    static long long msg_id = 0;
    double utm_x, utm_y, utm_z;

    utm_x = common::degree_to_radian(gnss_data.longitude);
    utm_y = common::degree_to_radian(gnss_data.latitude);

    // 高程直接取自gga报文中的高程
    utm_z = gnss_data.altitude;

    pj_transform(pj_latlong_, pj_utm_, 1, 1, &utm_x, &utm_y, NULL);

    common::Time time_current = common::FromRos(msg->header.stamp);

    // std::cout << "\033[32m" 
    //           << "utm_x:" << utm_x
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


    // 保存 sick 点云帧消息
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

    // 输出 g2o 文件
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


        // 输出 g2o 文件
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
    // map_generator_closeloop_optimization_.GeneratePointCloudMap();
    map_generator_closeloop_optimization_.GeneratePointCloudRGBAMap();

}

void MapBuilder::GenerateLaserScanMap()
{
    laserscan_pose_interpolation_.RunIterpolation();
    // map_generator_laserscan_.GeneratePointCloudMap();
    map_generator_laserscan_.GeneratePointCloudRGBAMap();
    
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


