///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_23
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
// #include <sstream>
#include <fstream>
#include "src/constraints/gnss_constraints_builder.h"

namespace constraints
{

GnssConstraintsBuilder::GnssConstraintsBuilder(std::string cfg_file_path)
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");

    std::string project_directory = cfg_file_["directory"]["project_directory"].as<std::string>();
    std::string velodyne_main_directory = cfg_file_["directory"]["velodyne"]["main_directory"].as<std::string>();
    std::string gnss_main_directory = cfg_file_["directory"]["gnss"]["main_directory"].as<std::string>();
    std::string compensation_directory = cfg_file_["directory"]["velodyne"]["compensation_directory"].as<std::string>();
    std::string registration_results_filelist = cfg_file_["directory"]["velodyne"]["registration_results_filelist"].as<std::string>();
    
    std::string gnss_pose_file = cfg_file_["directory"]["gnss"]["gnss_fix_pose_file"].as<std::string>();

    lidar_odom_pose_path_ = project_directory + velodyne_main_directory + compensation_directory + registration_results_filelist;

    gnss_pose_path_ = project_directory + gnss_main_directory + gnss_pose_file;

    gnss_constraints_builder_parameter_.min_delta_gnss_time = cfg_file_["gnss_constraints_builder"]["gnss_constraints_builder_parameter"]["min_delta_gnss_time"].as<double>();

    std::cout << std::endl;
    std::cout << "\033[32m" << "----------GnssConstraintsBuilderParameter parameter:----------" << "\033[0m" << std::endl;
    std::cout << "lidar_odom_pose_path_:" << lidar_odom_pose_path_ << std::endl;
    std::cout << "gnss_pose_path_:" << gnss_pose_path_ << std::endl;
    std::cout << "gnss_constraints_builder_parameter_.min_delta_gnss_time:" << gnss_constraints_builder_parameter_.min_delta_gnss_time << std::endl;
}

GnssConstraintsBuilder::~GnssConstraintsBuilder()
{
}

void GnssConstraintsBuilder::LoadLidarOdomPoses()
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

        // std::cout << "\033[32m size:" << registration_results_filelists_.size() 
        //           << " allframe_id: " << registration_results_filelist.allframe_id 
        //           << " time: " << registration_results_filelist.time 
        //           << " pcd_file_path: " << registration_results_filelist.pcd_file_path 
        //           << " final_transform: " << registration_results_filelist.alignment_result.final_transform
        //           << "\033[0m"
        //           << std::endl;
    }
    filelists.close();
}

void GnssConstraintsBuilder::LoadGnssPoses()
{
    std::ifstream filelists;
    filelists.open(gnss_pose_path_);
    GnssPose gnss_pose;
    long long time;
    while(filelists >> gnss_pose.allframe_id 
                    >> time
                    >> gnss_pose.status
                    >> gnss_pose.service
                    >> gnss_pose.position[0]
                    >> gnss_pose.position[1]
                    >> gnss_pose.position[2]
    )
    {
        gnss_pose.time = common::FromUniversal(time);
        gnss_poses_.push_back(gnss_pose);

        // std::cout << "\033[34m size:" << gnss_poses_.size() 
        //           << " allframe_id: " << gnss_pose.allframe_id 
        //           << " time: " << gnss_pose.time 
        //           << " position:x: " << gnss_pose.position[0]
        //           << " y: " << gnss_pose.position[1]
        //           << " z: " << gnss_pose.position[2]
        //           << "\033[0m"
        //           << std::endl;
    }
    filelists.close();
}

void GnssConstraintsBuilder::BuildConstraints()
{
    LoadLidarOdomPoses();
    LoadGnssPoses();



    std::cout << "\033[33m gnss_poses_.size:" << gnss_poses_.size() 
              << "\033[0m"
              << std::endl; 

    std::cout << "\033[33m registration_results_filelists_.size:" << registration_results_filelists_.size() 
              << "\033[0m"
              << std::endl; 

    
    std::vector<GnssPose>::iterator item_gnss_search_position = gnss_poses_.begin();
    
    // 外层遍历雷达里程计位姿
    for(std::vector<mapping::RegistrationResultsFilelist>::iterator item_lidar_pose = registration_results_filelists_.begin();
                                                                    item_lidar_pose != registration_results_filelists_.end();
                                                                    item_lidar_pose++)
    {
        double lidar_pose_time = common::ToUniversalSeconds(item_lidar_pose->time);
        // std::cout << std::setprecision(15)
        //           << "\033[33m lidar_pose_time:" << lidar_pose_time 
        //           << "\033[0m"
        //           << std::endl; 



        // 内层遍历 gnss 位置
        for(std::vector<GnssPose>::iterator item_gnss_pose = item_gnss_search_position;
                                            item_gnss_pose != gnss_poses_.end() - 1;
                                            item_gnss_pose++)
        {
            std::vector<GnssPose>::iterator item_gnss_pose_next = item_gnss_pose + 1;
            double gnss_time = common::ToUniversalSeconds(item_gnss_pose->time);
            double gnss_time_next = common::ToUniversalSeconds(item_gnss_pose_next->time);
            int position_type = item_gnss_pose->status;

            double gnss_pose_x = item_gnss_pose->position[0];
            double gnss_pose_y = item_gnss_pose->position[1];
            double gnss_pose_z = item_gnss_pose->position[2];

            double gnss_pose_next_x = item_gnss_pose_next->position[0];
            double gnss_pose_next_y = item_gnss_pose_next->position[1];
            double gnss_pose_next_z = item_gnss_pose_next->position[2];

            // std::cout << std::setprecision(15)
            //           << "\033[33m gnss_time:" << gnss_time
            //           << "  gnss_time_next:" << gnss_time_next 
            //           << "\033[0m"
            //           << std::endl; 

            if(gnss_time_next < lidar_pose_time)
            {
                item_gnss_search_position++;
                continue;
            }

            if(gnss_time > lidar_pose_time)
            {
                break;
            }

            double delta_gnss_time = gnss_time_next - gnss_time;
            if((gnss_time <= lidar_pose_time) 
            && (gnss_time_next >= lidar_pose_time) 
            && (delta_gnss_time < gnss_constraints_builder_parameter_.min_delta_gnss_time)
            && (position_type == 4))
            {
                // std::cout << std::setprecision(15)
                //           << "\033[33m gnss_time:" << gnss_time
                //           << "  lidar_time:" << lidar_pose_time
                //           << "  gnss_time_next:" << gnss_time_next
                //           << "  delta_time1:" << lidar_pose_time - gnss_time
                //           << "  delta_time2:" << gnss_time_next - lidar_pose_time
                //           << "  delta_gnss:" << delta_gnss_time
                //           << "\033[0m"
                //           << std::endl; 

                double alpha = (lidar_pose_time - gnss_time)/(gnss_time_next - gnss_time);
                double beta = (gnss_time_next - lidar_pose_time)/(gnss_time_next - gnss_time); 

                Eigen::Vector3d gnss_pose_interpolation;
                gnss_pose_interpolation[0] =  alpha * gnss_pose_next_x + beta * gnss_pose_x;
                gnss_pose_interpolation[1] =  alpha * gnss_pose_next_y + beta * gnss_pose_y;
                gnss_pose_interpolation[2] =  alpha * gnss_pose_next_z + beta * gnss_pose_z;  

                GnssPoseConstraint gnss_pose_constraint;
                gnss_pose_constraint.lidar_allframe_id = item_lidar_pose->allframe_id; 
                gnss_pose_constraint.prior_position = gnss_pose_interpolation; 
                gnss_pose_constraints_.push_back(gnss_pose_constraint);        
            }

        }
    }

    std::cout << std::setprecision(15)
              << "\033[32m gnss_pose_constraints_.size:" << gnss_pose_constraints_.size()
              << "\033[0m"
              << std::endl; 


}






}  // namespace registration


