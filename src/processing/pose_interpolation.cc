///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_02_24
///////////////////////////////////////////////////////////////////////////////

#include "src/processing/pose_interpolation.h"
// #include <tf/transform_datatypes.h>
#include <iostream>

namespace processing
{

PoseInterpolation::PoseInterpolation(std::string cfg_file_path,
                                     std::string project_directory_name)
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");

    std::string project_directory = cfg_file_["directory"]["project_directory"].as<std::string>();
    project_directory = project_directory + project_directory_name;
    
    std::string optimization_main_directory = cfg_file_["directory"]["optimization"]["main_directory"].as<std::string>();
    std::string sick_main_directory = cfg_file_["directory"]["sick"]["main_directory"].as<std::string>();
    std::string sick_raw_directory = cfg_file_["directory"]["sick"]["raw_directory"].as<std::string>();
    std::string sick_registration_results_filelist = cfg_file_["directory"]["sick"]["registration_results_filelist"].as<std::string>();

    std::string closeloop_optimization_results_file = cfg_file_["directory"]["optimization"]["closeloop_optimization_results_file"].as<std::string>();
    std::string interpolation_results_filelist = cfg_file_["directory"]["sick"]["interpolation_results_filelist"].as<std::string>();

    interpolation_parameter_.extract_frame_distance = cfg_file_["sick_pose_interpolation"]["interpolation_parameter"]["extract_frame_distance"].as<double>();
    interpolation_parameter_.min_delta_sick_time = cfg_file_["sick_pose_interpolation"]["interpolation_parameter"]["min_delta_sick_time"].as<double>();

    double x = cfg_file_["sick_pose_interpolation"]["transform_sick_in_vlp"]["x"].as<double>();
    double y = cfg_file_["sick_pose_interpolation"]["transform_sick_in_vlp"]["y"].as<double>();
    double z = cfg_file_["sick_pose_interpolation"]["transform_sick_in_vlp"]["z"].as<double>();
    double roll = cfg_file_["sick_pose_interpolation"]["transform_sick_in_vlp"]["roll"].as<double>();
    double pitch = cfg_file_["sick_pose_interpolation"]["transform_sick_in_vlp"]["pitch"].as<double>();
    double yaw = cfg_file_["sick_pose_interpolation"]["transform_sick_in_vlp"]["yaw"].as<double>();

    Eigen::Translation3f translation_sick_to_vlp(x, y, z);                    
    Eigen::AngleAxisf rotation_x_sick_to_vlp(common::degree_to_radian(roll), Eigen::Vector3f::UnitX());    
    Eigen::AngleAxisf rotation_y_sick_to_vlp(common::degree_to_radian(pitch), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotation_z_sick_to_vlp(common::degree_to_radian(yaw), Eigen::Vector3f::UnitZ()); 
    transform_sick_to_vlp_ = (translation_sick_to_vlp * rotation_z_sick_to_vlp * rotation_y_sick_to_vlp * rotation_x_sick_to_vlp).matrix();


    reference_poses_file_path_ = project_directory + optimization_main_directory + closeloop_optimization_results_file;
    interpolation_poses_file_path_ = project_directory + sick_main_directory + sick_raw_directory + sick_registration_results_filelist;
    interpolation_results_file_path_ = project_directory + sick_main_directory + sick_raw_directory + interpolation_results_filelist;
    
    std::cout << std::endl;
    std::cout << "\033[32m" << "----------PoseInterpolation parameter:----------" << "\033[0m" << std::endl;
    std::cout << "project_directory:"  << "\033[33m" << project_directory  << "\033[0m" << std::endl;
    std::cout << "optimization_main_directory:"  << "\033[33m" << optimization_main_directory  << "\033[0m" << std::endl;
    std::cout << "sick_main_directory:"  << "\033[33m" << sick_main_directory  << "\033[0m" << std::endl;
    std::cout << "sick_raw_directory:"  << "\033[33m" << sick_raw_directory  << "\033[0m" << std::endl;
    std::cout << "closeloop_optimization_results_file:"  << "\033[33m" << closeloop_optimization_results_file  << "\033[0m" << std::endl;
    std::cout << "reference_poses_file_path_:"  << "\033[33m" << reference_poses_file_path_  << "\033[0m" << std::endl;
    std::cout << "interpolation_poses_file_path_:"  << "\033[33m" << interpolation_poses_file_path_  << "\033[0m" << std::endl;
    std::cout << "interpolation_parameter_.extract_frame_distance:"  << "\033[33m" << interpolation_parameter_.extract_frame_distance  << "\033[0m" << std::endl;
    std::cout << "interpolation_parameter_.min_delta_sick_time:"  << "\033[33m" << interpolation_parameter_.min_delta_sick_time  << "\033[0m" << std::endl;
    std::cout << "x:"  << "\033[33m" << x  << "\033[0m" << std::endl;
    std::cout << "y:"  << "\033[33m" << y  << "\033[0m" << std::endl;
    std::cout << "z:"  << "\033[33m" << z  << "\033[0m" << std::endl;
    std::cout << "roll:"  << "\033[33m" << roll  << "\033[0m" << std::endl;
    std::cout << "pitch:"  << "\033[33m" << pitch  << "\033[0m" << std::endl;
    std::cout << "yaw:"  << "\033[33m" << yaw  << "\033[0m" << std::endl;






    interpolation_results_filelist_.open(interpolation_results_file_path_.c_str());
}

PoseInterpolation::~PoseInterpolation()
{
    interpolation_results_filelist_.close();
}

void PoseInterpolation::LoadReferencePoses()
{
    std::ifstream filelists;
    filelists.open(reference_poses_file_path_);
    mapping::RegistrationResultsFilelist reference_poses_filelist;
    long long time;

    while(filelists >> reference_poses_filelist.allframe_id 
                    >> time
                    >> reference_poses_filelist.is_gnss_constraint
                    >> reference_poses_filelist.pcd_file_path
                    >> reference_poses_filelist.alignment_result.is_converged
                    >> reference_poses_filelist.alignment_result.fitness_score
                    >> reference_poses_filelist.alignment_result.time_duration_ms
                    >> reference_poses_filelist.alignment_result.final_num_iteration
                    >> reference_poses_filelist.alignment_result.final_transform(0,0) >> reference_poses_filelist.alignment_result.final_transform(0,1) >> reference_poses_filelist.alignment_result.final_transform(0,2) >> reference_poses_filelist.alignment_result.final_transform(0,3)
                    >> reference_poses_filelist.alignment_result.final_transform(1,0) >> reference_poses_filelist.alignment_result.final_transform(1,1) >> reference_poses_filelist.alignment_result.final_transform(1,2) >> reference_poses_filelist.alignment_result.final_transform(1,3)
                    >> reference_poses_filelist.alignment_result.final_transform(2,0) >> reference_poses_filelist.alignment_result.final_transform(2,1) >> reference_poses_filelist.alignment_result.final_transform(2,2) >> reference_poses_filelist.alignment_result.final_transform(2,3)
                    >> reference_poses_filelist.alignment_result.final_transform(3,0) >> reference_poses_filelist.alignment_result.final_transform(3,1) >> reference_poses_filelist.alignment_result.final_transform(3,2) >> reference_poses_filelist.alignment_result.final_transform(3,3)
    )
    {

            reference_poses_filelist.time = common::FromUniversal(time);
            reference_poses_filelists_.push_back(reference_poses_filelist);

            std::cout << "\033[33m size:" << reference_poses_filelists_.size() 
                        << " allframe_id: " << reference_poses_filelist.allframe_id 
                        << " time: " << reference_poses_filelist.time 
                        << " pcd_file_path: " << reference_poses_filelist.pcd_file_path 
                        << " final_transform: " << reference_poses_filelist.alignment_result.final_transform
                        << "\033[0m"
                        << std::endl;
    }
    filelists.close();
}


void PoseInterpolation::LoadInterpolationPoses()
{
    std::ifstream filelists;
    filelists.open(interpolation_poses_file_path_);
    mapping::RegistrationResultsFilelist interpolation_poses_filelist;
    long long time;

    while(filelists >> interpolation_poses_filelist.allframe_id 
                    >> time
                    >> interpolation_poses_filelist.is_gnss_constraint
                    >> interpolation_poses_filelist.pcd_file_path
                    >> interpolation_poses_filelist.alignment_result.is_converged
                    >> interpolation_poses_filelist.alignment_result.fitness_score
                    >> interpolation_poses_filelist.alignment_result.time_duration_ms
                    >> interpolation_poses_filelist.alignment_result.final_num_iteration
                    >> interpolation_poses_filelist.alignment_result.final_transform(0,0) >> interpolation_poses_filelist.alignment_result.final_transform(0,1) >> interpolation_poses_filelist.alignment_result.final_transform(0,2) >> interpolation_poses_filelist.alignment_result.final_transform(0,3)
                    >> interpolation_poses_filelist.alignment_result.final_transform(1,0) >> interpolation_poses_filelist.alignment_result.final_transform(1,1) >> interpolation_poses_filelist.alignment_result.final_transform(1,2) >> interpolation_poses_filelist.alignment_result.final_transform(1,3)
                    >> interpolation_poses_filelist.alignment_result.final_transform(2,0) >> interpolation_poses_filelist.alignment_result.final_transform(2,1) >> interpolation_poses_filelist.alignment_result.final_transform(2,2) >> interpolation_poses_filelist.alignment_result.final_transform(2,3)
                    >> interpolation_poses_filelist.alignment_result.final_transform(3,0) >> interpolation_poses_filelist.alignment_result.final_transform(3,1) >> interpolation_poses_filelist.alignment_result.final_transform(3,2) >> interpolation_poses_filelist.alignment_result.final_transform(3,3)
    )
    {

            interpolation_poses_filelist.time = common::FromUniversal(time);
            interpolation_poses_filelists_.push_back(interpolation_poses_filelist);

            // std::cout << "\033[33m size:" << reference_poses_filelists_.size() 
            //             << " allframe_id: " << interpolation_poses_filelist.allframe_id 
            //             << " time: " << interpolation_poses_filelist.time 
            //             << " pcd_file_path: " << interpolation_poses_filelist.pcd_file_path 
            //             << " final_transform: " << interpolation_poses_filelist.alignment_result.final_transform
            //             << "\033[0m"
            //             << std::endl;
    }
    filelists.close();
}

void PoseInterpolation::RunIterpolation()
{
    LoadReferencePoses();
    LoadInterpolationPoses();
    LinearInterpolation();
    SaveInterpolationResultsFile();
}


void PoseInterpolation::LinearInterpolation()
{       
    std::cout << "\033[33m reference_poses_filelists_.size:" << reference_poses_filelists_.size() 
            << "\033[0m"
            << std::endl;


    std::cout << "\033[33m interpolation_poses_filelists_.size:" << interpolation_poses_filelists_.size() 
            << "\033[0m"
            << std::endl;

    std::vector<mapping::RegistrationResultsFilelist>::iterator item_reference_pose_search_position = reference_poses_filelists_.begin();
    
    for(std::vector<mapping::RegistrationResultsFilelist>::iterator item_interpolation_pose = interpolation_poses_filelists_.begin();
                                                                    item_interpolation_pose != interpolation_poses_filelists_.end();
                                                                    item_interpolation_pose++)
    {
        double interpolation_pose_time = common::ToUniversalSeconds(item_interpolation_pose->time);
        // std::cout << std::setprecision(15)
        //           << "\033[33m interpolation_pose_time:" << interpolation_pose_time 
        //           << "\033[0m"
        //           << std::endl; 



        // 内层遍历多线雷达位姿
        for(std::vector<mapping::RegistrationResultsFilelist>::iterator item_reference_pose = item_reference_pose_search_position;
                                                                        item_reference_pose != reference_poses_filelists_.end() - 1;
                                                                        item_reference_pose++)
        {
            std::vector<mapping::RegistrationResultsFilelist>::iterator item_reference_pose_next = item_reference_pose + 1;
            double reference_pose_time = common::ToUniversalSeconds(item_reference_pose->time);
            double reference_pose_time_next = common::ToUniversalSeconds(item_reference_pose_next->time);

            // std::cout << std::setprecision(15)
            //           << "\033[33m reference_pose_time:" << gnss_time
            //           << "  reference_pose_time_next:" << gnss_time_next 
            //           << "\033[0m"
            //           << std::endl; 

            if(reference_pose_time_next < interpolation_pose_time)
            {
                item_reference_pose_search_position++;
                continue;
            }

            if(reference_pose_time > interpolation_pose_time)
            {
                break;
            }

            double delta_time = reference_pose_time_next - reference_pose_time;
            if((reference_pose_time <= interpolation_pose_time) 
            && (reference_pose_time_next >= interpolation_pose_time)
            && (delta_time < interpolation_parameter_.min_delta_sick_time))
            {
                // std::cout << std::setprecision(15)
                //           << "reference_pose_time:" << "\033[32m" << reference_pose_time << "\033[0m"
                //           << "  interpolation_pose_time:" << "\033[32m" << interpolation_pose_time << "\033[0m"
                //           << "  reference_pose_time_next:" << "\033[32m" << reference_pose_time_next << "\033[0m"
                //           << "  delta_time1:" << "\033[32m" << interpolation_pose_time - reference_pose_time << "\033[0m"
                //           << "  delta_time2:" << "\033[32m" << reference_pose_time_next - interpolation_pose_time << "\033[0m"
                //           << "  delta_time:" << "\033[32m" << delta_time << "\033[0m"
                //           << std::endl; 


                Eigen::Vector3f reference_translation = item_reference_pose->alignment_result.final_transform.block<3,1>(0,3);
                Eigen::Vector3f reference_translation_next = item_reference_pose_next->alignment_result.final_transform.block<3,1>(0,3);

                Eigen::Quaternionf reference_quat(item_reference_pose->alignment_result.final_transform.block<3,3>(0,0));
                Eigen::Quaternionf reference_quat_next(item_reference_pose_next->alignment_result.final_transform.block<3,3>(0,0));

                double alpha = (interpolation_pose_time - reference_pose_time)/(reference_pose_time_next - reference_pose_time);
                double beta = (reference_pose_time_next - interpolation_pose_time)/(reference_pose_time_next - reference_pose_time); 

                Eigen::Vector3f interpolation_translation = alpha * reference_translation_next + beta * reference_translation;
                Eigen::Quaternionf interpolation_quat = reference_quat.slerp(alpha, 
                                                                             reference_quat_next);

                Eigen::Matrix4f interpolation_transform = Eigen::Matrix4f::Identity();
                interpolation_transform.block<3,3>(0,0) = Eigen::Matrix3f(interpolation_quat);
                interpolation_transform.block<3,1>(0,3) = Eigen::Vector3f(interpolation_translation);

                mapping::RegistrationResultsFilelist interpolation_poses_result = *item_interpolation_pose;

                interpolation_poses_result.alignment_result.final_transform = interpolation_transform;

                interpolation_poses_results_filelists_.push_back(interpolation_poses_result);
            }

        }
    }
}

void PoseInterpolation::SaveInterpolationResultsFile()
{
    std::vector<mapping::RegistrationResultsFilelist>::iterator item = interpolation_poses_results_filelists_.begin();
    Eigen::Vector3f translation_previous = Eigen::Vector3f(item->alignment_result.final_transform(0,3),
                                                           item->alignment_result.final_transform(1,3),
                                                           item->alignment_result.final_transform(2,3));

    for(;
        item != interpolation_poses_results_filelists_.end();
        item++)
    {
        Eigen::Vector3f translation_current = Eigen::Vector3f(item->alignment_result.final_transform(0,3),
                                                              item->alignment_result.final_transform(1,3),
                                                              item->alignment_result.final_transform(2,3));

        item->alignment_result.final_transform = item->alignment_result.final_transform * transform_sick_to_vlp_;
        Eigen::Matrix3f rotation_current = item->alignment_result.final_transform.block<3,3>(0,0);

        double delta_x = translation_current[0] - translation_previous[0];
        double delta_y = translation_current[1] - translation_previous[1];
        double delta_z = translation_current[2] - translation_previous[2];
        double delta_distance = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);

        if(delta_distance > interpolation_parameter_.extract_frame_distance)
        {
            translation_previous = translation_current;

            interpolation_results_filelist_ << std::setprecision(15)
                                            << item->allframe_id
                                            << " " << item->time
                                            << " " << item->is_gnss_constraint
                                            << " " << item->pcd_file_path
                                            << " " << item->alignment_result.is_converged
                                            << " " << item->alignment_result.fitness_score
                                            << " " << item->alignment_result.time_duration_ms
                                            << " " << item->alignment_result.final_num_iteration
                                            << " " << item->alignment_result.final_transform(0,0) << " " << item->alignment_result.final_transform(0,1) << " " << item->alignment_result.final_transform(0,2) << " " << item->alignment_result.final_transform(0,3)
                                            << " " << item->alignment_result.final_transform(1,0) << " " << item->alignment_result.final_transform(1,1) << " " << item->alignment_result.final_transform(1,2) << " " << item->alignment_result.final_transform(1,3)
                                            << " " << item->alignment_result.final_transform(2,0) << " " << item->alignment_result.final_transform(2,1) << " " << item->alignment_result.final_transform(2,2) << " " << item->alignment_result.final_transform(2,3)
                                            << " " << item->alignment_result.final_transform(3,0) << " " << item->alignment_result.final_transform(3,1) << " " << item->alignment_result.final_transform(3,2) << " " << item->alignment_result.final_transform(3,3)
                                            << std::endl;
        }
    }
}



}  // namespace processing


