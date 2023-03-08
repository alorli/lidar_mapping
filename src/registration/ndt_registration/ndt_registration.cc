///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_23
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>
#include "src/registration/ndt_registration/ndt_registration.h"
#include <pcl/filters/passthrough.h>

#include <tf/transform_datatypes.h>



namespace registration
{
NdtRegistration::NdtRegistration(std::string cfg_file_path,
                                 int registration_flag)
    :is_fist_frame_(true),
     is_fist_map_(true),
     registration_flag_(registration_flag),
     map_ptr_(new pcl::PointCloud<PointType>()),
     pose_last_keyframe_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
     pose_last_trim_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
     alignment_result_({true, 0.0, 0.0, 0.0, 1, Eigen::Matrix4f::Identity()}),
     alignment_result_previous_({true, 0.0, 0.0, 0.0, 0, Eigen::Matrix4f::Identity()})
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");

    filter_parameter_.min_horizontal_radius = cfg_file_["ndt_registration"]["filter_parameter"]["min_horizontal_radius"].as<double>();
    filter_parameter_.min_num_points = cfg_file_["ndt_registration"]["filter_parameter"]["min_num_points"].as<int>();
    filter_parameter_.max_radius = cfg_file_["ndt_registration"]["filter_parameter"]["max_radius"].as<double>();
    filter_parameter_.voxel_leaf_size = cfg_file_["ndt_registration"]["filter_parameter"]["voxel_leaf_size"].as<double>();
    filter_parameter_.scale_voxel_leaf_size = cfg_file_["ndt_registration"]["filter_parameter"]["scale_voxel_leaf_size"].as<double>();
    ndt_parameter_.transformation_epsilon = cfg_file_["ndt_registration"]["ndt_parameter"]["transformation_epsilon"].as<double>();
    ndt_parameter_.step_size = cfg_file_["ndt_registration"]["ndt_parameter"]["step_size"].as<double>();
    ndt_parameter_.resolution = cfg_file_["ndt_registration"]["ndt_parameter"]["resolution"].as<double>();
    ndt_parameter_.max_iterations = cfg_file_["ndt_registration"]["ndt_parameter"]["max_iterations"].as<int>();
    mapping_parameter_.num_start_frames_add_to_map = cfg_file_["ndt_registration"]["mapping_parameter"]["num_start_frames_add_to_map"].as<int>();
    mapping_parameter_.min_keyframe_distance = cfg_file_["ndt_registration"]["mapping_parameter"]["min_keyframe_distance"].as<double>();
    mapping_parameter_.map_trim_distance = cfg_file_["ndt_registration"]["mapping_parameter"]["map_trim_distance"].as<double>();

    mapping_parameter_.pass_through_filter.x_min = cfg_file_["ndt_registration"]["mapping_parameter"]["map_passthrough"]["x_min"].as<double>();
    mapping_parameter_.pass_through_filter.y_min = cfg_file_["ndt_registration"]["mapping_parameter"]["map_passthrough"]["y_min"].as<double>();
    mapping_parameter_.pass_through_filter.z_min = cfg_file_["ndt_registration"]["mapping_parameter"]["map_passthrough"]["z_min"].as<double>();
    mapping_parameter_.pass_through_filter.x_max = cfg_file_["ndt_registration"]["mapping_parameter"]["map_passthrough"]["x_max"].as<double>();
    mapping_parameter_.pass_through_filter.y_max = cfg_file_["ndt_registration"]["mapping_parameter"]["map_passthrough"]["y_max"].as<double>();
    mapping_parameter_.pass_through_filter.z_max = cfg_file_["ndt_registration"]["mapping_parameter"]["map_passthrough"]["z_max"].as<double>();

    std::cout << std::endl;
    std::cout << "\033[32m" << "----------NdtRegistration parameter:----------" << "\033[0m" << std::endl;
    std::cout << "filter_parameter_.min_horizontal_radius:" << "\033[33m"  << filter_parameter_.min_horizontal_radius  << "\033[0m" << std::endl;
    std::cout << "filter_parameter_.min_num_points:" << "\033[33m"  << filter_parameter_.min_num_points  << "\033[0m" << std::endl;
    std::cout << "filter_parameter_.max_radius:" << "\033[33m"  << filter_parameter_.max_radius  << "\033[0m" << std::endl;
    std::cout << "filter_parameter_.voxel_leaf_size:" << "\033[33m"  << filter_parameter_.voxel_leaf_size  << "\033[0m" << std::endl;
    std::cout << "filter_parameter_.scale_voxel_leaf_size:" << "\033[33m"  << filter_parameter_.scale_voxel_leaf_size  << "\033[0m" << std::endl;
    std::cout << "ndt_parameter_.transformation_epsilon:" << "\033[33m"  << ndt_parameter_.transformation_epsilon  << "\033[0m" << std::endl;
    std::cout << "ndt_parameter_.step_size:" << "\033[33m"  << ndt_parameter_.step_size  << "\033[0m" << std::endl;
    std::cout << "ndt_parameter_.resolution:" << "\033[33m"  << ndt_parameter_.resolution  << "\033[0m" << std::endl;
    std::cout << "ndt_parameter_.max_iterations:" << "\033[33m"  << ndt_parameter_.max_iterations << "\033[0m"  << std::endl;
    std::cout << "mapping_parameter_.min_keyframe_distance:" << "\033[33m"  << mapping_parameter_.min_keyframe_distance << "\033[0m"  << std::endl;
    std::cout << "mapping_parameter_.map_trim_distance:" << "\033[33m"  << mapping_parameter_.map_trim_distance << "\033[0m"  << std::endl;
    std::cout << "mapping_parameter_.pass_through_filter.x_min:" << "\033[33m"  << mapping_parameter_.pass_through_filter.x_min << "\033[0m"  << std::endl;
    std::cout << "mapping_parameter_.pass_through_filter.x_max:" << "\033[33m"  << mapping_parameter_.pass_through_filter.x_max << "\033[0m"  << std::endl;
    std::cout << "mapping_parameter_.pass_through_filter.y_min:" << "\033[33m"  << mapping_parameter_.pass_through_filter.y_min << "\033[0m"  << std::endl;
    std::cout << "mapping_parameter_.pass_through_filter.y_max:" << "\033[33m"  << mapping_parameter_.pass_through_filter.y_max  << "\033[0m" << std::endl;
    std::cout << "mapping_parameter_.pass_through_filter.z_min:" << "\033[33m"  << mapping_parameter_.pass_through_filter.z_min  << "\033[0m" << std::endl;
    std::cout << "mapping_parameter_.pass_through_filter.z_max:" << "\033[33m"  << mapping_parameter_.pass_through_filter.z_max << "\033[0m"  << std::endl;
}

NdtRegistration::~NdtRegistration()
{
}


void NdtRegistration::AddSensorData(const TimedIdPointCloud& timed_id_pointcloud,
                                    const Eigen::Matrix4f  predict_matrix)
{
    pcl::PointCloud<PointType>::Ptr voxel_filtered_pointcloud_ptr(new pcl::PointCloud<PointType>());

    if(FilterSensorData(timed_id_pointcloud,
                        voxel_filtered_pointcloud_ptr))
    {
        Alignment(voxel_filtered_pointcloud_ptr,
                  predict_matrix);

        AddKeyframeToMap();

        TrimMap();
    }

}


void NdtRegistration::AddSensorData(const TimedIdPointCloud& timed_id_pointcloud)
{
    pcl::PointCloud<PointType>::Ptr voxel_filtered_pointcloud_ptr(new pcl::PointCloud<PointType>());

    if(FilterSensorData(timed_id_pointcloud,
                        voxel_filtered_pointcloud_ptr))
    {
        Alignment(voxel_filtered_pointcloud_ptr,
                  PredictPose());

        // Alignment(voxel_filtered_pointcloud_ptr,
        //           PredictPoseUseTf());

        AddKeyframeToMap();

        TrimMap();
    }

}

bool NdtRegistration::FilterSensorData(const TimedIdPointCloud& timed_id_pointcloud,
                                       pcl::PointCloud<PointType>::Ptr voxel_filtered_pointcloud_ptr)
{
    PointType point;
    filtered_timed_id_pointcloud_.pointcloud.clear();
    filtered_timed_id_pointcloud_.time = timed_id_pointcloud.time;
    filtered_timed_id_pointcloud_.allframe_id = timed_id_pointcloud.allframe_id;
    for(pcl::PointCloud<PointType>::const_iterator item = timed_id_pointcloud.pointcloud.begin();
        item != timed_id_pointcloud.pointcloud.end();
        item++)
    {
        point.x = (double)item->x;
        point.y = (double)item->y;
        point.z = (double)item->z;

        if(typeid(PointType) == typeid(pcl::PointXYZI))
        {
            point.intensity = (double)item->intensity;
        }

        double horizontal_radius = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));
        double radius = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0) + pow(point.z, 2.0));

        if((horizontal_radius > filter_parameter_.min_horizontal_radius) && (radius < filter_parameter_.max_radius))
        {
            filtered_timed_id_pointcloud_.pointcloud.push_back(point);
        }
    }

    if(is_fist_frame_)
    {
        is_fist_frame_ = false;
        *map_ptr_ += filtered_timed_id_pointcloud_.pointcloud;
    }


    pcl::PointCloud<PointType>::Ptr pointcloud_ptr(new pcl::PointCloud<PointType>(filtered_timed_id_pointcloud_.pointcloud));
    pcl::VoxelGrid<PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(filter_parameter_.voxel_leaf_size,
                                  filter_parameter_.voxel_leaf_size,
                                  filter_parameter_.voxel_leaf_size);

    voxel_grid_filter.setInputCloud(pointcloud_ptr);
    voxel_grid_filter.filter(*voxel_filtered_pointcloud_ptr);


    // added by lichunjing 2021-04-24
    // std::cout << "pointcloud_ptr->size():" << pointcloud_ptr->size()
    //           << "   voxel_filtered_pointcloud_ptr->size():" << voxel_filtered_pointcloud_ptr->size()
    //           << std::endl;
    //
    // double voxel_leaf_size = filter_parameter_.voxel_leaf_size * filter_parameter_.scale_voxel_leaf_size;
    // while((pointcloud_ptr->size() == voxel_filtered_pointcloud_ptr->size()) && (voxel_filtered_pointcloud_ptr->size() > 5000))
    // {
    //     voxel_grid_filter.setLeafSize(voxel_leaf_size,
    //                                   voxel_leaf_size,
    //                                   voxel_leaf_size);
    //
    //     voxel_leaf_size = voxel_leaf_size * filter_parameter_.scale_voxel_leaf_size;
    //     voxel_grid_filter.setInputCloud(pointcloud_ptr);
    //     voxel_grid_filter.filter(*voxel_filtered_pointcloud_ptr);
    //
    //     std::cout << "-----------pointcloud_ptr->size():" << pointcloud_ptr->size()
    //               << "   voxel_filtered_pointcloud_ptr->size():" << voxel_filtered_pointcloud_ptr->size()
    //               << std::endl;
    //
    // }
    //
    //
    // // added by lichunjing 2021-04-24
    // if(voxel_filtered_pointcloud_ptr->size() < filter_parameter_.min_num_points)
    // {
    //     return false;
    // }
    // else
    // {
    //    return true;
    // }

    return true;

}

void NdtRegistration::Alignment(pcl::PointCloud<PointType>::Ptr voxel_filtered_pointcloud_ptr,
                                const Eigen::Matrix4f  predict_matrix)
{
    if(is_fist_map_)
    {
        is_fist_map_ = false;
        pcl_ndt_.setInputTarget(map_ptr_);
    }

    pcl_ndt_.setTransformationEpsilon(ndt_parameter_.transformation_epsilon);
    pcl_ndt_.setStepSize(ndt_parameter_.step_size);
    pcl_ndt_.setResolution(ndt_parameter_.resolution);
    pcl_ndt_.setMaximumIterations(ndt_parameter_.max_iterations);
    pcl_ndt_.setInputSource(voxel_filtered_pointcloud_ptr);

    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

    pcl::PointCloud<PointType>::Ptr output_pointcloud_ptr(new pcl::PointCloud<PointType>);  // ndt匹配后的输出点云
    pcl_ndt_.align(*output_pointcloud_ptr, predict_matrix);

    std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> delta_time
          = std::chrono::duration_cast<std::chrono::duration<double>>(time_end - time_start);

    alignment_result_.time_duration_ms = delta_time.count()*1000.0;
    alignment_result_.fitness_score = pcl_ndt_.getFitnessScore();
    alignment_result_.trans_probability = pcl_ndt_.getTransformationProbability();
    alignment_result_.final_transform = pcl_ndt_.getFinalTransformation();
    alignment_result_.is_converged = pcl_ndt_.hasConverged();
    alignment_result_.final_num_iteration = pcl_ndt_.getFinalNumIteration();

    //
    if(registration_flag_ == 0)
    {
        std::cout << "\033[34m" << std::endl
                  << "allframe_id:" << filtered_timed_id_pointcloud_.allframe_id
                  << "  points size::" << voxel_filtered_pointcloud_ptr->size()
                  << std::endl
                  << "final_transform:"
                  << std::endl
                  << alignment_result_.final_transform
                  << "\033[0m"
                  << std::endl;

        std::cout << "\033[34m Fitness score: " << alignment_result_.fitness_score 
                  << "  trans_probability: " << alignment_result_.trans_probability 
                  << "\033[0m"<< std::endl;
    }
    else if(registration_flag_ == 1)
    {
        std::cout << "\033[32m" 
                  << "allframe_id:" << filtered_timed_id_pointcloud_.allframe_id
                  << "  points size::" << voxel_filtered_pointcloud_ptr->size()
                  << std::endl
                  << "final_transform:"
                  << std::endl
                  << alignment_result_.final_transform
                  << "\033[0m"
                  << std::endl;

        std::cout << "\033[32m Fitness score: " << alignment_result_.fitness_score 
                  << "  trans_probability: " << alignment_result_.trans_probability 
                  << "\033[0m"<< std::endl;
    }

}

Eigen::Matrix4f NdtRegistration::PredictPose()
{
    Eigen::Matrix4f  delta_matrix
        = alignment_result_previous_.final_transform.inverse() * alignment_result_.final_transform;

    Eigen::Matrix4f  predict_matrix = alignment_result_.final_transform * delta_matrix;

    alignment_result_previous_ = alignment_result_;

    return predict_matrix;
}

double NdtRegistration::calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double delta_rad = lhs_rad - rhs_rad;
  if(delta_rad >= M_PI)
  {
      delta_rad = delta_rad - 2 * M_PI;
  }
  else if (delta_rad < -M_PI)
  {
      delta_rad = delta_rad + 2 * M_PI;
  }

  return delta_rad;
}

// 计算量比eigen库的位姿预测算法计算量大，并且依赖tf，不建议使用
Eigen::Matrix4f NdtRegistration::PredictPoseUseTf()
{
    double delta_x = alignment_result_.final_transform(0, 3) - alignment_result_previous_.final_transform(0, 3);
    double delta_y = alignment_result_.final_transform(1, 3) - alignment_result_previous_.final_transform(1, 3);
    double delta_z = alignment_result_.final_transform(2, 3) - alignment_result_previous_.final_transform(2, 3);

    tf::Matrix3x3 mat_previous, mat_current;

    mat_previous.setValue(static_cast<double>(alignment_result_previous_.final_transform(0, 0)), static_cast<double>(alignment_result_previous_.final_transform(0, 1)), static_cast<double>(alignment_result_previous_.final_transform(0, 2)),
                          static_cast<double>(alignment_result_previous_.final_transform(1, 0)), static_cast<double>(alignment_result_previous_.final_transform(1, 1)), static_cast<double>(alignment_result_previous_.final_transform(1, 2)),
                          static_cast<double>(alignment_result_previous_.final_transform(2, 0)), static_cast<double>(alignment_result_previous_.final_transform(2, 1)), static_cast<double>(alignment_result_previous_.final_transform(2, 2)));

    mat_current.setValue(static_cast<double>(alignment_result_.final_transform(0, 0)), static_cast<double>(alignment_result_.final_transform(0, 1)), static_cast<double>(alignment_result_.final_transform(0, 2)),
                         static_cast<double>(alignment_result_.final_transform(1, 0)), static_cast<double>(alignment_result_.final_transform(1, 1)), static_cast<double>(alignment_result_.final_transform(1, 2)),
                         static_cast<double>(alignment_result_.final_transform(2, 0)), static_cast<double>(alignment_result_.final_transform(2, 1)), static_cast<double>(alignment_result_.final_transform(2, 2)));

    double roll_current = 0, pitch_current = 0, yaw_current = 0, roll_previous = 0, pitch_previous = 0, yaw_previous = 0;
    mat_previous.getRPY(roll_previous, pitch_previous, yaw_previous, 1);
    mat_current.getRPY(roll_current, pitch_current, yaw_current, 1);

    double delta_roll = calcDiffForRadian(roll_current, roll_previous);
    double delta_pitch = calcDiffForRadian(pitch_current, pitch_previous);
    double delta_yaw = calcDiffForRadian(yaw_current, yaw_previous);

    double predict_x = alignment_result_.final_transform(0, 3) + delta_x;
    double predict_y = alignment_result_.final_transform(1, 3) + delta_y;
    double predict_z = alignment_result_.final_transform(2, 3) + delta_z;
    double predict_roll = roll_current;
    double predict_pitch = pitch_current;
    double predict_yaw = yaw_current + delta_yaw;

    Eigen::AngleAxisf init_rotation_x(predict_roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(predict_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(predict_yaw, Eigen::Vector3f::UnitZ());

    Eigen::Translation3f init_translation(predict_x, predict_y, predict_z);

    Eigen::Matrix4f predict_matrix =
          (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();


    alignment_result_previous_ = alignment_result_;

    return predict_matrix;
}


void NdtRegistration::AddKeyframeToMap()
{
    PoseType pose_current;
    Eigen::Matrix3f rotation_matrix = alignment_result_.final_transform.block<3,3>(0, 0);
    Eigen::Vector3f euler_angles = rotation_matrix.eulerAngles(2, 1, 0);

    // 经验证，这个欧拉角和tf工具中的 getRPY() 函数计算的欧拉角一致。2021-01-09:li-chunjing
    pose_current.roll = euler_angles[2];
    pose_current.pitch = euler_angles[1];
    pose_current.yaw = euler_angles[0];
    pose_current.x = alignment_result_.final_transform(0,3);
    pose_current.y = alignment_result_.final_transform(1,3);
    pose_current.z = alignment_result_.final_transform(2,3);

    double shift = sqrt(pow(pose_current.x - pose_last_keyframe_.x, 2.0)
                      + pow(pose_current.y - pose_last_keyframe_.y, 2.0));

    // added by lichunjing 2021-12-21
    // 前 20帧数据都加到地图中
    static int num_start_frames = 0;
    static bool is_start_frames = true;
    if(num_start_frames < mapping_parameter_.num_start_frames_add_to_map)
    {
        num_start_frames++;
        std::cout << "num_start_frames:" << num_start_frames << std::endl;
    }
    else
    {
        is_start_frames = false;
    }


    // if(shift >= mapping_parameter_.min_keyframe_distance)
    if((shift >= mapping_parameter_.min_keyframe_distance)||(is_start_frames))
    {
        pose_last_keyframe_ = pose_current;
        pcl::PointCloud<PointType>::Ptr transformed_pointcloud_ptr(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(filtered_timed_id_pointcloud_.pointcloud,
                                 *transformed_pointcloud_ptr,
                                 alignment_result_.final_transform);

        pcl_ndt_.setInputTarget(map_ptr_);
        *map_ptr_ += *transformed_pointcloud_ptr;
    }
}

void NdtRegistration::TrimMap()
{
    PoseType pose_current;
    pose_current.x = alignment_result_.final_transform(0,3);
    pose_current.y = alignment_result_.final_transform(1,3);
    pose_current.z = alignment_result_.final_transform(2,3);

    double distance = sqrt(pow(pose_current.x - pose_last_trim_.x, 2.0)
                         + pow(pose_current.y - pose_last_trim_.y, 2.0));

    // std::cout << "\033[31m distance:" << distance << "\033[0m" << std::endl;

    if(distance > mapping_parameter_.map_trim_distance)
    {
        pose_last_trim_ = pose_current;

        pcl::PassThrough<PointType> pass_through_filter;

        pcl::PointCloud<PointType>::Ptr cloud_before_pass_through_x(new pcl::PointCloud<PointType>(*map_ptr_));
        pass_through_filter.setInputCloud(cloud_before_pass_through_x);
        pass_through_filter.setFilterFieldName("x");
        pass_through_filter.setFilterLimits(pose_current.x + mapping_parameter_.pass_through_filter.x_min,
                                            pose_current.x + mapping_parameter_.pass_through_filter.x_max);
        pass_through_filter.filter(*map_ptr_);

        pcl::PointCloud<PointType>::Ptr cloud_before_pass_through_y(new pcl::PointCloud<PointType>(*map_ptr_));
        pass_through_filter.setInputCloud(cloud_before_pass_through_y);
        pass_through_filter.setFilterFieldName("y");
        pass_through_filter.setFilterLimits(pose_current.y + mapping_parameter_.pass_through_filter.y_min,
                                            pose_current.y + mapping_parameter_.pass_through_filter.y_max);
        pass_through_filter.filter(*map_ptr_);

        pcl::PointCloud<PointType>::Ptr cloud_before_pass_through_z(new pcl::PointCloud<PointType>(*map_ptr_));
        pass_through_filter.setInputCloud(cloud_before_pass_through_z);
        pass_through_filter.setFilterFieldName("z");
        pass_through_filter.setFilterLimits(pose_current.z + mapping_parameter_.pass_through_filter.z_min,
                                            pose_current.z + mapping_parameter_.pass_through_filter.z_max);
        pass_through_filter.filter(*map_ptr_);

        // std::cout << "直通滤波后点云数据点数：" << map_ptr_->points.size() << std::endl;
        // filename = "/home/alorli/test_data/after.pcd";
        // pcl::io::savePCDFileASCII(filename, *map_ptr_);
        // std::cout << "Saved " << map_ptr_->points.size() << " data points to " << filename << "." << std::endl;

    }


}



}  // namespace registration
