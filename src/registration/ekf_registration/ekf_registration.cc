///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2022_04_24
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>
#include "src/registration/ekf_registration/ekf_registration.h"

#include <tf/transform_datatypes.h>



namespace registration
{
// global_ikd_tree不能定义为类的成员变量,否则即便能编译通过,运行时会报错,运行时程序无法进入main函数就崩掉
// 这里定义成全局变量
ikd_tree::KD_TREE global_ikd_tree;

EkfRegistration::EkfRegistration(std::string cfg_file_path)
    :imu_processor_(cfg_file_path),
     ekf_processor_(cfg_file_path),
     ekf_state_(),
     alignment_result_({true, 0.0, 0.0, 0.0, 1, Eigen::Matrix4f::Identity()})
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");

    all_parameter_.lidar_process_parameter.lidar_min_range = cfg_file_["ekf_registration"]["lidar_process"]["lidar_min_range"].as<double>();
    all_parameter_.lidar_process_parameter.ring_filter_interval = cfg_file_["ekf_registration"]["lidar_process"]["ring_filter_interval"].as<int>();
    all_parameter_.imu_parameter.gravity_local = cfg_file_["ekf_registration"]["imu_parameter"]["gravity_local"].as<double>();
    all_parameter_.imu_parameter.covariance_acc = cfg_file_["ekf_registration"]["imu_parameter"]["covariance_acc"].as<double>();
    all_parameter_.imu_parameter.covariance_gyro = cfg_file_["ekf_registration"]["imu_parameter"]["covariance_gyro"].as<double>();
    all_parameter_.imu_parameter.covariance_bias_acc = cfg_file_["ekf_registration"]["imu_parameter"]["covariance_bias_acc"].as<double>();
    all_parameter_.imu_parameter.covariance_bias_gyro = cfg_file_["ekf_registration"]["imu_parameter"]["covariance_bias_gyro"].as<double>();
    all_parameter_.extrinsic_lidar_in_imu.translation = cfg_file_["ekf_registration"]["extrinsic_lidar_in_imu"]["translation"].as<std::vector<double>>();
    all_parameter_.extrinsic_lidar_in_imu.rotation = cfg_file_["ekf_registration"]["extrinsic_lidar_in_imu"]["rotation"].as<std::vector<double>>();
    all_parameter_.local_map_parameter.cube_length = cfg_file_["ekf_registration"]["local_map"]["cube_length"].as<double>();
    all_parameter_.local_map_parameter.move_threshold = cfg_file_["ekf_registration"]["local_map"]["move_threshold"].as<double>();
    all_parameter_.local_map_parameter.delta_range = cfg_file_["ekf_registration"]["local_map"]["delta_range"].as<double>();
    all_parameter_.local_map_parameter.filter_size_surf = cfg_file_["ekf_registration"]["local_map"]["filter_size_surf"].as<double>();
    all_parameter_.local_map_parameter.filter_size_map = cfg_file_["ekf_registration"]["local_map"]["filter_size_map"].as<double>();

    // 初始化参数
    is_start_ = false;
    lidar_allframe_id_ = 0;
    is_lidar_prepared_ = false;
    lidar_average_scan_period_ = 0.0;
    lidar_scan_count_ = 0;
    last_lidar_time_ = common::FromUniversal(0);
    last_imu_time_ = common::FromUniversal(0);

    imu_processor_.SetImuParameter(all_parameter_.imu_parameter);
    imu_processor_.SetExtrinsic(all_parameter_.extrinsic_lidar_in_imu);

    features_array_.reset(new pcl::PointCloud<LidarPointType>());
    normal_vector_.reset(new pcl::PointCloud<LidarPointType>(100000, 1));
    features_from_map_.reset(new pcl::PointCloud<LidarPointType>());


    downsize_filter_surf_.setLeafSize(all_parameter_.local_map_parameter.filter_size_surf, 
                                      all_parameter_.local_map_parameter.filter_size_surf, 
                                      all_parameter_.local_map_parameter.filter_size_surf);

    std::cout << std::endl;
    std::cout << "\033[32m" << "----------EkfRegistration parameter:----------" << "\033[0m" << std::endl;
    std::cout << "all_parameter_.lidar_process_parameter.lidar_min_range:" << "\033[33m"  << all_parameter_.lidar_process_parameter.lidar_min_range  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.lidar_process_parameter.ring_filter_interval:" << "\033[33m"  << all_parameter_.lidar_process_parameter.ring_filter_interval  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.imu_parameter.gravity_local:" << "\033[33m"  << all_parameter_.imu_parameter.gravity_local  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.imu_parameter.covariance_acc:" << "\033[33m"  << all_parameter_.imu_parameter.covariance_acc  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.imu_parameter.covariance_gyro:" << "\033[33m"  << all_parameter_.imu_parameter.covariance_gyro  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.imu_parameter.covariance_bias_acc:" << "\033[33m"  << all_parameter_.imu_parameter.covariance_bias_acc  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.imu_parameter.covariance_bias_gyro:" << "\033[33m"  << all_parameter_.imu_parameter.covariance_bias_gyro  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.local_map_parameter.cube_length:" << "\033[33m"  << all_parameter_.local_map_parameter.cube_length  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.local_map_parameter.move_threshold:" << "\033[33m"  << all_parameter_.local_map_parameter.move_threshold  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.local_map_parameter.delta_range:" << "\033[33m"  << all_parameter_.local_map_parameter.delta_range  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.local_map_parameter.filter_size_surf:" << "\033[33m"  << all_parameter_.local_map_parameter.filter_size_surf  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.local_map_parameter.filter_size_map:" << "\033[33m"  << all_parameter_.local_map_parameter.filter_size_map  << "\033[0m" << std::endl;
}

EkfRegistration::~EkfRegistration()
{
}


void EkfRegistration::AddSensorData(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // std::cout << "------------------------------------------------------------------------------------------------------------------" << std::endl;
    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();


    common::Time current_lidar_time = common::FromRos(msg->header.stamp);

    if(current_lidar_time < last_lidar_time_)
    {
        ROS_ERROR("Warn:lidar message loop back, clear timed_id_pointcloud_buffer buffer");
        timed_id_lidar_pointcloud_buffer_.clear();
        timed_id_lidar_pointcloud_raw_compensationed_buffer_.clear();
    }
    last_lidar_time_ = current_lidar_time;

    //这里默认调用TimedIdLidarPointCloud的无参构造函数
    TimedIdLidarPointCloud timed_id_lidar_pointcloud;
    timed_id_lidar_pointcloud.time = current_lidar_time;
    timed_id_lidar_pointcloud.allframe_id = allframe_id++;

    timed_id_lidar_pointcloud_raw_.time = current_lidar_time;
    timed_id_lidar_pointcloud_raw_.allframe_id = timed_id_lidar_pointcloud.allframe_id;
    timed_id_lidar_pointcloud_raw_.pointcloud_ptr->points.clear();

    TimedIdLidarPointCloud timed_id_lidar_pointcloud_raw_compensationed;
    timed_id_lidar_pointcloud_raw_compensationed.time = current_lidar_time;
    timed_id_lidar_pointcloud_raw_compensationed.allframe_id = timed_id_lidar_pointcloud.allframe_id;
    timed_id_lidar_pointcloud_raw_compensationed.pointcloud_ptr->points.clear();

    pcl::PointCloud<velodyne::Point> velodyne_pointcloud;
    pcl::fromROSMsg(*msg, velodyne_pointcloud);

    ProcessPointCloud(velodyne_pointcloud, 
                      timed_id_lidar_pointcloud, 
                      timed_id_lidar_pointcloud_raw_compensationed);

    timed_id_lidar_pointcloud_buffer_.push_back(timed_id_lidar_pointcloud);
    timed_id_lidar_pointcloud_raw_compensationed_buffer_.push_back(timed_id_lidar_pointcloud_raw_compensationed);

    if(PrepareMeasurements())
    {
        // if(!is_start_)
        // {
            // if(measurements_.imu_data_buffer.size() >= 40)
            // {
                // is_start_ = true;
            // }
            // else
            // {
                // alignment_result_.is_converged = false;
                // return;
            // }
        // }

        if(is_first_scan_)
        {
            first_lidar_time_ = measurements_.lidar_begin_time;
            is_first_scan_ = false;
            alignment_result_.is_converged = false;
            return;
        }

 
        if(!is_start_)
        {
            // if(measurements_.timed_id_lidar_pointcloud.pointcloud_ptr->points.size() == 4984)
            if(measurements_.timed_id_lidar_pointcloud.pointcloud_ptr->points.size() == 13206)
            {
                is_start_ = true;
            }
            else
            {
                alignment_result_.is_converged = false;
                return;
            }
        }
        
        static long long frame_id = 0;
        std::cout << std::setprecision(20) << std::endl;
        std::cout << std::endl << std::endl << std::endl;
        std::cout << "------------------------------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "\033[32m" << "---------allframe_id:" << frame_id++
                  << "   lidar_points:" << measurements_.timed_id_lidar_pointcloud.pointcloud_ptr->points.size() 
                  << "\033[0m" << std::endl;



        // 显示这个雷达旋转周期内IMU的数据
        // for(int i=0; i<measurements_.imu_data_buffer.size(); i++)
        // {
        //     // std::cout << " " << measurements_.imu_data_buffer.at(i).angular_velocity[0];
        //     std::cout << std::setprecision(25) << " " << common::ToUniversalSeconds(measurements_.imu_data_buffer.at(i).time);
        // }
        // std::cout << std::endl;


        imu_processor_.Process(ekf_processor_, 
                               measurements_,
                               compensationed_features_pointcloud_,
                               timed_id_lidar_pointcloud_raw_compensationed_
                               );
        
        // EkfState ekf_state = ekf_processor_.GetEkfState();
        ekf_state_ = ekf_processor_.GetEkfState();

        lidar_position_ = ekf_state_.position + ekf_state_.rotation * ekf_state_.extrinsic_translation;

        std::cout << "------predict lidar_position_:" << lidar_position_.transpose()
                  << std::endl;

        std::cout << "----feats_undistort.size:" << compensationed_features_pointcloud_.pointcloud_ptr->points.size() << std::endl;

        if(compensationed_features_pointcloud_.pointcloud_ptr->empty())
        {
            ROS_WARN("No point, skip this scan!\n");
            alignment_result_.is_converged = false;
            return;
        }

        is_ekf_inited_ = (common::ToUniversalSeconds(measurements_.lidar_begin_time) - common::ToUniversalSeconds(first_lidar_time_)) < kEkfInitTime ? false : true;
        
        // 判断是否需要移动地图
        MoveMap();

        // 对雷达扫描帧中的特征点进行降采样
        compensationed_features_pointcloud_downsize_lidar_.time = compensationed_features_pointcloud_.time;
        compensationed_features_pointcloud_downsize_lidar_.allframe_id = compensationed_features_pointcloud_.allframe_id;
        compensationed_features_pointcloud_downsize_world_.time = compensationed_features_pointcloud_.time;
        compensationed_features_pointcloud_downsize_world_.allframe_id = compensationed_features_pointcloud_.allframe_id;

        downsize_filter_surf_.setInputCloud(compensationed_features_pointcloud_.pointcloud_ptr);
        downsize_filter_surf_.filter(*compensationed_features_pointcloud_downsize_lidar_.pointcloud_ptr);
        num_features_points_downsize_ = compensationed_features_pointcloud_downsize_lidar_.pointcloud_ptr->points.size();

        // 初始化地图 kdtree,只执行一次
        if(global_ikd_tree.Root_Node == nullptr)
        {
            if(num_features_points_downsize_ > 5)
            {
                global_ikd_tree.set_downsample_param(all_parameter_.local_map_parameter.filter_size_map);
                compensationed_features_pointcloud_downsize_world_.pointcloud_ptr->resize(num_features_points_downsize_);

                math::PointcloudLidarToWorld(ekf_state_, 
                                             compensationed_features_pointcloud_downsize_lidar_, 
                                             compensationed_features_pointcloud_downsize_world_);

                global_ikd_tree.Build(compensationed_features_pointcloud_downsize_world_.pointcloud_ptr->points);
            }
            alignment_result_.is_converged = false;
            return;
        }

        int features_from_map_num = global_ikd_tree.validnum();

        std::cout << "------------features_from_map_num:" << features_from_map_num << std::endl;

        /*** ICP 和 EKF 更新 ***/
        if(num_features_points_downsize_ < 5)
        {
            ROS_WARN("No point, skip this scan!\n");
            alignment_result_.is_converged = false;
            return;
        }

        normal_vector_->resize(num_features_points_downsize_);
        compensationed_features_pointcloud_downsize_world_.pointcloud_ptr->resize(num_features_points_downsize_);


        // 地图可视化
        if(1) // If you need to see map point, change to "if(1)"
        {
            PointVector().swap(global_ikd_tree.PCL_Storage);
            global_ikd_tree.flatten(global_ikd_tree.Root_Node, global_ikd_tree.PCL_Storage, ikd_tree::NOT_RECORD);
            features_from_map_->clear();
            features_from_map_->points = global_ikd_tree.PCL_Storage;
        }

        nearest_points_vector_.resize(num_features_points_downsize_);

        // 使用雷达测量值作为观测值进行EKF状态更新
        ekf_processor_.UpdateState(global_ikd_tree,
                                   nearest_points_vector_,
                                   compensationed_features_pointcloud_downsize_lidar_,
                                   compensationed_features_pointcloud_downsize_world_
                                   );

        ekf_state_ = ekf_processor_.GetEkfState();

        // 雷达位姿是以imu初始位姿为参考的，例如当初始时刻 ekf_state_.position和ekf_state_.rotation都是0时，算出来的雷达位姿就是雷达-imu外参，也就是雷达在imu坐标系中的位姿
        lidar_position_ = ekf_state_.position + ekf_state_.rotation * ekf_state_.extrinsic_translation;

        std::cout << "------update lidar_position:" << lidar_position_.transpose()
                  << std::endl;

        // 保存配准结果：
        std::chrono::steady_clock::time_point time_now = std::chrono::steady_clock::now();
        std::chrono::duration<double> delta_time
            = std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_start);

        Eigen::Quaterniond lidar_rotation = ekf_state_.rotation * ekf_state_.extrinsic_rotation;  
        Eigen::Matrix3d lidar_rotation_matrix = lidar_rotation.matrix();
        Eigen::Matrix4f lidar_transformation = Eigen::Matrix4f::Identity();
        lidar_transformation(0,0) = static_cast<float>(lidar_rotation_matrix(0,0)); 
        lidar_transformation(0,1) = static_cast<float>(lidar_rotation_matrix(0,1)); 
        lidar_transformation(0,2) = static_cast<float>(lidar_rotation_matrix(0,2)); 
        lidar_transformation(0,3) = static_cast<float>(lidar_position_(0));

        lidar_transformation(1,0) = static_cast<float>(lidar_rotation_matrix(1,0)); 
        lidar_transformation(1,1) = static_cast<float>(lidar_rotation_matrix(1,1)); 
        lidar_transformation(1,2) = static_cast<float>(lidar_rotation_matrix(1,2)); 
        lidar_transformation(1,3) = static_cast<float>(lidar_position_(1));

        lidar_transformation(2,0) = static_cast<float>(lidar_rotation_matrix(2,0)); 
        lidar_transformation(2,1) = static_cast<float>(lidar_rotation_matrix(2,1)); 
        lidar_transformation(2,2) = static_cast<float>(lidar_rotation_matrix(2,2)); 
        lidar_transformation(2,3) = static_cast<float>(lidar_position_(2));

        alignment_result_.is_converged = true;
        alignment_result_.time_duration_ms = delta_time.count()*1000.0;
        alignment_result_.fitness_score = 1.0;
        alignment_result_.final_num_iteration = 1;
        alignment_result_.final_transform = lidar_transformation;
        
        UpdateMap();
    }
    // else
    // {
        // alignment_result_.is_converged = false;
    // }


    std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> delta_time
        = std::chrono::duration_cast<std::chrono::duration<double>>(time_end - time_start);
    
    // std::cout << "--------------delta_time:" << delta_time.count()*1000.0 << std::endl;
}

void EkfRegistration::AddSensorData(const sensor::ImuData& imu_data)
{
    if(imu_data.time < last_imu_time_)
    {
        ROS_ERROR("Warn:imu message loop back, clear imu_data_buffer_ buffer");
        imu_data_buffer_.clear();
    }
    last_imu_time_ = imu_data.time;

    imu_data_buffer_.push_back(imu_data);
}

void EkfRegistration::ProcessPointCloud(pcl::PointCloud<velodyne::Point>& velodyne_pointcloud,
                                        TimedIdLidarPointCloud& timed_id_lidar_pointcloud,
                                        TimedIdLidarPointCloud& timed_id_lidar_pointcloud_raw_compensationed)
{
    bool is_1st = true;
    int N_SCANS   = 16;
    int SCAN_RATE = 10;
    bool given_offset_time = false;

    /*** These variables only works when no point timestamps given ***/
    // 1Hz 1s转360度 1ms转0.36度；10Hz 1s转360×10度 1ms转0.36*10度
    // double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    double omega_l = 360.0 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS, true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time

    int points_size = velodyne_pointcloud.points.size();

    // 假如提供了每个点的时间戳
    if(velodyne_pointcloud.points[points_size - 1].time > 0)
    {
        given_offset_time = true;  // 提供时间偏移
    }
    else   // 没有提供每个点的时间戳
    {
        given_offset_time = false;
        double yaw_first = atan2(velodyne_pointcloud.points[0].y, velodyne_pointcloud.points[0].x) * 57.29578;
        double yaw_end  = yaw_first;
        int layer_first = velodyne_pointcloud.points[0].ring;
        for(uint i = points_size - 1; i > 0; i--)
        {
            if(velodyne_pointcloud.points[i].ring == layer_first)
            {
                yaw_end = atan2(velodyne_pointcloud.points[i].y, velodyne_pointcloud.points[i].x) * 57.29578;   //寻找终止yaw角
                break;
            }
        }
    }

    for(int i=0; i<velodyne_pointcloud.points.size(); i++)
    {
        LidarPointType lidar_point;
        
        lidar_point.normal_x = 0;
        lidar_point.normal_y = 0;
        lidar_point.normal_z = 0;
        lidar_point.x = velodyne_pointcloud.points.at(i).x;
        lidar_point.y = velodyne_pointcloud.points.at(i).y;
        lidar_point.z = velodyne_pointcloud.points.at(i).z;
        lidar_point.intensity = velodyne_pointcloud.points.at(i).intensity;
        lidar_point.curvature = velodyne_pointcloud.points.at(i).time;

        
        // if(is_1st)
        // {
        //     is_1st = false;
        //     std::cout << std::setprecision(20) 
        //               << "plsize:" << velodyne_pointcloud.points.size()
        //               << "  lidar_point.curvature:" << lidar_point.curvature
        //               << "  velodyne_pointcloud.points.at(i).time:" << velodyne_pointcloud.points.at(i).time
        //               << std::endl;
        // }

        if(!given_offset_time)
        {
            int layer = velodyne_pointcloud.points[i].ring;
            double yaw_angle = atan2(lidar_point.y, lidar_point.x) * 57.2957;
            if(is_first[layer])
            {
                // printf("layer: %d; is first: %d", layer, is_first[layer]);
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                lidar_point.curvature = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = lidar_point.curvature;
                continue;
            }
            // compute offset time
            if(yaw_angle <= yaw_fp[layer])
            {
                lidar_point.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
            }
            else
            {
                lidar_point.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
            }
            if (lidar_point.curvature < time_last[layer])  
            {
                lidar_point.curvature+=360.0/omega_l;
            }

            yaw_last[layer] = yaw_angle;
            time_last[layer] = lidar_point.curvature;
        }

        timed_id_lidar_pointcloud_raw_.pointcloud_ptr->points.push_back(lidar_point);
        timed_id_lidar_pointcloud_raw_compensationed.pointcloud_ptr->points.push_back(lidar_point);

        if(i % all_parameter_.lidar_process_parameter.ring_filter_interval == 0)
        {
           if(lidar_point.x*lidar_point.x + lidar_point.y*lidar_point.y + lidar_point.z*lidar_point.z 
               > (all_parameter_.lidar_process_parameter.lidar_min_range * all_parameter_.lidar_process_parameter.lidar_min_range))
           {
              timed_id_lidar_pointcloud.pointcloud_ptr->points.push_back(lidar_point);
           }
        }
    }

    // std::cout << "------origin.points.size:" << velodyne_pointcloud.points.size() 
            //   << " filtered.points.size:" << timed_id_lidar_pointcloud.pointcloud_ptr->points.size() 
            //   << std::endl;
}

bool EkfRegistration::PrepareMeasurements()
{
    std::cout << "----timed_id_lidar_pointcloud_buffer_.size():" << timed_id_lidar_pointcloud_buffer_.size() 
              << "  imu_data_buffer_.size():" << imu_data_buffer_.size() 
              << std::endl;

    if(timed_id_lidar_pointcloud_buffer_.empty() || imu_data_buffer_.empty()) 
    {
        return false;
    }    

    if(!is_lidar_prepared_)
    {
        measurements_.timed_id_lidar_pointcloud = timed_id_lidar_pointcloud_buffer_.front();
        measurements_.timed_id_lidar_pointcloud_raw = timed_id_lidar_pointcloud_raw_compensationed_buffer_.front();
        measurements_.lidar_end_time = measurements_.timed_id_lidar_pointcloud.time;     //实际采集vlp16时间戳是雷达扫描结束时的时间戳

        if(measurements_.timed_id_lidar_pointcloud.pointcloud_ptr->points.size() <= 1)
        {
            measurements_.lidar_begin_time = measurements_.lidar_end_time + common::FromSeconds(lidar_average_scan_period_);
            ROS_WARN("Too few points!\n");
        }
        // 雷达有可能在空旷环境中返回点比较少，有可能不能通过雷达最后一个点的时间戳来确定雷达扫描的结束时间戳
        else if(measurements_.timed_id_lidar_pointcloud.pointcloud_ptr->points.back().curvature < 0.5 * lidar_average_scan_period_)
        {
            measurements_.lidar_begin_time = measurements_.lidar_end_time + common::FromSeconds(lidar_average_scan_period_);
        }
        else
        {
            // std::cout << "lidar_average_scan_period_:" << lidar_average_scan_period_
                    //   << "  scan_1st_time:" << measurements_.timed_id_lidar_pointcloud.pointcloud_ptr->points.front().curvature
                    //   << "  scan_last_time:" << measurements_.timed_id_lidar_pointcloud.pointcloud_ptr->points.back().curvature
                    //   << std::endl;

            lidar_scan_count_++;
            measurements_.lidar_begin_time 
                = measurements_.lidar_end_time + common::FromSeconds(measurements_.timed_id_lidar_pointcloud.pointcloud_ptr->points.front().curvature);

            lidar_average_scan_period_ += (measurements_.timed_id_lidar_pointcloud.pointcloud_ptr->points.front().curvature - lidar_average_scan_period_) / lidar_scan_count_;
        }

        is_lidar_prepared_ = true;        
    }

    // std::cout << std::setprecision(40) 
            //   <<  "------delta_time:" << common::ToUniversalSeconds(measurements_.lidar_end_time) - common::ToUniversalSeconds(measurements_.lidar_begin_time)
            //   <<  "  points.front_time:" << measurements_.timed_id_lidar_pointcloud.pointcloud_ptr->points.front().curvature
            //   <<  "  lidar_start_time:" << common::ToUniversalSeconds(measurements_.lidar_begin_time)
            //   <<  "  --lidar_start_time:" << common::ToUniversalSeconds(measurements_.lidar_end_time) + measurements_.timed_id_lidar_pointcloud.pointcloud_ptr->points.front().curvature
            //   <<  "  lidar_end_time:" << common::ToUniversalSeconds(measurements_.lidar_end_time)
            //   <<  "  lidar_mean_scantime:" << lidar_average_scan_period_
            //   <<  "  lidar_scan_count_:" << lidar_scan_count_ 
            //   << std::endl;

   // 如果IMU数据还没有充满整个雷达的扫描周期
   if(last_imu_time_ < measurements_.lidar_end_time)
   {
       return false;
   }

   common::Time imu_time = imu_data_buffer_.front().time;
   measurements_.imu_data_buffer.clear();

   while((!imu_data_buffer_.empty()) && (imu_time < measurements_.lidar_end_time))
   {
       imu_time = imu_data_buffer_.front().time;

    //    if(imu_time > measurements_.lidar_end_time) 
       if(common::ToUniversalSeconds(imu_time) > common::ToUniversalSeconds(measurements_.lidar_end_time))  
       {
           break;
       }

       measurements_.imu_data_buffer.push_back(imu_data_buffer_.front());
       imu_data_buffer_.pop_front();
   }

   std::cout << "measurements_.imu_data_buffer.size:" << measurements_.imu_data_buffer.size() << std::endl;

   timed_id_lidar_pointcloud_buffer_.pop_front();
   timed_id_lidar_pointcloud_raw_compensationed_buffer_.pop_front();
   is_lidar_prepared_ = false;

   return true;
}

void EkfRegistration::MoveMap()
{
    cube_need_remove_.clear();
    kdtree_delete_counter_ = 0;

    if(!is_local_map_initialized_)
    {
        // LocalMap是以雷达位置为中心点，以 cube_len为边长的正方体
        for(int i = 0; i < 3; i++)
        {
            local_map_points_.vertex_min[i] = lidar_position_(i) - all_parameter_.local_map_parameter.cube_length / 2.0;
            local_map_points_.vertex_max[i] = lidar_position_(i) + all_parameter_.local_map_parameter.cube_length / 2.0;
        }
        is_local_map_initialized_ = true;

        return;
    } 

    // 判断是否需要移动地图
    float distance_to_local_map_edge[3][2];
    bool is_need_move = false;
    for(int i = 0; i < 3; i++)
    {
        // 雷达距离地图左下角的坐标差值
        distance_to_local_map_edge[i][0] = fabs(lidar_position_(i) - local_map_points_.vertex_min[i]);

        // 雷达距离地图左下角的坐标差值
        distance_to_local_map_edge[i][1] = fabs(lidar_position_(i) - local_map_points_.vertex_max[i]);

        if(distance_to_local_map_edge[i][0] <= all_parameter_.local_map_parameter.move_threshold * all_parameter_.local_map_parameter.delta_range 
        || distance_to_local_map_edge[i][1] <= all_parameter_.local_map_parameter.move_threshold * all_parameter_.local_map_parameter.delta_range) 
        {
            is_need_move = true;
        }
    }

    if(!is_need_move) 
    {
        return;
    }   

    // 如果需要移动地图
    ikd_tree::BoxPointType new_local_map_points, temp_boxpoints;
    new_local_map_points = local_map_points_;

    // 计算移动距离
    float move_distance = std::max((all_parameter_.local_map_parameter.cube_length - 2.0 * all_parameter_.local_map_parameter.move_threshold * all_parameter_.local_map_parameter.delta_range) * 0.5 * 0.9, 
                              double(all_parameter_.local_map_parameter.delta_range * (all_parameter_.local_map_parameter.move_threshold -1)));

    std::cout << "------------move_distance:" << move_distance << std::endl;

    for(int i = 0; i < 3; i++)
    {
        temp_boxpoints = local_map_points_;
        if(distance_to_local_map_edge[i][0] <= all_parameter_.local_map_parameter.move_threshold * all_parameter_.local_map_parameter.delta_range)
        {
            new_local_map_points.vertex_max[i] -= move_distance;
            new_local_map_points.vertex_min[i] -= move_distance;
            temp_boxpoints.vertex_min[i] = local_map_points_.vertex_max[i] - move_distance;
            cube_need_remove_.push_back(temp_boxpoints);
        } 
        else if(distance_to_local_map_edge[i][1] <= all_parameter_.local_map_parameter.move_threshold * all_parameter_.local_map_parameter.delta_range)
        {
            new_local_map_points.vertex_max[i] += move_distance;
            new_local_map_points.vertex_min[i] += move_distance;
            temp_boxpoints.vertex_max[i] = local_map_points_.vertex_min[i] + move_distance;
            cube_need_remove_.push_back(temp_boxpoints);
        }
    }

    // 更新地图点
    local_map_points_ = new_local_map_points;

    PointVector points_history;
    global_ikd_tree.acquire_removed_points(points_history);

    for(int i = 0; i < points_history.size(); i++) 
    {
        features_array_->push_back(points_history[i]);
    }

    if(cube_need_remove_.size() > 0)
    {
        kdtree_delete_counter_ = global_ikd_tree.Delete_Point_Boxes(cube_need_remove_);
    }

}


void EkfRegistration::UpdateMap()
{
    PointVector point_to_add;
    PointVector point_no_need_downsample;

    point_to_add.reserve(num_features_points_downsize_);
    point_no_need_downsample.reserve(num_features_points_downsize_);

    // EkfState ekf_state = ekf_processor_.GetEkfState();
    ekf_state_ = ekf_processor_.GetEkfState();
    math::PointcloudLidarToWorld(ekf_state_, 
                                 compensationed_features_pointcloud_downsize_lidar_, 
                                 compensationed_features_pointcloud_downsize_world_);

    for(int i = 0; i < num_features_points_downsize_; i++)
    {
        if(!nearest_points_vector_.at(i).empty() && is_ekf_inited_)
        {
            const PointVector &points_near = nearest_points_vector_.at(i);

            bool is_need_add = true;
            ikd_tree::BoxPointType box_of_point;
            LidarPointType downsample_result, middle_point; 

            const double filter_size_map = all_parameter_.local_map_parameter.filter_size_map;
            middle_point.x = floor(compensationed_features_pointcloud_downsize_world_.pointcloud_ptr->points.at(i).x/filter_size_map)*filter_size_map 
                           + 0.5 * filter_size_map;
            middle_point.y = floor(compensationed_features_pointcloud_downsize_world_.pointcloud_ptr->points.at(i).y/filter_size_map)*filter_size_map 
                           + 0.5 * filter_size_map;
            middle_point.z = floor(compensationed_features_pointcloud_downsize_world_.pointcloud_ptr->points.at(i).z/filter_size_map)*filter_size_map 
                           + 0.5 * filter_size_map;

            double distance  = math::CalculateLidarPointDistance(compensationed_features_pointcloud_downsize_world_.pointcloud_ptr->points.at(i), 
                                                                 middle_point);

            if(fabs(points_near[0].x - middle_point.x) > 0.5 * filter_size_map 
            && fabs(points_near[0].y - middle_point.y) > 0.5 * filter_size_map 
            && fabs(points_near[0].z - middle_point.z) > 0.5 * filter_size_map)
            {
                point_no_need_downsample.push_back(compensationed_features_pointcloud_downsize_world_.pointcloud_ptr->points.at(i));
                continue;
            }

            for(int readd_i = 0; readd_i < KNumPointsEstimatePlane; readd_i ++)
            {
                if(points_near.size() < KNumPointsEstimatePlane) 
                {
                    break;
                }

                if(math::CalculateLidarPointDistance(points_near.at(readd_i), middle_point) < distance)
                {
                    is_need_add = false;
                    break;
                }
            }
            
            if(is_need_add)
            {
                point_to_add.push_back(compensationed_features_pointcloud_downsize_world_.pointcloud_ptr->points.at(i));
            } 
        }
        else
        {
            point_to_add.push_back(compensationed_features_pointcloud_downsize_world_.pointcloud_ptr->points.at(i));
        }
    }

    int add_point_size = global_ikd_tree.Add_Points(point_to_add, true);
    global_ikd_tree.Add_Points(point_no_need_downsample, false); 

    add_point_size = point_to_add.size() + point_no_need_downsample.size();
}

void EkfRegistration::GetTimedIdPointCloudRaw(TimedIdPointCloud& timed_id_pointcloud_raw)
{
    timed_id_pointcloud_raw.time = timed_id_lidar_pointcloud_raw_.time;
    timed_id_pointcloud_raw.allframe_id = timed_id_lidar_pointcloud_raw_.allframe_id;
    timed_id_pointcloud_raw.pointcloud.points.clear();

    for(auto iterator_point = timed_id_lidar_pointcloud_raw_.pointcloud_ptr->points.begin(); 
        iterator_point < timed_id_lidar_pointcloud_raw_.pointcloud_ptr->points.end();
        iterator_point++)
    {
        PointType point;
        point.x = iterator_point->x;
        point.y = iterator_point->y;
        point.z = iterator_point->z;
        point.intensity = iterator_point->intensity;

        timed_id_pointcloud_raw.pointcloud.points.push_back(point);
    }
}

/*
void EkfRegistration::GetTimedIdPointCloudRawCompensationed(TimedIdPointCloud& timed_id_pointcloud_raw_compensationed)
{
    timed_id_pointcloud_raw_compensationed.time = timed_id_lidar_pointcloud_raw_compensationed_.time;
    timed_id_pointcloud_raw_compensationed.allframe_id = timed_id_lidar_pointcloud_raw_compensationed_.allframe_id;
    timed_id_pointcloud_raw_compensationed.pointcloud.points.clear();

    for(auto iterator_point = timed_id_lidar_pointcloud_raw_compensationed_.pointcloud_ptr->points.begin(); 
        iterator_point < timed_id_lidar_pointcloud_raw_compensationed_.pointcloud_ptr->points.end();
        iterator_point++)
    {
        PointType point;
        point.x = iterator_point->x;
        point.y = iterator_point->y;
        point.z = iterator_point->z;
        point.intensity = iterator_point->intensity;

        timed_id_pointcloud_raw_compensationed.pointcloud.points.push_back(point);
    }
}
*/

void EkfRegistration::GetTimedIdPointCloudRawCompensationed(TimedIdPointCloud& timed_id_pointcloud_raw_compensationed)
{
    timed_id_pointcloud_raw_compensationed.time = compensationed_features_pointcloud_.time;
    timed_id_pointcloud_raw_compensationed.allframe_id = compensationed_features_pointcloud_.allframe_id;
    timed_id_pointcloud_raw_compensationed.pointcloud.points.clear();

    for(auto iterator_point = compensationed_features_pointcloud_.pointcloud_ptr->points.begin(); 
        iterator_point < compensationed_features_pointcloud_.pointcloud_ptr->points.end();
        iterator_point++)
    {
        PointType point;
        point.x = iterator_point->x;
        point.y = iterator_point->y;
        point.z = iterator_point->z;
        point.intensity = iterator_point->intensity;

        timed_id_pointcloud_raw_compensationed.pointcloud.points.push_back(point);
    }
}

}  // namespace registration
