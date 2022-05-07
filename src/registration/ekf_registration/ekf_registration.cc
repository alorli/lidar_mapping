///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2022_04_24
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>
#include "src/registration/ekf_registration/ekf_registration.h"

#include <tf/transform_datatypes.h>



namespace registration
{
EkfRegistration::EkfRegistration(std::string cfg_file_path)
    :imu_processor_(cfg_file_path),
     ekf_processor_(cfg_file_path),
     ekf_state_(EkfState())
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

    std::cout << std::endl;
    std::cout << "\033[32m" << "----------EkfRegistration parameter:----------" << "\033[0m" << std::endl;
    std::cout << "all_parameter_.lidar_process_parameter.lidar_min_range:" << "\033[33m"  << all_parameter_.lidar_process_parameter.lidar_min_range  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.lidar_process_parameter.ring_filter_interval:" << "\033[33m"  << all_parameter_.lidar_process_parameter.ring_filter_interval  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.imu_parameter.gravity_local:" << "\033[33m"  << all_parameter_.imu_parameter.gravity_local  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.imu_parameter.covariance_acc:" << "\033[33m"  << all_parameter_.imu_parameter.covariance_acc  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.imu_parameter.covariance_gyro:" << "\033[33m"  << all_parameter_.imu_parameter.covariance_gyro  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.imu_parameter.covariance_bias_acc:" << "\033[33m"  << all_parameter_.imu_parameter.covariance_bias_acc  << "\033[0m" << std::endl;
    std::cout << "all_parameter_.imu_parameter.covariance_bias_gyro:" << "\033[33m"  << all_parameter_.imu_parameter.covariance_bias_gyro  << "\033[0m" << std::endl;
}

EkfRegistration::~EkfRegistration()
{
}


void EkfRegistration::AddSensorData(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    common::Time current_lidar_time = common::FromRos(msg->header.stamp);

    if(current_lidar_time < last_lidar_time_)
    {
        ROS_ERROR("Warn:lidar message loop back, clear timed_id_pointcloud_buffer buffer");
        timed_id_lidar_pointcloud_buffer_.clear();
    }
    last_lidar_time_ = current_lidar_time;


    TimedIdLidarPointCloud timed_id_lidar_pointcloud;
    timed_id_lidar_pointcloud.time = current_lidar_time;

    pcl::PointCloud<velodyne::Point> velodyne_pointcloud;
    pcl::fromROSMsg(*msg, velodyne_pointcloud);

    ProcessPointCloud(velodyne_pointcloud, timed_id_lidar_pointcloud);

    timed_id_lidar_pointcloud_buffer_.push_back(timed_id_lidar_pointcloud);

    if(PrepareMeasurements())
    {
        // if(!is_start_)
        // {
        //     if(measurements_.imu_data_buffer.size() >= 40)
        //     {
        //         is_start_ = true;
        //     }
        //     else
        //     {
        //         return;
        //     }
        // }

        if(!is_start_)
        {
            if(measurements_.timed_id_lidar_pointcloud.pointcloud.points.size() == 4984)
            {
                is_start_ = true;
            }
            else
            {
                return;
            }
        }


        std::cout << "------------------------------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "lidar_points:" << measurements_.timed_id_lidar_pointcloud.pointcloud.points.size() << std::endl;


        for(int i=0; i<measurements_.imu_data_buffer.size(); i++)
        {
            std::cout << " " << measurements_.imu_data_buffer.at(i).angular_velocity[0];
        }
        std::cout << std::endl;


        imu_processor_.Process(ekf_processor_, 
                               measurements_,
                               compensationed_pointcloud_);
        

    }
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
                                        TimedIdLidarPointCloud& timed_id_lidar_pointcloud)
{
    for(int i=0; i<velodyne_pointcloud.points.size(); i++)
    {
        LidarPointType lidar_point;
        
        lidar_point.normal_x = 0;
        lidar_point.normal_y = 0;
        lidar_point.normal_z = 0;
        lidar_point.x = velodyne_pointcloud.points[i].x;
        lidar_point.y = velodyne_pointcloud.points[i].y;
        lidar_point.z = velodyne_pointcloud.points[i].z;
        lidar_point.intensity = velodyne_pointcloud.points[i].intensity;
        lidar_point.curvature = velodyne_pointcloud.points[i].time;

        if(i % all_parameter_.lidar_process_parameter.ring_filter_interval == 0)
        {
           if(lidar_point.x*lidar_point.x + lidar_point.y*lidar_point.y + lidar_point.z*lidar_point.z 
               > (all_parameter_.lidar_process_parameter.lidar_min_range * all_parameter_.lidar_process_parameter.lidar_min_range))
           {
              timed_id_lidar_pointcloud.pointcloud.points.push_back(lidar_point);
           }
        }
    }

    std::cout << "------origin.points.size:" << velodyne_pointcloud.points.size() 
              << " filtered.points.size:" << timed_id_lidar_pointcloud.pointcloud.points.size() 
              << std::endl;
}

bool EkfRegistration::PrepareMeasurements()
{
    if(timed_id_lidar_pointcloud_buffer_.empty() || imu_data_buffer_.empty()) 
    {
        return false;
    }    

    if(!is_lidar_prepared_)
    {
        measurements_.timed_id_lidar_pointcloud = timed_id_lidar_pointcloud_buffer_.front();
        measurements_.lidar_end_time = measurements_.timed_id_lidar_pointcloud.time;     //实际采集vlp16时间戳是雷达扫描结束时的时间戳

        if(measurements_.timed_id_lidar_pointcloud.pointcloud.points.size() <= 1)
        {
            measurements_.lidar_begin_time = measurements_.lidar_end_time + common::FromSeconds(lidar_average_scan_period_);
            ROS_WARN("Too few points!\n");
        }
        // 雷达有可能在空旷环境中返回点比较少，有可能不能通过雷达最后一个点的时间戳来确定雷达扫描的结束时间戳
        else if(measurements_.timed_id_lidar_pointcloud.pointcloud.points.back().curvature < 0.5 * lidar_average_scan_period_)
        {
            measurements_.lidar_begin_time = measurements_.lidar_end_time + common::FromSeconds(lidar_average_scan_period_);
        }
        else
        {
            // std::cout << "lidar_average_scan_period_:" << lidar_average_scan_period_
                    //   << "  scan_1st_time:" << measurements_.timed_id_lidar_pointcloud.pointcloud.points.front().curvature
                    //   << "  scan_last_time:" << measurements_.timed_id_lidar_pointcloud.pointcloud.points.back().curvature
                    //   << std::endl;

            lidar_scan_count_++;
            measurements_.lidar_begin_time 
                = measurements_.lidar_end_time + common::FromSeconds(measurements_.timed_id_lidar_pointcloud.pointcloud.points.front().curvature);

            lidar_average_scan_period_ += (measurements_.timed_id_lidar_pointcloud.pointcloud.points.front().curvature - lidar_average_scan_period_) / lidar_scan_count_;
        }

        is_lidar_prepared_ = true;        
    }

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
       if(imu_time > measurements_.lidar_end_time) 
       {
           break;
       }

       measurements_.imu_data_buffer.push_back(imu_data_buffer_.front());
       imu_data_buffer_.pop_front();
   }
   
   std::cout << "measurements_.imu_data_buffer.size:" << measurements_.imu_data_buffer.size() << std::endl;

   timed_id_lidar_pointcloud_buffer_.pop_front();
   is_lidar_prepared_ = false;

   return true;
}


}  // namespace registration
