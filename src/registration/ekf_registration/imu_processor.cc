///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2022_04_25
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>
#include "src/registration/ekf_registration/imu_processor.h"
#include <tf/transform_datatypes.h>



namespace registration
{

const bool TimeSort(LidarPointType &x, LidarPointType &y) 
{
  return (x.curvature < y.curvature);
}

ImuProcessor::ImuProcessor(std::string cfg_file_path)
    :is_first_frame_(true),
     is_imu_initialized_(false)
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");

    imu_parameter_.gravity_local = cfg_file_["ekf_registration"]["imu_parameter"]["gravity_local"].as<double>();
    imu_parameter_.init_count_max = cfg_file_["ekf_registration"]["imu_parameter"]["init_count_max"].as<int>();
    

    init_count_ = 1;
    Q_ = Eigen::Matrix<double, 12, 12>::Zero();
    Q_.block<3, 3>(0, 0).diagonal() = 0.0001*Eigen::Vector3d::Identity();       //noise_gyro
    Q_.block<3, 3>(3, 3).diagonal() = 0.0001*Eigen::Vector3d::Identity();       //noise_acc
    Q_.block<3, 3>(6, 6).diagonal() = 0.00001*Eigen::Vector3d::Identity();      //noise_bias_gyro
    Q_.block<3, 3>(9, 9).diagonal() = 0.00001*Eigen::Vector3d::Identity();      //noise_bias_acc

    covariance_acc_ = Eigen::Vector3d(0.1, 0.1, 0.1);
    covariance_gyro_ = Eigen::Vector3d(0.1, 0.1, 0.1);

    covariance_bias_gyro_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
    covariance_bias_acc_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);

    average_acc_ = Eigen::Vector3d(0.0, 0.0, -1.0);
    average_gyro_ = Eigen::Vector3d(0.0, 0.0, 0.0);

    translation_lidar_in_imu_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    rotation_lidar_in_imu_ = Eigen::Matrix3d::Identity();

    std::cout << std::endl;
    std::cout << "\033[32m" << "----------ImuProcessor parameter:----------" << "\033[0m" << std::endl;
    std::cout << "imu_parameter_.gravity_local:" << "\033[33m"  << imu_parameter_.gravity_local  << "\033[0m" << std::endl;
    std::cout << "imu_parameter_.init_count_max:" << "\033[33m"  << imu_parameter_.init_count_max  << "\033[0m" << std::endl;
}

ImuProcessor::~ImuProcessor()
{
}

void ImuProcessor::ImuReset() 
{
    average_acc_ = Eigen::Vector3d(0.0, 0.0, -1.0);
    average_gyro_ = Eigen::Vector3d(0.0, 0.0, 0.0);

    // angvel_last = Zero3d;
    is_imu_initialized_ = false;
    // start_timestamp_ = -1;
    init_count_ = 1;
}

void ImuProcessor::SetImuParameter(ImuParameter& imu_parameter)
{
    covariance_acc_scale_ = Eigen::Vector3d(imu_parameter.covariance_acc, 
                                            imu_parameter.covariance_acc, 
                                            imu_parameter.covariance_acc);
    covariance_gyro_scale_ = Eigen::Vector3d(imu_parameter.covariance_gyro, 
                                             imu_parameter.covariance_gyro, 
                                             imu_parameter.covariance_gyro);
    covariance_bias_acc_ = Eigen::Vector3d(imu_parameter.covariance_bias_acc, 
                                           imu_parameter.covariance_bias_acc, 
                                           imu_parameter.covariance_bias_acc);
    covariance_bias_gyro_ = Eigen::Vector3d(imu_parameter.covariance_bias_gyro, 
                                            imu_parameter.covariance_bias_gyro, 
                                            imu_parameter.covariance_bias_gyro);
}

void ImuProcessor::SetExtrinsic(ExtrinsicLidarInImu& extrinsic_lidar_in_imu)
{
    translation_lidar_in_imu_ << extrinsic_lidar_in_imu.translation[0], extrinsic_lidar_in_imu.translation[1], extrinsic_lidar_in_imu.translation[2];
    rotation_lidar_in_imu_ << extrinsic_lidar_in_imu.rotation[0], extrinsic_lidar_in_imu.rotation[1], extrinsic_lidar_in_imu.rotation[2],
                              extrinsic_lidar_in_imu.rotation[3], extrinsic_lidar_in_imu.rotation[4], extrinsic_lidar_in_imu.rotation[5],
                              extrinsic_lidar_in_imu.rotation[6], extrinsic_lidar_in_imu.rotation[7], extrinsic_lidar_in_imu.rotation[8];
}

void ImuProcessor::ImuIntialize(EkfProcessor& ekf_processor,
                                Measurements& measurements,
                                int& num_measurements)
{
  Eigen::Vector3d current_acc, current_gyro;
  
  if(is_first_frame_)
  {
    ImuReset();
    num_measurements = 1;
    is_first_frame_ = false;

    const auto &linear_acceleration = measurements.imu_data_buffer.front().linear_acceleration;
    const auto &angular_velocity = measurements.imu_data_buffer.front().angular_velocity;

    average_acc_ << linear_acceleration[0], linear_acceleration[1], linear_acceleration[2];
    average_gyro_ << angular_velocity[0], angular_velocity[1], angular_velocity[2];
  }

  for(const auto &imu_data : measurements.imu_data_buffer)
  {
    const auto &linear_acceleration = imu_data.linear_acceleration;
    const auto &angular_velocity = imu_data.angular_velocity;

    current_acc << linear_acceleration[0], linear_acceleration[1], linear_acceleration[2];
    current_gyro << angular_velocity[0], angular_velocity[1], angular_velocity[2];

    average_acc_ += (current_acc - average_acc_) / num_measurements;
    average_gyro_ += (current_gyro - average_gyro_) / num_measurements;

    covariance_acc_ = covariance_acc_ * (num_measurements - 1.0) / num_measurements 
                    + (current_acc - average_acc_).cwiseProduct(current_acc - average_acc_) * (num_measurements - 1.0) / (num_measurements * num_measurements);
    covariance_gyro_ = covariance_gyro_ * (num_measurements - 1.0) / num_measurements 
                    + (current_gyro - average_gyro_).cwiseProduct(current_gyro - average_gyro_) * (num_measurements - 1.0) / (num_measurements * num_measurements);

    num_measurements++;
  }

  std::cout << "----num_measurements:" << num_measurements << std::endl;
  std::cout << "----average_acc_:" << average_acc_[0] << " " << average_acc_[1] << " " << average_acc_[2] << " " << std::endl;
  std::cout << "----average_gyro_:" << average_gyro_[0] << " " << average_gyro_[1] << " " << average_gyro_[2] << " " << std::endl;
  std::cout << "----covariance_acc_:" << covariance_acc_[0] << " " << covariance_acc_[1] << " " << covariance_acc_[2] << " " << std::endl;
  std::cout << "----covariance_gyro_:" << covariance_gyro_[0] << " " << covariance_gyro_[1] << " " << covariance_gyro_[2] << " " << std::endl;


  EkfState ekf_state = ekf_processor.GetEkfState();
  ekf_state.gravity = - average_acc_ / average_acc_.norm() * imu_parameter_.gravity_local;
  ekf_state.gravity = ekf_state.gravity/ekf_state.gravity.norm() * kGravityBase;

  
  ekf_state.bias_gyro  = average_gyro_;        //初始的陀螺漂移使用初始20帧雷达数据对应的IMU数据进行估算
  ekf_state.extrinsic_translation = translation_lidar_in_imu_;
  ekf_state.extrinsic_rotation = Eigen::Quaterniond(rotation_lidar_in_imu_);
  ekf_processor.SetEkfState(ekf_state);

//   std::cout << "----ekf_state.gravity:" << ekf_state.gravity.transpose() << std::endl;
//   std::cout << "----ekf_state.rotation:" << ekf_state.rotation.w() << " " << ekf_state.rotation.x() << " " << ekf_state.rotation.y() << std::endl;
//   std::cout << "----ekf_state.position:" << ekf_state.position.transpose() << std::endl;
//   std::cout << "----ekf_state.velocity:" << ekf_state.velocity.transpose() << std::endl;
//   std::cout << "----ekf_state.bias_acc:" << ekf_state.bias_acc.transpose() << std::endl;
//   std::cout << "----ekf_state.bias_gyro:" << ekf_state.bias_gyro.transpose() << std::endl;
//   std::cout << "----ekf_state.extrinsic_rotation:" << ekf_state.extrinsic_rotation.w() << " " << ekf_state.extrinsic_rotation.x() << " " << ekf_state.extrinsic_rotation.y() << std::endl;
//   std::cout << "----ekf_state.extrinsic_translation:" << ekf_state.extrinsic_translation.transpose() << std::endl;

  Eigen::Matrix<double, 23, 23> process_covariance
    = ekf_processor.GetProcessCovariance();

  process_covariance.setIdentity();
  process_covariance(18,18) = process_covariance(19,19) = process_covariance(20,20) = 0.001;          //bias_acc
  process_covariance(15,15) = process_covariance(16,16) = process_covariance(17,17) = 0.0001;         //bias_gyro
  process_covariance(21,21) = process_covariance(22,22) = 0.00001;                                    //gravity
  process_covariance(6,6) = process_covariance(7,7) = process_covariance(8,8) = 0.00001;              //extrinsic_rotation
  process_covariance(9,9) = process_covariance(10,10) = process_covariance(11,11) = 0.00001;          //extrinsic_translation

//   std::cout << "----process_covariance:" << process_covariance << std::endl;

  ekf_processor.SetProcessCovariance(process_covariance);

  last_imu_data_ = measurements.imu_data_buffer.back();
}

void ImuProcessor::LidarMotionCompensation(EkfProcessor& ekf_processor,
                                           Measurements& measurements,
                                           TimedIdLidarPointCloud& compensationed_pointcloud,
                                           TimedIdLidarPointCloud& timed_id_lidar_pointcloud_raw_compensationed
                                          )
{
    auto imu_buffer = measurements.imu_data_buffer;
    imu_buffer.push_front(last_imu_data_);

    const common::Time &imu_begin_time = imu_buffer.front().time;
    const common::Time &imu_end_time = imu_buffer.back().time;

    const common::Time &pointcloud_begin_time = measurements.lidar_begin_time;
    const common::Time &pointcloud_end_time = measurements.lidar_end_time;

    compensationed_pointcloud = measurements.timed_id_lidar_pointcloud;

    sort(compensationed_pointcloud.pointcloud_ptr->points.begin(), 
         compensationed_pointcloud.pointcloud_ptr->points.end(), 
         TimeSort);

    // std::cout << std::setprecision(35)
    //           << "----------imu_buffer.size:" << imu_buffer.size() 
    //           << "  lidar_end_time:" << common::ToUniversalSeconds(pointcloud_end_time) 
    //           << std::endl;

    
    Eigen::Vector3d angle_velocity_average, acc_average;

    std::vector<TimedImuPose> timed_imu_poses;
    InputState input_state;
    EkfState ekf_state_init = ekf_processor.GetEkfState();

    timed_imu_poses.push_back(TimedImuPose{
                                            0.0, 
                                            acc_last_, 
                                            angle_velocity_last_, 
                                            ekf_state_init.velocity, 
                                            ekf_state_init.position, 
                                            ekf_state_init.rotation
                                           }
                             );

    for(auto imu_data = imu_buffer.begin(); imu_data < (imu_buffer.end() - 1); imu_data++)
    {
        double delta_time = 0.0;

        auto &imu_data_current = *(imu_data);
        auto &imu_data_next = *(imu_data + 1);

        // std::cout << std::setprecision(20) << "**imu_data_current.time:" << common::ToUniversalSeconds(imu_data_current.time) << std::endl;

        // std::cout << "**last_imu_data_:" << last_imu_data_.angular_velocity[0] << "  " << last_imu_data_.angular_velocity[1] << "  " << last_imu_data_.angular_velocity[2] << std::endl;
        // std::cout << "**angular_velocity:" << imu_data_current.angular_velocity[0] << "  " << imu_data_current.angular_velocity[1] << "  " << imu_data_current.angular_velocity[2] << std::endl;
        // std::cout << "**linear_acceleration:" << imu_data_current.linear_acceleration[0] << "  " << imu_data_current.linear_acceleration[1] << "  " << imu_data_current.linear_acceleration[2] << std::endl;

        if(imu_data_next.time < last_lidar_end_time_)
        {
            std::cout << "----------" << std::endl;
            continue;
        }    

        angle_velocity_average << 0.5 * (imu_data_current.angular_velocity[0] + imu_data_next.angular_velocity[0]),
                                  0.5 * (imu_data_current.angular_velocity[1] + imu_data_next.angular_velocity[1]),
                                  0.5 * (imu_data_current.angular_velocity[2] + imu_data_next.angular_velocity[2]);
                    
        acc_average << 0.5 * (imu_data_current.linear_acceleration[0] + imu_data_next.linear_acceleration[0]),
                       0.5 * (imu_data_current.linear_acceleration[1] + imu_data_next.linear_acceleration[1]),
                       0.5 * (imu_data_current.linear_acceleration[2] + imu_data_next.linear_acceleration[2]);

        // std::cout << "----angle_velocity_average:" << angle_velocity_average.transpose()
                //   << "   acc_average:" << acc_average.transpose() 
                //   << std::endl;
     
        acc_average = acc_average * imu_parameter_.gravity_local / average_acc_.norm();

        // std::cout << "----acc_average:" << acc_average.transpose()
                //   << std::endl;

        if(imu_data_current.time < last_lidar_end_time_)
        {
            delta_time = common::ToUniversalSeconds(imu_data_next.time) - common::ToUniversalSeconds(last_lidar_end_time_);
        }
        else
        {
            delta_time = common::ToUniversalSeconds(imu_data_next.time) - common::ToUniversalSeconds(imu_data_current.time);
        }

        input_state.acc = acc_average;
        input_state.gyro = angle_velocity_average;

        // 目前运行，下面四个协方差矩阵使用yaml中值配置
        Q_.block<3, 3>(0, 0).diagonal() = covariance_gyro_;           // noise_gyro
        Q_.block<3, 3>(3, 3).diagonal() = covariance_acc_;            // noise_acc
        Q_.block<3, 3>(6, 6).diagonal() = covariance_bias_gyro_;      // noise_bias_gyro
        Q_.block<3, 3>(9, 9).diagonal() = covariance_bias_acc_;       // noise_bias_acc

        ekf_processor.PredictState(delta_time, Q_, input_state);

        EkfState ekf_state = ekf_processor.GetEkfState();
        acc_last_ = ekf_state.rotation * (acc_average - ekf_state.bias_acc) + ekf_state.gravity;
        angle_velocity_last_ = angle_velocity_average - ekf_state.bias_gyro;

        // double offset_time = common::ToUniversalSeconds(imu_data_next.time) - common::ToUniversalSeconds(pointcloud_begin_time);
        double offset_time = common::ToUniversalSeconds(imu_data_next.time) - common::ToUniversalSeconds(imu_begin_time);  //测试用

        timed_imu_poses.push_back(TimedImuPose{
                                                offset_time, 
                                                acc_last_, 
                                                angle_velocity_last_, 
                                                ekf_state.velocity, 
                                                ekf_state.position, 
                                                ekf_state.rotation
                                              }
                                 );

        // std::cout << std::setprecision(20)
        //           << "-offs_t:" << offset_time
        //           << " pcl_beg_time:" << common::ToUniversalSeconds(pointcloud_begin_time) << std::endl;


        // std::cout << "--offset_time:" << offset_time
        //           << " acc_last_:" << acc_last_ << std::endl
        //           << " angle_velocity_last_:" << angle_velocity_last_ << std::endl
        //           << " velocity:" << ekf_state.velocity << std::endl
        //           << " position:" << ekf_state.position << std::endl
        //           << " rotation:" << ekf_state.rotation.toRotationMatrix() << std::endl;
    }

    last_imu_data_ = imu_buffer.back();
    last_lidar_end_time_ = pointcloud_end_time;

    
    // std::cout << "\033[032m" << "------points_size:" << compensationed_pointcloud.pointcloud_ptr->points.size() << "\033[0m" << std::endl
    //           << "\033[032m" << "  pos:" << "\033[0m" << ekf_processor.GetEkfState().position
    //           << std::endl
    //           << "\033[032m" << "  vel:" << "\033[0m" << ekf_processor.GetEkfState().velocity
    //           << std::endl
    //           << "\033[032m" << "  rot:" << "\033[0m" << ekf_processor.GetEkfState().rotation.toRotationMatrix() 
    //           << std::endl;

    double delta_time = common::ToUniversalSeconds(pointcloud_end_time) - common::ToUniversalSeconds(imu_end_time);
    if(delta_time < 0.0)
    {
      delta_time *= -1.0;
    }
    // std::cout << "delta_time:" << delta_time << std::endl;
    ekf_processor.PredictState(delta_time, Q_, input_state);
    EkfState ekf_state = ekf_processor.GetEkfState();

    // std::cout << "------pos:" << ekf_state.position
    //         << std::endl
    //         << "  vel:" << ekf_state.velocity
    //         << std::endl
    //         << "  rot:" << ekf_state.rotation.toRotationMatrix() 
    //         << std::endl;

    // 点云去畸变
    Eigen::Vector3d acc_imu, angle_velocity_imu, velocity_imu, position_imu;
    Eigen::Matrix3d rotation_imu;

    auto iterator_point = compensationed_pointcloud.pointcloud_ptr->points.end() - 1;
    auto iterator_point_raw = timed_id_lidar_pointcloud_raw_compensationed.pointcloud_ptr->points.end() - 1;
    for(auto iterator_imu_pose = timed_imu_poses.end() - 1; iterator_imu_pose != timed_imu_poses.begin(); iterator_imu_pose--)
    {
      auto imu_pose_current = iterator_imu_pose - 1;
      auto imu_pose_next = iterator_imu_pose;

      rotation_imu = imu_pose_current->rotation.toRotationMatrix();
      velocity_imu = imu_pose_current->velocity;
      position_imu = imu_pose_current->position;
      acc_imu = imu_pose_next->acc;
      angle_velocity_imu =  imu_pose_next->gyro;

      // bool is_1st = true;
      for(; iterator_point->curvature > imu_pose_current->offset_time; iterator_point--)
      {
        double delta_time = iterator_point->curvature - imu_pose_current->offset_time;
        Eigen::Matrix3d rotation_point(rotation_imu * math::Exp(angle_velocity_imu, delta_time));
        Eigen::Vector3d position_point(iterator_point->x, iterator_point->y, iterator_point->z);

        Eigen::Vector3d point_interpolate(position_imu + velocity_imu * delta_time + 0.5 * acc_imu * delta_time * delta_time - ekf_state.position);

        Eigen::Vector3d point_compensate = ekf_state.extrinsic_rotation.conjugate() 
                                        * (ekf_state.rotation.conjugate() * (rotation_point * (ekf_state.extrinsic_rotation * position_point + ekf_state.extrinsic_translation) + point_interpolate) 
                                         - ekf_state.extrinsic_translation);
        
        iterator_point->x = point_compensate(0);
        iterator_point->y = point_compensate(1);
        iterator_point->z = point_compensate(2);

        // if(is_1st)
        // {
        //   is_1st = false;
        //   std::cout << std::setprecision(25)
        //             << " delta_time:" << delta_time
        //             << " rotation_point:" << rotation_point
        //             << " position_point:" << position_point
        //             << " point_interpolate:" << point_interpolate
        //             << " point_compensate:" << point_compensate
        //             << std::endl;
        // }

        if(iterator_point == compensationed_pointcloud.pointcloud_ptr->points.begin()) 
        {
          break;
        }
      }



      for(; iterator_point_raw->curvature > imu_pose_current->offset_time; iterator_point_raw--)
      {
        double delta_time = iterator_point_raw->curvature - imu_pose_current->offset_time;
        Eigen::Matrix3d rotation_point(rotation_imu * math::Exp(angle_velocity_imu, delta_time));
        Eigen::Vector3d position_point(iterator_point_raw->x, iterator_point_raw->y, iterator_point_raw->z);

        Eigen::Vector3d point_interpolate(position_imu + velocity_imu * delta_time + 0.5 * acc_imu * delta_time * delta_time - ekf_state.position);

        Eigen::Vector3d point_compensate = ekf_state.extrinsic_rotation.conjugate() 
                                        * (ekf_state.rotation.conjugate() * (rotation_point * (ekf_state.extrinsic_rotation * position_point + ekf_state.extrinsic_translation) + point_interpolate) 
                                         - ekf_state.extrinsic_translation);
        
        iterator_point_raw->x = point_compensate(0);
        iterator_point_raw->y = point_compensate(1);
        iterator_point_raw->z = point_compensate(2);

        if(iterator_point_raw == timed_id_lidar_pointcloud_raw_compensationed.pointcloud_ptr->points.begin()) 
        {
          break;
        }
      }



    }

    auto it1 = compensationed_pointcloud.pointcloud_ptr->points.begin();
    auto it2 = compensationed_pointcloud.pointcloud_ptr->points.end()-1;

    // std::cout << "-----size:" << compensationed_pointcloud.pointcloud_ptr->points.size()
    //           << "  x1:" << it1->x << " y1:" << it1->y<< " z1:" << it1->z
    //           << "  x2:" << it2->x << " y2:" << it2->y<< " z2:" << it2->z 
    //           << std::endl;
}

void ImuProcessor::Process(EkfProcessor& ekf_processor,
                           Measurements& measurements,
                           TimedIdLidarPointCloud& compensationed_pointcloud,
                           TimedIdLidarPointCloud& timed_id_lidar_pointcloud_raw_compensationed)
{
    if(measurements.imu_data_buffer.empty() || measurements.timed_id_lidar_pointcloud.pointcloud_ptr->points.size() < 10) 
    {
        return;
    };

    if(!is_imu_initialized_)
    {
        ImuIntialize(ekf_processor, 
                     measurements, 
                     init_count_);

        if(init_count_ > imu_parameter_.init_count_max)
        {
            // cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
            is_imu_initialized_ = true;
        
            covariance_acc_ = covariance_acc_scale_;
            covariance_gyro_ = covariance_gyro_scale_;     

            last_imu_data_ = measurements.imu_data_buffer.back();       
        }

        return;
    }

    LidarMotionCompensation(ekf_processor, 
                            measurements, 
                            compensationed_pointcloud,
                            timed_id_lidar_pointcloud_raw_compensationed);


}



}  // namespace registration
