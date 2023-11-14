///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_19
///////////////////////////////////////////////////////////////////////////////

#include "src/node/lidar_mapping_node.h"
#include <iostream>

LidarMappingNode::LidarMappingNode(std::string cfg_file_path,
                                   std::string project_directory_name,
                                   ros::NodeHandle node_handle)
    : map_builder_(cfg_file_path, project_directory_name),
      node_handle_(node_handle),
      is_generate_map_(false),
      is_receive_sensor_data_(true),
      is_run_generate_map_(false),
      is_publish_pcd_map_(false),
      is_publish_pose_registration_(false),
      is_publish_generate_map_status_(false),
      is_publish_occupancy_grid_map_(false),
      is_publish_generate_map_service_(false),
      is_publish_start_builder_service_(false),
      is_publish_save_map_service_(false),
      is_exit_node_(false),
      is_get_init_encoder_(false),
      odom_data_{0.0, 0.0, 0.0}
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");

    bool is_use_lidar = cfg_file_["ros_interface"]["subscribe_message"]["is_use_lidar"].as<bool>();
    bool is_use_gnss = cfg_file_["ros_interface"]["subscribe_message"]["is_use_gnss"].as<bool>();
    bool is_use_sick = cfg_file_["ros_interface"]["subscribe_message"]["is_use_sick"].as<bool>();
    bool is_use_imu = cfg_file_["ros_interface"]["subscribe_message"]["is_use_imu"].as<bool>();
    bool is_use_odom = cfg_file_["ros_interface"]["subscribe_message"]["is_use_odom"].as<bool>();
    bool is_use_encoder = cfg_file_["ros_interface"]["subscribe_message"]["is_use_encoder"].as<bool>();

    is_generate_raw_map_ = cfg_file_["lidar_mapping_node"]["is_generate_raw_map"].as<bool>();
    is_generate_compensation_map_ = cfg_file_["lidar_mapping_node"]["is_generate_compensation_map"].as<bool>();
    is_generate_gnss_optimization_map_ = cfg_file_["lidar_mapping_node"]["is_generate_gnss_optimization_map"].as<bool>();
    is_generate_closeloop_optimization_map_ = cfg_file_["lidar_mapping_node"]["is_generate_closeloop_optimization_map"].as<bool>();
    is_generate_laserscan_map_ = cfg_file_["lidar_mapping_node"]["is_generate_laserscan_map"].as<bool>();
    is_run_gnss_aidded_optimization_ = cfg_file_["lidar_mapping_node"]["is_run_gnss_aidded_optimization"].as<bool>();
    is_run_anchor_points_optimization_ = cfg_file_["lidar_mapping_node"]["is_run_anchor_points_optimization"].as<bool>();
    is_run_closeloop_optimization_ = cfg_file_["lidar_mapping_node"]["is_run_closeloop_optimization"].as<bool>();
    is_run_map_partition_ = cfg_file_["lidar_mapping_node"]["is_run_map_partition"].as<bool>();
    is_generate_arealist_file_ = cfg_file_["lidar_mapping_node"]["is_generate_arealist_file"].as<bool>();
    is_delete_project_directory_after_save_map_ = cfg_file_["lidar_mapping_node"]["is_delete_project_directory_after_save_map"].as<bool>();


    std::string lidar_topic = cfg_file_["ros_interface"]["subscribe_message"]["lidar_topic"].as<std::string>();
    int lidar_queue_size = cfg_file_["ros_interface"]["subscribe_message"]["lidar_queue_size"].as<int>();
    std::string sick_topic = cfg_file_["ros_interface"]["subscribe_message"]["sick_topic"].as<std::string>();
    int sick_queue_size = cfg_file_["ros_interface"]["subscribe_message"]["sick_queue_size"].as<int>();
    std::string  gnss_topic = cfg_file_["ros_interface"]["subscribe_message"]["gnss_topic"].as<std::string>();
    int gnss_queue_size = cfg_file_["ros_interface"]["subscribe_message"]["gnss_queue_size"].as<int>();
    std::string imu_topic = cfg_file_["ros_interface"]["subscribe_message"]["imu_topic"].as<std::string>();
    int imu_queue_size = cfg_file_["ros_interface"]["subscribe_message"]["imu_queue_size"].as<int>();
    std::string  encoder_topic = cfg_file_["ros_interface"]["subscribe_message"]["encoder_topic"].as<std::string>();
    int encoder_queue_size = cfg_file_["ros_interface"]["subscribe_message"]["encoder_queue_size"].as<int>();
    std::string  odom_topic = cfg_file_["ros_interface"]["subscribe_message"]["odom_topic"].as<std::string>();
    int odom_queue_size = cfg_file_["ros_interface"]["subscribe_message"]["odom_queue_size"].as<int>();

    is_publish_pose_registration_ = cfg_file_["ros_interface"]["advertise_message"]["is_publish_pose_registration"].as<bool>();
    std::string pose_registration_topic = cfg_file_["ros_interface"]["advertise_message"]["pose_registration_topic"].as<std::string>();
    int pose_registration_queue_size = cfg_file_["ros_interface"]["advertise_message"]["pose_registration_queue_size"].as<int>();

    
    is_publish_generate_map_status_ = cfg_file_["ros_interface"]["advertise_message"]["is_publish_generate_map_status"].as<bool>();
    std::string generate_map_status_topic = cfg_file_["ros_interface"]["advertise_message"]["generate_map_status_topic"].as<std::string>();
    int generate_map_status_queue_size = cfg_file_["ros_interface"]["advertise_message"]["generate_map_status_queue_size"].as<int>();

    is_publish_pcd_map_ = cfg_file_["ros_interface"]["advertise_message"]["is_publish_pcd_map"].as<bool>();
    std::string  pcd_map_topic = cfg_file_["ros_interface"]["advertise_message"]["pcd_map_topic"].as<std::string>();
    int pcd_map_topic_queue_size = cfg_file_["ros_interface"]["advertise_message"]["pcd_map_topic_queue_size"].as<int>();

    is_publish_occupancy_grid_map_ = cfg_file_["ros_interface"]["advertise_message"]["occupancy_grid_map"]["is_publish_occupancy_grid_map"].as<bool>();
    std::string  occupancy_grid_map_topic = cfg_file_["ros_interface"]["advertise_message"]["occupancy_grid_map"]["occupancy_grid_map_topic"].as<std::string>();
    int occupancy_grid_map_queue_size = cfg_file_["ros_interface"]["advertise_message"]["occupancy_grid_map"]["occupancy_grid_map_queue_size"].as<int>();

    occupancy_grid_map_update_distance_ = cfg_file_["ros_interface"]["advertise_message"]["occupancy_grid_map"]["update_distance"].as<double>();

    is_publish_generate_map_service_ = cfg_file_["ros_interface"]["advertise_service"]["is_publish_generate_map_service"].as<bool>();
    std::string  generate_map_service_name = cfg_file_["ros_interface"]["advertise_service"]["generate_map_service_name"].as<std::string>();

    is_publish_start_builder_service_ = cfg_file_["ros_interface"]["advertise_service"]["is_publish_start_builder_service"].as<bool>();
    std::string  start_builder_service_name = cfg_file_["ros_interface"]["advertise_service"]["start_builder_service_name"].as<std::string>();

    is_publish_save_map_service_ = cfg_file_["ros_interface"]["advertise_service"]["is_publish_save_map_service"].as<bool>();
    std::string  save_map_service_name = cfg_file_["ros_interface"]["advertise_service"]["save_map_service_name"].as<std::string>();


    if(is_use_lidar)
    {
        pointcloud_subscriber_ = node_handle_.subscribe<sensor_msgs::PointCloud2>(lidar_topic,                // "points_raw"
                                                                                  lidar_queue_size,             // 10hz 存 27小时数据
                                                                                  &LidarMappingNode::HandlePointCloud2Message,
                                                                                  this);
    }

    if(is_use_gnss)
    {
        gnss_fix_subscriber_ = node_handle_.subscribe<sensor_msgs::NavSatFix>(gnss_topic,           // "/gps/fix"
                                                                              gnss_queue_size,
                                                                              &LidarMappingNode::HandleGnssFixMessage,
                                                                              this);
    }

    if(is_use_sick)
    {
        sick_subscriber_ = node_handle_.subscribe<sensor_msgs::LaserScan>(sick_topic,
                                                                          sick_queue_size,
                                                                          &LidarMappingNode::HandleSickMessage,
                                                                          this);
    }

    if(is_use_imu)
    {
        imu_subscriber_ = node_handle_.subscribe<sensor_msgs::Imu>(imu_topic,
                                                                   imu_queue_size,
                                                                   &LidarMappingNode::HandleImuMessage,
                                                                   this);
    }

    if(is_use_odom)
    {
        odom_subscriber_ = node_handle_.subscribe<nav_msgs::Odometry>(odom_topic,
                                                                      odom_queue_size,
                                                                      &LidarMappingNode::HandleOdomMessage,
                                                                      this);
    }

    if(is_use_encoder)
    {
        encoder_subscriber_ = node_handle_.subscribe<std_msgs::Int64MultiArray>(encoder_topic,
                                                                                encoder_queue_size,
                                                                                &LidarMappingNode::HandleEncoderMessage,
                                                                                this);
    }

    if(is_publish_pcd_map_)
    {
        pcd_map_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(pcd_map_topic,
                                                                              pcd_map_topic_queue_size);
    }
    
    if(is_publish_generate_map_service_)
    {
        generate_map_service_ = node_handle_.advertiseService(generate_map_service_name,
                                                              &LidarMappingNode::GenerateMapCallback,
                                                              this);
    }
    
    if(is_publish_start_builder_service_)
    {
        // 如果设置等待开始建图的 service，则等待开始建图的service后，开始建图，否则，开机自动开始接收数据建图
        is_receive_sensor_data_ = false;
        start_builder_service_ = node_handle_.advertiseService(start_builder_service_name,
                                                               &LidarMappingNode::StartBuilderCallback,
                                                               this);
    }

    if(is_publish_save_map_service_)
    {
        save_map_service_ = node_handle_.advertiseService(save_map_service_name,
                                                          &LidarMappingNode::SaveMapCallback,
                                                          this);
    }
    
   

    if(is_publish_pose_registration_)
    {
        pose_registration_publisher_ = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_registration_topic, 
                                                                                                        pose_registration_queue_size,
                                                                                                        true);
    }

    if(is_publish_occupancy_grid_map_)
    {
        occupancy_grid_map_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(occupancy_grid_map_topic, 
                                                                                        occupancy_grid_map_queue_size,
                                                                                        true);
    }

    if(is_publish_generate_map_status_)
    {
        generate_map_status_publisher_ = node_handle_.advertise<std_msgs::Float64>(generate_map_status_topic, 
                                                                                   generate_map_status_queue_size,
                                                                                   true);
    }
    

    wall_timers_.push_back(node_handle_.createWallTimer(ros::WallDuration(kPublishPcdMapSec),
                                                        &LidarMappingNode::PublishPcdMap,
                                                        this));

    wall_timers_.push_back(node_handle_.createWallTimer(ros::WallDuration(kPublishTfMsgSec),
                                                        &LidarMappingNode::PublishTfMsg,
                                                        this));

    wall_timers_.push_back(node_handle_.createWallTimer(ros::WallDuration(kRunGenerateMapSec),
                                                        &LidarMappingNode::RunGenerateMap,
                                                        this));
}


LidarMappingNode::~LidarMappingNode()
{
}

bool LidarMappingNode::GenerateMapCallback(lidar_mapping::generate_map_srv::Request  &req,
         			                       lidar_mapping::generate_map_srv::Response &res)
{
    if(req.is_start_generate)
    {
        is_receive_sensor_data_ = false;
        is_run_generate_map_ = true;

        res.is_received = true;

        return true;
    }
}


bool LidarMappingNode::StartBuilderCallback(lidar_mapping::start_builder_srv::Request  &req,
         			                        lidar_mapping::start_builder_srv::Response &res)
{
    if(req.is_start)
    {
        is_receive_sensor_data_ = true;
    }
    
    res.is_received = true;
    return true;
}


bool LidarMappingNode::SaveMapCallback(lidar_mapping::save_map_srv::Request  &req,
                                       lidar_mapping::save_map_srv::Response &res)
{
    res.is_map_saved = false;
    res.is_received = true;

    if(req.is_save_map)
    {
        if(map_builder_.SaveMap(req.map_name))
        {
            res.is_map_saved = true;
        }
    }


    if(is_delete_project_directory_after_save_map_)
    {
        map_builder_.DeleteProjectDirectory();
    }

    is_exit_node_ = true;
    return true;
}

void LidarMappingNode::HandleImuMessage(const sensor_msgs::Imu::ConstPtr& msg)
{
    std::unique_ptr<sensor::ImuData> imu_data = ToImuData(msg);

    if(imu_data != nullptr)
    {
        map_builder_.AddSensorData(
            sensor::ImuData
                {
                    imu_data->time,
                    imu_data->linear_acceleration,
                    imu_data->angular_velocity
                });
    }
}

void LidarMappingNode::HandleOdomMessage(const nav_msgs::Odometry::ConstPtr& msg)
{
    // lidar_localization_bridge_.sensor_bridge()->HandleOdomMessage(msg);
}

void LidarMappingNode::HandleEncoderMessage(const std_msgs::Int64MultiArray::ConstPtr& msg)
{
    int64_t encoder_left = msg->data[0];
    int64_t encoder_right = msg->data[1];
    int64_t time_stamp = msg->data[2];

    // 里程计在往消息中放时间戳的时候将 double类型的时间戳乘以 1000000.0再发布
    ros::Time ros_time_stamp;
    ros_time_stamp.sec = (uint32_t)(time_stamp/1000000.0);
    ros_time_stamp.nsec = (time_stamp/1000000.0 - ros_time_stamp.sec) * 1e9;

    // std::cout << std::setprecision(15)
    //           << "HandleEncoderMessage::encoder_left:" << encoder_left
    //           << "  encoder_right:" << encoder_right
    //           << "  time_stamp:" << time_stamp
    //           << "  ros_time_stamp:" << ros_time_stamp.toSec()
    //           << "  kd_left_:" << all_options_ptr_->ros_interface_.odometry_model_.intrinsic_.kd_left_
    //           << "  kd_right_:" << all_options_ptr_->ros_interface_.odometry_model_.intrinsic_.kd_left_
    //           << "  wheel_track_:" << all_options_ptr_->ros_interface_.odometry_model_.intrinsic_.wheel_track_
    //           << std::endl;

    // # 编码器里程计 2020_02_24更换安装件以后标定参数
    constexpr double kKdRight = 0.00015787074574;
    constexpr double kKdLeft = 0.000153387434168;
    constexpr double kWheelTrack = 0.714661691185;

    if(!is_get_init_encoder_)
    {
        is_get_init_encoder_ = true;
        encoder_current_.encoder_left = encoder_left;
        encoder_current_.encoder_right = encoder_right;
        encoder_current_.time_stamp = ros_time_stamp;

        encoder_previous_ = encoder_current_;
        encoder_init_ = encoder_current_;
    }

    encoder_current_.encoder_left = encoder_left;
    encoder_current_.encoder_right = encoder_right;
    encoder_current_.time_stamp = ros_time_stamp;

    {
        // absl::MutexLock lock(&mutex_);
        odom_data_.odom_yaw = ((encoder_current_.encoder_right - encoder_init_.encoder_right)*kKdRight
                            - (encoder_current_.encoder_left - encoder_init_.encoder_left)*kKdLeft)
                                / kWheelTrack;

        double distance_delta =  ((encoder_current_.encoder_left - encoder_previous_.encoder_left)*kKdLeft
                                + (encoder_current_.encoder_right - encoder_previous_.encoder_right)*kKdRight) / 2.0;




        encoder_previous_ = encoder_current_;

        odom_data_.odom_x += distance_delta*cos(odom_data_.odom_yaw);
        odom_data_.odom_y += distance_delta*sin(odom_data_.odom_yaw);

        odom_msg_.header.stamp = ros_time_stamp;

        odom_msg_.header.frame_id = "map";

        // 使用IMU航向角计算里程计位姿
        odom_msg_.pose.pose.position.x = odom_data_.odom_x;
        odom_msg_.pose.pose.position.y = odom_data_.odom_y;
        geometry_msgs::Quaternion q_odom = tf::createQuaternionMsgFromYaw(odom_data_.odom_yaw);


        odom_msg_.pose.pose.position.z = 0.0;
        odom_msg_.pose.pose.orientation = q_odom;

        odom_msg_.pose.covariance =  {0.001,    0.0,     0.0,    0.0,    0.0,    0.0,
                                      0.0,      0.003,   0.0,    0.0,    0.0,    0.0,
                                      0.0,      0.0,     0.005,  0.0,    0.0,    0.0,
                                      0.0,      0.0,     0.0,    0.001,  0.0,    0.0,
                                      0.0,      0.0,     0.0,    0.0,    0.001,  0.0,
                                      0.0,      0.0,     0.0,    0.0,    0.0,    0.001};

        // ROS_INFO("covariance[0]:%f  covariance[7]:%f  covariance[14]:%f  ", odom.pose.covariance[0], odom.pose.covariance[7], odom.pose.covariance[14]);

        odom_msg_.child_frame_id = "base_link";
        // odom.twist.twist.linear.x = distance_delta / (current_time - last_time).toSec();   //单位：m/s
        odom_msg_.twist.twist.linear.x = 0.0;   //单位：m/s
        odom_msg_.twist.twist.linear.y = 0.0;
        odom_msg_.twist.twist.linear.z = 0.0;
        // odom.twist.twist.angular.z = (encoder_odom_yaw_rad - odom_yaw_back_rad) / (current_time - last_time).toSec();  //单位：rad/s
        odom_msg_.twist.twist.angular.z = 0.0;  //单位：rad/s

        odom_msg_.twist.covariance = {0.001,    0.0,     0.0,    0.0,    0.0,    0.0,
                                      0.0,      0.003,   0.0,    0.0,    0.0,    0.0,
                                      0.0,      0.0,     0.005,  0.0,    0.0,    0.0,
                                      0.0,      0.0,     0.0,    0.001,  0.0,    0.0,
                                      0.0,      0.0,     0.0,    0.0,    0.001,  0.0,
                                      0.0,      0.0,     0.0,    0.0,    0.0,    0.001};
    }



    // std::cout << std::setprecision(15)
    //           << "  distance_delta:" << distance_delta
    //           << "  odom_yaw:" << odom_data_.odom_yaw
    //           << "  odom_data_.odom_x:" << odom_data_.odom_x
    //           << "  odom_data_.odom_y:" << odom_data_.odom_y
    //           << std::endl;


    std::unique_ptr<sensor::OdometryData> odometry_data = ToOdometryData(odom_msg_);

    if(odometry_data != nullptr)
    {
        map_builder_.AddSensorData(sensor::OdometryData{odometry_data->time,
                                                        odometry_data->pose});
    }

    // return *odometry_data;
}



void LidarMappingNode::HandlePointCloud2Message(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if(is_receive_sensor_data_)
    {
        map_builder_.AddVlpPointCloudData(msg);
    }
}

void LidarMappingNode::HandleGnssFixMessage(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    if(is_receive_sensor_data_)
    {
        map_builder_.AddGnssFixData(msg);
    }
}

void LidarMappingNode::HandleSickMessage(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if(is_receive_sensor_data_)
    {
        map_builder_.AddSickData(msg);
    }
}

void LidarMappingNode::PublishPcdMap(const ros::WallTimerEvent& unused_timer_event)
{
    if(is_publish_pcd_map_)
    {
        if(pcd_map_publisher_.getNumSubscribers() > 0)
        {
            map_builder_.GetPcdMapMsg(pcd_map_msg_);
            pcd_map_publisher_.publish(pcd_map_msg_);
        }        
    }


    if(is_publish_occupancy_grid_map_)
    {
        nav_msgs::OccupancyGrid occupancy_grid_map;
        if(map_builder_.GetOccupancyGridMsg(occupancy_grid_map, occupancy_grid_map_update_distance_))
        {
            occupancy_grid_map_publisher_.publish(occupancy_grid_map);
        }
        
    }
}


void LidarMappingNode::PublishTfMsg(const ros::WallTimerEvent& unused_timer_event)
{
    tf_broadcaster_.sendTransform(map_builder_.GetTfMsg());

    if(is_publish_pose_registration_)
    {
        pose_registration_publisher_.publish(map_builder_.GetPoseRegistration());
    }
}


void LidarMappingNode::RunGenerateMap(const ros::WallTimerEvent& unused_timer_event)
{
    if(is_exit_node_)
    {
        ros::shutdown();
    }

    if(is_run_generate_map_)
    {
        is_run_generate_map_ = false;

        std_msgs::Float64 generate_map_status;

        generate_map_status.data = 0.0;
        generate_map_status_publisher_.publish(generate_map_status);

        if(is_run_gnss_aidded_optimization_)
        {
            std::cout << "----------------------------------------run_gnss_aidded_optimization---------------------------------" << std::endl;
            map_builder_.GnssAidedOptimization();
        }

        if(is_run_anchor_points_optimization_)
        {
            std::cout << "----------------------------------------run_anchor_points_optimization---------------------------------" << std::endl;
            map_builder_.AnchorPointsOptimization();
        }

        generate_map_status.data = 0.25;
        generate_map_status_publisher_.publish(generate_map_status);

        if(is_run_closeloop_optimization_)
        {
            std::cout << "----------------------------------------run_closeloop_optimization---------------------------------" << std::endl;
            map_builder_.CloseloopOptimization();
        }

        generate_map_status.data = 0.5;
        generate_map_status_publisher_.publish(generate_map_status);

        if(is_generate_raw_map_)
        {
            std::cout << "----------------------------------------generate_raw_map---------------------------------" << std::endl;
            map_builder_.GenerateRawMap();
        }

        if(is_generate_compensation_map_)
        {
            std::cout << "----------------------------------------generate_compensation_map---------------------------------" << std::endl;
            map_builder_.GenerateCompensationMap();
        }

        if(is_generate_gnss_optimization_map_)
        {
            std::cout << "----------------------------------------run_closeloop_optimization---------------------------------" << std::endl;
            map_builder_.GenerateGnssOptimizationMap();
        }

        if(is_generate_closeloop_optimization_map_)
        {
            std::cout << "----------------------------------------generate_closeloop_optimization_map---------------------------------" << std::endl;
            map_builder_.GenerateCloseloopOptimizationMap();
        }

        if(is_generate_laserscan_map_)
        {
            std::cout << "----------------------------------------generate_laserscan_map---------------------------------" << std::endl;
            map_builder_.GenerateLaserScanMap();
        }

        if(is_run_map_partition_)
        {
            std::cout << "----------------------------------------run_map_partition---------------------------------" << std::endl;
            map_builder_.RunMapPartition();
        }

        generate_map_status.data = 0.75;
        generate_map_status_publisher_.publish(generate_map_status);

        if(is_generate_arealist_file_)
        {
            std::cout << "----------------------------------------run_generate_arealist_filen---------------------------------" << std::endl;
            map_builder_.GenerateArealistFile();
        }

        generate_map_status.data = 1.0;
        generate_map_status_publisher_.publish(generate_map_status);

        // 发布最终生成的地图
        if(is_publish_occupancy_grid_map_)
        {
            nav_msgs::OccupancyGrid occupancy_grid_map;

            if(map_builder_.GetOccupancyGridMsg(occupancy_grid_map))
            {
                occupancy_grid_map_publisher_.publish(occupancy_grid_map);
            }
        }
    }
}


std::unique_ptr<sensor::ImuData> LidarMappingNode::ToImuData(const sensor_msgs::Imu::ConstPtr& msg)
{

    CHECK_NE(msg->linear_acceleration_covariance[0], -1)
        << "Your IMU data claims to not contain linear acceleration measurements "
            "by setting linear_acceleration_covariance[0] to -1. Cartographer "
            "requires this data to work. See "
            "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";


    CHECK_NE(msg->angular_velocity_covariance[0], -1)
        << "Your IMU data claims to not contain angular velocity measurements "
            "by setting angular_velocity_covariance[0] to -1. Cartographer "
            "requires this data to work. See "
            "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

    const common::Time time = common::FromRos(msg->header.stamp);


    //   const auto sensor_to_tracking = tf_bridge_.LookupToTracking(time,
    //                                                               CheckNoLeadingSlash(msg->header.frame_id));


    // Eigen::Vector3d translation_tracking_to_imu(all_options_ptr_->ros_interface_.sensor_frames_.tracking_to_imu_.x,
    //                                             all_options_ptr_->ros_interface_.sensor_frames_.tracking_to_imu_.y,
    //                                             all_options_ptr_->ros_interface_.sensor_frames_.tracking_to_imu_.z
    //                                             );

    // Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(common::degree_to_radian(all_options_ptr_->ros_interface_.sensor_frames_.tracking_to_imu_.roll), Eigen::Vector3d::UnitX()));
    // Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(common::degree_to_radian(all_options_ptr_->ros_interface_.sensor_frames_.tracking_to_imu_.pitch), Eigen::Vector3d::UnitY()));
    // Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(common::degree_to_radian(all_options_ptr_->ros_interface_.sensor_frames_.tracking_to_imu_.yaw), Eigen::Vector3d::UnitZ()));
    // Eigen::Quaterniond quaternion_tracking_to_imu = yaw_angle * pitch_angle * roll_angle;


    // std::unique_ptr<transform::Rigid3d> sensor_to_tracking = absl::make_unique<transform::Rigid3d>(translation_tracking_to_imu,
    //                                                                                                quaternion_tracking_to_imu);

    // if(sensor_to_tracking == nullptr)
    // {
    //     return nullptr;
    // }

    // std::cout << "imu_to_tracking:" << sensor_to_tracking->translation()[0] << " " << sensor_to_tracking->translation()[0] << " " << sensor_to_tracking->translation()[0] << " "
    //           << sensor_to_tracking->rotation().x() << " " << sensor_to_tracking->rotation().y() << " " << sensor_to_tracking->rotation().z() << " " << sensor_to_tracking->rotation().w() << " "
    //           << std::endl;


    // CHECK(sensor_to_tracking->translation().norm() < 1e-5)
    //     << "The IMU frame must be colocated with the tracking frame. "
    //        "Transforming linear acceleration into the tracking frame will "
    //        "otherwise be imprecise.";

    // return absl::make_unique<sensor::ImuData>(
    //     sensor::ImuData  //构造 ImuData类型的数据
    //         {
    //             time,
    //             sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
    //             sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)
    //         });
    

    return absl::make_unique<sensor::ImuData>(
        sensor::ImuData  //构造 ImuData类型的数据
            {
                time,
                ToEigen(msg->linear_acceleration),
                ToEigen(msg->angular_velocity)
            });
}

// ROS 消息转换成 ODOM类型数据，并且转到 tracking 坐标系下
std::unique_ptr<sensor::OdometryData> LidarMappingNode::ToOdometryData(const nav_msgs::Odometry& msg)
{
  const common::Time time = common::FromRos(msg.header.stamp);

  const auto sensor_to_tracking = absl::make_unique<transform::Rigid3d>();

  if(sensor_to_tracking == nullptr)
  {
    return nullptr;
  }

  return absl::make_unique<sensor::OdometryData>(
      sensor::OdometryData
        {
            time,
            ToRigid3d(msg.pose.pose) * sensor_to_tracking->inverse()
        });
}

transform::Rigid3d LidarMappingNode::ToRigid3d(const geometry_msgs::Pose& pose)
{
  return transform::Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                            ToEigen(pose.orientation));
}


Eigen::Vector3d LidarMappingNode::ToEigen(const geometry_msgs::Vector3& vector3)
{
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond LidarMappingNode::ToEigen(const geometry_msgs::Quaternion& quaternion)
{
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_mapping_node");
    ros::NodeHandle node_handle;

    std::string cfg_file_path;
    ros::param::get("/lidar_mapping_node/cfg_file_path", cfg_file_path);
    std::cout << "cfg_file_path:" << cfg_file_path << std::endl;

    auto time_now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
 
	//转为字符串
	std::stringstream ss;
	ss << std::put_time(std::localtime(&time_now), "%Y-%m-%d-%H-%M-%S");

	std::string project_directory_name = ss.str();
    // std::string project_directory_name = "2023-09-02-22-59-11";

    std::cout << "poject_directory_name:" << project_directory_name << std::endl;

    LidarMappingNode lidar_mapping_node(cfg_file_path,
                                        project_directory_name + "/",
                                        node_handle);
    ros::spin();

    ros::shutdown();
}
