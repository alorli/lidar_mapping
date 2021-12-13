///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_19
///////////////////////////////////////////////////////////////////////////////

#include "src/node/lidar_mapping_node.h"
#include <iostream>


LidarMappingNode::LidarMappingNode(std::string cfg_file_path, 
                                   ros::NodeHandle node_handle)
    : map_builder_(cfg_file_path),
      node_handle_(node_handle),
      is_generate_map_(false)
{    
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");

    is_generate_raw_map_ = cfg_file_["lidar_mapping_node"]["is_generate_raw_map"].as<bool>();
    is_generate_compensation_map_ = cfg_file_["lidar_mapping_node"]["is_generate_compensation_map"].as<bool>();
    is_generate_gnss_optimization_map_ = cfg_file_["lidar_mapping_node"]["is_generate_gnss_optimization_map"].as<bool>();
    is_generate_closeloop_optimization_map_ = cfg_file_["lidar_mapping_node"]["is_generate_closeloop_optimization_map"].as<bool>();
    is_generate_laserscan_map_ = cfg_file_["lidar_mapping_node"]["is_generate_laserscan_map"].as<bool>();
    is_run_gnss_aidded_optimization_ = cfg_file_["lidar_mapping_node"]["is_run_gnss_aidded_optimization"].as<bool>();
    is_run_closeloop_optimization_ = cfg_file_["lidar_mapping_node"]["is_run_closeloop_optimization"].as<bool>();


    pointcloud_subscriber_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/points_raw", 
                                                                               1000000,   //10hz 存 27小时数据
                                                                               &LidarMappingNode::HandlePointCloud2Message,
                                                                               this);  

    gnss_fix_subscriber_ = node_handle_.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 
                                                                          1000000,   
                                                                          &LidarMappingNode::HandleGnssFixMessage,
                                                                          this);

    sick_subscriber_ = node_handle_.subscribe<sensor_msgs::LaserScan>("/scan", 
                                                                      1000000,   
                                                                      &LidarMappingNode::HandleSickMessage,
                                                                      this);

    pcd_map_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("lidar_mapping_node/map", 
                                                                          1);

    generate_map_service_ = node_handle_.advertiseService("/generate_map", 
                                                          &LidarMappingNode::GenerateMapCallback, 
                                                          this);

    wall_timers_.push_back(node_handle_.createWallTimer(ros::WallDuration(kPublishPcdMapSec),
                                                        &LidarMappingNode::PublishPcdMap, 
                                                        this));

    wall_timers_.push_back(node_handle_.createWallTimer(ros::WallDuration(kPublishTfMsgSec),
                                                        &LidarMappingNode::PublishTfMsg, 
                                                        this));
}


LidarMappingNode::~LidarMappingNode()
{
}

bool LidarMappingNode::GenerateMapCallback(lidar_mapping::generate_map_srv::Request  &req,
         			                       lidar_mapping::generate_map_srv::Response &res)
{
    ROS_INFO("voxel_size:%f", req.voxel_size);

    if(is_run_gnss_aidded_optimization_)
    {
        map_builder_.GnssAidedOptimization();
    }    

    if(is_run_closeloop_optimization_)
    {
        map_builder_.CloseloopOptimization();
    }  


    if(is_generate_raw_map_)
    {
        map_builder_.GenerateRawMap();
    }

    if(is_generate_compensation_map_)
    {
        map_builder_.GenerateCompensationMap();
    }
    
    if(is_generate_gnss_optimization_map_)
    {
        map_builder_.GenerateGnssOptimizationMap();
    }

    if(is_generate_closeloop_optimization_map_)
    {
        map_builder_.GenerateCloseloopOptimizationMap();
    }

    if(is_generate_laserscan_map_)
    {
        map_builder_.GenerateLaserScanMap();
    }

 
    is_generate_map_ = true;

	res.generated = true;
    return true;   
}


void LidarMappingNode::HandlePointCloud2Message(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if(!is_generate_map_)
    {
        map_builder_.AddVlpPointCloudData(msg);
    }
    else
    {
        ros::shutdown();
    } 
}

void LidarMappingNode::HandleGnssFixMessage(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    if(!is_generate_map_)
    {
        map_builder_.AddGnssFixData(msg);
    } 
}

void LidarMappingNode::HandleSickMessage(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if(!is_generate_map_)
    {
        map_builder_.AddSickData(msg);
    } 
}

void LidarMappingNode::PublishPcdMap(const ros::WallTimerEvent& unused_timer_event)
{
  if(pcd_map_publisher_.getNumSubscribers() > 0)
  {
     map_builder_.GetPcdMapMsg(pcd_map_msg_);
     pcd_map_publisher_.publish(pcd_map_msg_);
  }
}


void LidarMappingNode::PublishTfMsg(const ros::WallTimerEvent& unused_timer_event)
{
    tf_broadcaster_.sendTransform(map_builder_.GetTfMsg());
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_mapping_node");

    ros::NodeHandle node_handle;

    std::string cfg_file_path;
    ros::param::get("/lidar_mapping_node/cfg_file_path", cfg_file_path);

    std::cout << "cfg_file_path:" << cfg_file_path << std::endl;

    LidarMappingNode lidar_mapping_node(cfg_file_path, 
                                        node_handle);
    

    ros::spin();

    return 0;
}