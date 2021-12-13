///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_19
///////////////////////////////////////////////////////////////////////////////

#include "src/node/map_building_visualization_node.h"
#include <iostream>


MapBuildingVisualizationNode::MapBuildingVisualizationNode(std::string cfg_file_path,
                                                           ros::NodeHandle node_handle)
    : node_handle_(node_handle),
      map_cloud_ptr_(new pcl::PointCloud<registration::PointType>())
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg_map_building_visualization.yaml");

    std::string registration_results_filelist_directory = cfg_file_["directory"]["registration_results_filelist_directory"].as<std::string>();
    std::string map_directory = cfg_file_["directory"]["map_directory"].as<std::string>();
    mapping_parameter_.min_map_horizontal_radius = cfg_file_["parameter"]["min_map_horizontal_radius"].as<double>();
    mapping_parameter_.min_keyframe_distance = cfg_file_["parameter"]["min_keyframe_distance"].as<double>();
    mapping_parameter_.num_keyframe_submap = cfg_file_["parameter"]["num_keyframe_submap"].as<double>();
    mapping_parameter_.min_intensity = cfg_file_["parameter"]["min_intensity"].as<double>();
    mapping_parameter_.max_intensity = cfg_file_["parameter"]["max_intensity"].as<double>();

    int map_cloud_vector_size = cfg_file_["map_cloud_vector_size"].as<int>();

    double map_cloud_publish_frequency = cfg_file_["map_cloud_publish_frequency"].as<double>();


    map_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("map_building_visualization_node/map",
                                                                          1);

    for(int i=0; i<map_cloud_vector_size; i++)
    {
       map_cloud_topic_vector_.push_back("/map_building_visualization_node/map_"+std::to_string(i));

       ros::Publisher map_cloud_publisher = node_handle_.advertise<sensor_msgs::PointCloud2>(map_cloud_topic_vector_.at(i),
                                                                                            1);

       map_cloud_publisher_vector_.push_back(map_cloud_publisher);
    }


    wall_timers_.push_back(node_handle_.createWallTimer(ros::WallDuration( 1.0/map_cloud_publish_frequency ),
                                                        &MapBuildingVisualizationNode::PublishPcdMap,
                                                        this));


    map_generator_closeloop_optimization_.SetPathName(registration_results_filelist_directory,
                                                      map_directory);

    map_generator_closeloop_optimization_.SetParameter(mapping_parameter_.min_keyframe_distance,
                                                       mapping_parameter_.min_map_horizontal_radius,
                                                       mapping_parameter_.num_keyframe_submap,
                                                       mapping_parameter_.min_intensity,
                                                       mapping_parameter_.max_intensity
                                                      );

    map_generator_closeloop_optimization_.SetMapVectorSize(map_cloud_vector_size);
    map_generator_closeloop_optimization_.LoadRegistrationFileLists();


}


MapBuildingVisualizationNode::~MapBuildingVisualizationNode()
{
}


std::vector<geometry_msgs::TransformStamped> MapBuildingVisualizationNode::GetTfMsg()
{
    std::vector<geometry_msgs::TransformStamped> stamped_transforms;
    geometry_msgs::TransformStamped stamped_transform;
    geometry_msgs::Transform transform;
    registration::AlignmentResult alignment_result_current
        = map_generator_closeloop_optimization_.GetCurrentAlignmentResult();

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


void MapBuildingVisualizationNode::PublishPcdMap(const ros::WallTimerEvent& unused_timer_event)
{
  if(map_cloud_publisher_.getNumSubscribers() > 0)
  {
      int update_index = map_generator_closeloop_optimization_.UpdateMapCloudVector();

      const std::vector<pcl::PointCloud<registration::PointType>::Ptr>& map_cloud_vector_ptr
          = map_generator_closeloop_optimization_.GetMapCloudVector();

      map_cloud_ptr_->clear();

      pcl::toROSMsg(*map_cloud_vector_ptr.at(update_index), map_cloud_message_);
      map_cloud_message_.header.stamp = ros::Time::now();
      map_cloud_message_.header.frame_id = "map";
      map_cloud_publisher_vector_.at(update_index).publish(map_cloud_message_);

      tf_broadcaster_.sendTransform(GetTfMsg());
  }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_building_visualization_node");

    ros::NodeHandle node_handle;

    std::string cfg_file_path;
    ros::param::get("/map_building_visualization_node/cfg_file_path", cfg_file_path);

    std::cout << "cfg_file_path:" << cfg_file_path << std::endl;

    MapBuildingVisualizationNode MapBuildingVisualizationNode(cfg_file_path,
                                                              node_handle);


    ros::spin();

    return 0;
}
