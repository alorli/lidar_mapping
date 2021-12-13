///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_19
///////////////////////////////////////////////////////////////////////////////

#ifndef MAP_BUILDING_VISUALIZATION_NODE_H_
#define MAP_BUILDING_VISUALIZATION_NODE_H_


#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "yaml-cpp/yaml.h"

#include "src/map_generator.h"

#include "tf2_ros/transform_broadcaster.h"

#include <vector>

struct MappingParameter
{
    double min_map_horizontal_radius;
    double min_keyframe_distance;
    int num_keyframe_submap;
    double min_intensity;
    double max_intensity;
};

class MapBuildingVisualizationNode
{

public:

    MapBuildingVisualizationNode(std::string cfg_file_path,
                                 ros::NodeHandle node_handle);
    ~MapBuildingVisualizationNode();


    void PublishPcdMap(const ros::WallTimerEvent& unused_timer_event);
    void PublishTfMsg(const ros::WallTimerEvent& unused_timer_event);

    std::vector<geometry_msgs::TransformStamped> GetTfMsg();

private:
    YAML::Node cfg_file_;

    ros::NodeHandle node_handle_;

    ros::Publisher map_cloud_publisher_;

    std::vector<ros::Publisher> map_cloud_publisher_vector_;
    std::vector<std::string> map_cloud_topic_vector_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::vector<ros::WallTimer> wall_timers_;
    sensor_msgs::PointCloud2 pcd_map_msg_;

    MappingParameter mapping_parameter_;
    pcl::PointCloud<registration::PointType>::Ptr map_cloud_ptr_;
    sensor_msgs::PointCloud2 map_cloud_message_;

    mapping::MapGenerator map_generator_closeloop_optimization_;
    // mapping::MapGenerator map_generator_laserscan_;
};

#endif    //MAP_BUILDING_VISUALIZATION_NODE_H_
