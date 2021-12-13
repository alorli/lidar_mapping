///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_11_26
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "src/processing/to_grid_map.h"
#include "pcl/filters/voxel_grid.h"

#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>

namespace processing
{

ToGridMap::ToGridMap(std::string cfg_file_path)
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");

    occupancy_grid_map_parameter_.resolution = cfg_file_["ros_interface"]["advertise_message"]["occupancy_grid_map"]["resolution"].as<double>();
    occupancy_grid_map_parameter_.empty_grid_value = cfg_file_["ros_interface"]["advertise_message"]["occupancy_grid_map"]["empty_grid_value"].as<int>();
    occupancy_grid_map_parameter_.occupancy_grid_value = cfg_file_["ros_interface"]["advertise_message"]["occupancy_grid_map"]["occupancy_grid_value"].as<int>();
}


ToGridMap::~ToGridMap()
{
}

void ToGridMap::PointCloudToOccupancyGridMsg(pcl::PointCloud<pcl::PointXYZI>& pcd_map, 
                                             nav_msgs::OccupancyGrid& occupancy_grid_map_msg)
{
    double x_max = std::numeric_limits<double>::min();
    double x_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();
    double y_min = std::numeric_limits<double>::max();
    for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = pcd_map.begin();
        item != pcd_map.end();
        item++)
    {
        pcl::PointXYZI point;
        point.x = (double)item->x;
        point.y = (double)item->y;
        point.z = (double)item->z;

        if(point.x > x_max)
        {
            x_max = point.x;
        }
        if(point.x < x_min)
        {
            x_min = point.x;
        }
        if(point.y > y_max)
        {
            y_max = point.y;
        }
        if(point.y < y_min)
        {
            y_min = point.y;
        }
    }

    double delta_x = x_max - x_min;
    double delta_y = y_max - y_min;

    double width_meter = delta_x;
    double height_meter = delta_y;

    const int width = (int)(width_meter/occupancy_grid_map_parameter_.resolution) + 1;
    const int height =(int)(height_meter/occupancy_grid_map_parameter_.resolution) + 1;

    occupancy_grid_map_msg.header.stamp = ros::Time::now();
    occupancy_grid_map_msg.header.frame_id = "map";
    occupancy_grid_map_msg.info.map_load_time = ros::Time::now();
    occupancy_grid_map_msg.info.resolution = occupancy_grid_map_parameter_.resolution;
    occupancy_grid_map_msg.info.width = width;
    occupancy_grid_map_msg.info.height = height;
    occupancy_grid_map_msg.info.origin.position.x = x_min;
    occupancy_grid_map_msg.info.origin.position.y = y_min;
    occupancy_grid_map_msg.info.origin.position.z = 0.0;
    occupancy_grid_map_msg.info.origin.orientation.w = 1.;
    occupancy_grid_map_msg.info.origin.orientation.x = 0.;
    occupancy_grid_map_msg.info.origin.orientation.y = 0.;
    occupancy_grid_map_msg.info.origin.orientation.z = 0.;
    occupancy_grid_map_msg.data.reserve(width * height);
    
    for(int i = 0; i < height; i++) 
    {
        for(int j = 0; j < width; j++) 
        {      
            occupancy_grid_map_msg.data.push_back(occupancy_grid_map_parameter_.empty_grid_value);
        }
    }



    for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = pcd_map.begin();
        item != pcd_map.end();
        item++)
    {
        pcl::PointXYZI point;
        point.x = (double)item->x;
        point.y = (double)item->y;
        point.z = (double)item->z;

        int x_index = (int)((point.x - x_min)/occupancy_grid_map_parameter_.resolution);
        int y_index = (int)((point.y - y_min)/occupancy_grid_map_parameter_.resolution);
        int index = y_index*width+x_index;

        if((x_index < 0) || (x_index >= width) 
        || (y_index < 0) || (y_index >= height) 
        || (index < 0) || (index >= width * height))
        {
            continue;
        }

        occupancy_grid_map_msg.data[index] = occupancy_grid_map_parameter_.occupancy_grid_value;
    }
}

} // namespace processing
