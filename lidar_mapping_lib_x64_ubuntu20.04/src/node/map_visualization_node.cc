///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_19
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "src/node/map_visualization_node.h"
#include "pcl/filters/voxel_grid.h"

#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>

void GetAllFiles(std::string map_file_path,
                 std::vector<std::string>& all_files)
{
    const char *dir_name = map_file_path.c_str();

	if(NULL == dir_name)
	{
		std::cout << " dir_name is null ! " << std::endl;
		return ;
	}
 
	struct stat s;
	lstat(dir_name , &s);

	if(!S_ISDIR(s.st_mode))
	{
		std::cout << "dir_name is not a valid directory !" 
                  << std::endl;
		return;
	}
	
	struct dirent * filename;    
 	DIR * dir;                 

	dir = opendir(dir_name);

	if(NULL == dir)
	{
		std::cout << "Can not open dir " << dir_name << std::endl;
		return;
	}

	std::cout << "Successfully opened the dir !" << std::endl;
	
	while((filename = readdir(dir)) != NULL)
	{
		// get rid of "." and ".."
		if(strcmp(filename->d_name , "." ) == 0 || 
           strcmp(filename->d_name , "..") == 0 )
			continue;

        all_files.push_back(filename ->d_name);
		std::cout << filename ->d_name << std::endl;
	}
} 



MapVisualizationNode::MapVisualizationNode(std::string map_file_path,
                                           double voxel_leaf_size,
                                           ros::NodeHandle node_handle)
    : node_handle_(node_handle),
      map_file_path_(map_file_path),
      voxel_leaf_size_(voxel_leaf_size),
      pcd_map_updated_(true)
{
    GetAllFiles(map_file_path, 
                map_file_lists_);

    pcd_map_publisher_ 
        = node_handle_.advertise<sensor_msgs::PointCloud2>("map_visualization_node/map", 1, true);

    wall_timers_.push_back(node_handle_.createWallTimer(ros::WallDuration(kPublishPcdMapSec),
                                                        &MapVisualizationNode::PublishPcdMap, this));
}


MapVisualizationNode::~MapVisualizationNode()
{
}


void MapVisualizationNode::PublishPcdMap(const ros::WallTimerEvent& unused_timer_event)
{
    if(pcd_map_updated_)
    {
        pcd_map_updated_ = false;

        for(std::vector<std::string>::iterator item = map_file_lists_.begin();
                                               item != map_file_lists_.end();
                                               item++)
        {
            pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>);
            if(pcl::io::loadPCDFile<PointType>(map_file_path_ + *item, *cloud_ptr) == -1)
            {
                PCL_ERROR("Couldn't load pcd file \n");
            }

            // pcl::PointCloud<PointType> pcd_map_filtered;
            // pcl::VoxelGrid<PointType> voxel_grid_filter;

            // voxel_grid_filter.setLeafSize(voxel_leaf_size_, 
            //                               voxel_leaf_size_, 
            //                               voxel_leaf_size_);

            // voxel_grid_filter.setInputCloud(cloud_ptr);
            // voxel_grid_filter.filter(pcd_map_filtered);

            // pcd_map_ += pcd_map_filtered;
            pcd_map_ += *cloud_ptr;
        }
        
        pcl::toROSMsg(pcd_map_, pcd_map_msg_);
        pcd_map_msg_.header.stamp = ros::Time::now();
        pcd_map_msg_.header.frame_id = "map";

        pcd_map_publisher_.publish(pcd_map_msg_);
    }


    
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_visualization_node");

    ros::NodeHandle node_handle;

    std::string map_file_path;
    double voxel_leaf_size;
    ros::param::get("/map_visualization_node/map_file_path", map_file_path);
    ros::param::get("/map_visualization_node/voxel_leaf_size", voxel_leaf_size);

    
    std::cout << "map_file_path:" << map_file_path << std::endl;
    std::cout << "voxel_leaf_size:" << voxel_leaf_size << std::endl;


    MapVisualizationNode map_visualization_node(map_file_path, 
                                                voxel_leaf_size,
                                                node_handle);


    ros::spin();
    
    return 0;
}