///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_19
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "src/node/to_grid_map_node.h"
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



ToGridMapNode::ToGridMapNode(std::string map_file_path,
                             bool use_voxel_filter,
                             double voxel_leaf_size,
                             ros::NodeHandle node_handle)
    : node_handle_(node_handle),
      map_file_path_(map_file_path),
      voxel_leaf_size_(voxel_leaf_size),
      use_voxel_filter_(use_voxel_filter)
{
    pointcloud_subscriber_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/lidar_localization_node/matched_pointcloud",     // "points_raw"
                                                                               1000000,             // 10hz 存 27小时数据
                                                                               &ToGridMapNode::HandlePointCloud2Message,
                                                                               this);

    wall_timers_.push_back(node_handle_.createWallTimer(ros::WallDuration(1.0/1.0),
                                                        &ToGridMapNode::TimerPublishMessage,
                                                        this));

    GetAllFiles(map_file_path,
                map_file_lists_);

    pcd_map_publisher_
        = node_handle_.advertise<sensor_msgs::PointCloud2>("map_visualization_node/map", 1, true);

    occupancy_grid_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1, true);

    occupancy_grid_scan_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("/grid_map_laser_scan", 1, true);

    laser_scan_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>("/laser_scan", 1, true);

    PublishXYZIMap();
}


ToGridMapNode::~ToGridMapNode()
{
}



void ToGridMapNode::HandlePointCloud2Message(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<PointXYZIRT> pointcloud;
    pcl::fromROSMsg(*msg, pointcloud);

    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(pointcloud,
    //                              pointcloud,
    //                              indices);


    ///////////////////////////////////////////////////////////////////
    // double angle_min = -3.1415;           //-M_PI/2
    // double angle_max = 3.1415;            //M_PI/2
    // double angle_increment = 0.003489;      // vlp16 10hz for 0.2°

    // double scan_time = 0.1;
    // double range_min = 0.45;
    // double range_max = 80;

    // double min_height = -20.0;
    // double max_height = 20.0;

    
    // sensor_msgs::LaserScan scan_msg;

    // scan_msg.header = msg->header;
    // scan_msg.header.frame_id = "/lidar";

    // scan_msg.angle_min = angle_min;
    // scan_msg.angle_max = angle_max;
    // scan_msg.angle_increment = angle_increment;
    // scan_msg.time_increment = 0.0;
    // scan_msg.scan_time = scan_time;
    // scan_msg.range_min = range_min;
    // scan_msg.range_max = range_max;

    // uint32_t ranges_size = std::ceil(
    //   (scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);

    // scan_msg.ranges.assign(ranges_size, 1000.0);

    
    // for(pcl::PointCloud<PointXYZIRT>::const_iterator item = pointcloud.begin();
    //     item != pointcloud.end();
    //     item++)
    // {
    //     PointXYZIRT point;
    //     point.x = (double)item->x;
    //     point.y = (double)item->y;
    //     point.z = (double)item->z;
    //     point.intensity = (double)item->intensity;
    //     point.ring = (double)item->ring;
    //     point.time = (double)item->time;

    //     // 16 线雷达，就是 0 - 15，ring 的编号是从最下面一根线到最上面一根线，ID 是从 0 - 15
    //     if(point.ring == 7)
    //     {
    //         double range = hypot(point.x, point.y);
    //         double angle = atan2(point.y, point.x);

    //         if((point.z < min_height) ||  (point.z > max_height) || (range < range_min) || (range > range_max) || (angle < scan_msg.angle_min) || (angle > scan_msg.angle_max))
    //         {
    //             continue;
    //         }

    //         int index = (angle - scan_msg.angle_min) / scan_msg.angle_increment;

    //         if((index < 0) || (index >= ranges_size))
    //         {
    //             continue;
    //         }

    //         scan_msg.ranges[index] = range;
    //     }
    // }

    // laser_scan_publisher_.publish(scan_msg);
    ///////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////
    double x_max = std::numeric_limits<double>::min();
    double x_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();
    double y_min = std::numeric_limits<double>::max();
    for(pcl::PointCloud<PointXYZIRT>::const_iterator item = pointcloud.begin();
        item != pointcloud.end();
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

    std::cout << "  x_max:" << x_max
              << "  x_min:" << x_min
              << "  y_max:" << y_max
              << "  y_min:" << y_min
              << "  delta_x:" << delta_x
              << "  delta_y:" << delta_y
              << std::endl;

    double resolution = 0.1;
    double width_meter = delta_x;
    double height_meter = delta_y;

    const int width = (int)(width_meter/resolution) + 1;
    const int height =(int)(height_meter/resolution) + 1;

    nav_msgs::OccupancyGrid occupancy_grid;
    occupancy_grid.header.stamp = ros::Time::now();
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.map_load_time = ros::Time::now();
    occupancy_grid.info.resolution = resolution;
    occupancy_grid.info.width = width;
    occupancy_grid.info.height = height;
    occupancy_grid.info.origin.position.x = x_min;
    occupancy_grid.info.origin.position.y = y_min;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.;
    occupancy_grid.info.origin.orientation.x = 0.;
    occupancy_grid.info.origin.orientation.y = 0.;
    occupancy_grid.info.origin.orientation.z = 0.;
    occupancy_grid.data.reserve(width * height);
    
    for(int i = 0; i < height; i++) 
    {
        for(int j = 0; j < width; j++) 
        {      
            occupancy_grid.data.push_back(255);
        }
    }



    for(pcl::PointCloud<PointXYZIRT>::const_iterator item = pointcloud.begin();
        item != pointcloud.end();
        item++)
    {
        pcl::PointXYZI point;
        point.x = (double)item->x;
        point.y = (double)item->y;
        point.z = (double)item->z;

        int x_index = (int)((point.x - x_min)/resolution);
        int y_index = (int)((point.y - y_min)/resolution);
        int index = y_index*width+x_index;

        if((x_index < 0) || (x_index >= width) 
        || (y_index < 0) || (y_index >= height) 
        || (index < 0) || (index >= width * height))
        {
            continue;
        }

        occupancy_grid.data[index] = 390;
    }


    occupancy_grid_scan_publisher_.publish(occupancy_grid);
    ///////////////////////////////////////////////////////////////////




    // double resolution = 0.1;
    // double width_meter = 60.0;
    // double height_meter = 60.0;

    // const int width = (int)(width_meter/resolution) + 1;
    // const int height =(int)(width_meter/resolution) + 1;

    // nav_msgs::OccupancyGrid occupancy_grid;
    // occupancy_grid.header.stamp = ros::Time::now();
    // occupancy_grid.header.frame_id = "map";
    // occupancy_grid.info.map_load_time = ros::Time::now();
    // occupancy_grid.info.resolution = resolution;
    // occupancy_grid.info.width = width;
    // occupancy_grid.info.height = height;
    // occupancy_grid.info.origin.position.x = -1.0*width_meter/2.0;
    // occupancy_grid.info.origin.position.y = -1.0*height_meter/2.0;
    // occupancy_grid.info.origin.position.z = 0.0;
    // occupancy_grid.info.origin.orientation.w = 1.;
    // occupancy_grid.info.origin.orientation.x = 0.;
    // occupancy_grid.info.origin.orientation.y = 0.;
    // occupancy_grid.info.origin.orientation.z = 0.;
    // occupancy_grid.data.reserve(width * height);
    
    // for(int i = 0; i < height; i++) 
    // {
    //     for(int j = 0; j < width; j++) 
    //     {      
    //         occupancy_grid.data.push_back(255);
    //     }
    // }

    // for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = pointcloud.begin();
    //     item != pointcloud.end();
    //     item++)
    // {
    //     pcl::PointXYZI point;
    //     point.x = (double)item->x;
    //     point.y = (double)item->y;
    //     point.z = (double)item->z;

    //     int x_index = (int)((point.x + width_meter/2.0)/resolution);
    //     int y_index = (int)((point.y + height_meter/2.0)/resolution);
    //     int index = y_index*width+x_index;

    //     if((x_index < 0) || (x_index >= width) 
    //     || (y_index < 0) || (y_index >= height) 
    //     || (index < 0) || (index >= width * height))
    //     {
    //         continue;
    //     }

    //     occupancy_grid.data[index] = 0;
    // }


    // occupancy_grid_publisher_.publish(occupancy_grid);
}

void ToGridMapNode::PublishXYZIMap()
{
    for(std::vector<std::string>::iterator item = map_file_lists_.begin();
                                           item != map_file_lists_.end();
                                           item++)
    {
        pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>);
        if(pcl::io::loadPCDFile<PointType>(map_file_path_ + *item, *cloud_ptr) == -1)
        {
            PCL_ERROR("Couldn't load pcd file \n");
        }

        if(use_voxel_filter_)
        {
            pcl::PointCloud<PointType> pcd_map_filtered;
            pcl::VoxelGrid<PointType> voxel_grid_filter;

            voxel_grid_filter.setLeafSize(voxel_leaf_size_,
                                          voxel_leaf_size_,
                                          voxel_leaf_size_);

            voxel_grid_filter.setInputCloud(cloud_ptr);
            voxel_grid_filter.filter(pcd_map_filtered);
            pcd_map_ += pcd_map_filtered;
        }
        else
        {
            pcd_map_ += *cloud_ptr;
        }
    }

    //   pcl::toROSMsg(pcd_map_, pcd_map_msg_);
    //   pcd_map_msg_.header.stamp = ros::Time::now();
    //   pcd_map_msg_.header.frame_id = "map";

    //   pcd_map_publisher_.publish(pcd_map_msg_);



    /////////////////////////////////////////////////////////////////////////////
    double x_max = std::numeric_limits<double>::min();
    double x_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();
    double y_min = std::numeric_limits<double>::max();
    for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = pcd_map_.begin();
        item != pcd_map_.end();
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

    std::cout << "  x_max:" << x_max
              << "  x_min:" << x_min
              << "  y_max:" << y_max
              << "  y_min:" << y_min
              << "  delta_x:" << delta_x
              << "  delta_y:" << delta_y
              << std::endl;

    double resolution = 0.1;
    double width_meter = delta_x;
    double height_meter = delta_y;

    const int width = (int)(width_meter/resolution) + 1;
    const int height =(int)(height_meter/resolution) + 1;

    nav_msgs::OccupancyGrid occupancy_grid;
    occupancy_grid.header.stamp = ros::Time::now();
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.map_load_time = ros::Time::now();
    occupancy_grid.info.resolution = resolution;
    occupancy_grid.info.width = width;
    occupancy_grid.info.height = height;
    occupancy_grid.info.origin.position.x = x_min;
    occupancy_grid.info.origin.position.y = y_min;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.;
    occupancy_grid.info.origin.orientation.x = 0.;
    occupancy_grid.info.origin.orientation.y = 0.;
    occupancy_grid.info.origin.orientation.z = 0.;
    occupancy_grid.data.reserve(width * height);
    
    for(int i = 0; i < height; i++) 
    {
        for(int j = 0; j < width; j++) 
        {      
            occupancy_grid.data.push_back(255);
        }
    }



    for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = pcd_map_.begin();
        item != pcd_map_.end();
        item++)
    {
        pcl::PointXYZI point;
        point.x = (double)item->x;
        point.y = (double)item->y;
        point.z = (double)item->z;

        int x_index = (int)((point.x - x_min)/resolution);
        int y_index = (int)((point.y - y_min)/resolution);
        int index = y_index*width+x_index;

        if((x_index < 0) || (x_index >= width) 
        || (y_index < 0) || (y_index >= height) 
        || (index < 0) || (index >= width * height))
        {
            continue;
        }

        occupancy_grid.data[index] = 0;
    }


    occupancy_grid_publisher_.publish(occupancy_grid);

}

void ToGridMapNode::TimerPublishMessage(const ros::WallTimerEvent& unused_timer_event)
{
    PublishXYZIMap();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "to_grid_map_node");

    ros::NodeHandle node_handle;

    std::string map_file_path;
    double voxel_leaf_size;
    bool use_voxel_filter;
    ros::param::get("/to_grid_map_node/map_file_path", map_file_path);
    ros::param::get("/to_grid_map_node/voxel_leaf_size", voxel_leaf_size);
    ros::param::get("/to_grid_map_node/use_voxel_filter", use_voxel_filter);



    std::cout << "map_file_path:" << map_file_path << std::endl;
    std::cout << "use_voxel_filter:" << use_voxel_filter << std::endl;
    std::cout << "voxel_leaf_size:" << voxel_leaf_size << std::endl;


    ToGridMapNode to_grid_map_node(map_file_path,
                                   use_voxel_filter,
                                   voxel_leaf_size,
                                   node_handle
                                   );


    ros::spin();

    return 0;
}
