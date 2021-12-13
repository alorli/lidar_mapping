///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_11_29
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "src/processing/map_partition.h"
#include "pcl/filters/voxel_grid.h"

#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>

namespace processing
{

MapPartition::MapPartition(std::string cfg_file_path,
                           std::string project_directory_name)
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");

    std::string project_directory = cfg_file_["directory"]["project_directory"].as<std::string>();
    std::string map_main_directory = cfg_file_["directory"]["map"]["main_directory"].as<std::string>();
    std::string closeloop_optimization_map_directory = cfg_file_["directory"]["map"]["closeloop_optimization_map"].as<std::string>();
    std::string output_map_directory = cfg_file_["directory"]["map_partition"]["output_map_directory"].as<std::string>();


    map_partition_parameter_.voxel_filter_size = cfg_file_["map_partition"]["voxel_filter_size"].as<double>();
    map_partition_parameter_.step_range_x = cfg_file_["map_partition"]["step_range_x"].as<double>();
    map_partition_parameter_.step_range_y = cfg_file_["map_partition"]["step_range_y"].as<double>();

    // raw_pointcloud_file_path_ = "/home/alorli/test_data/test/test_map/";
    raw_pointcloud_file_path_ = project_directory + project_directory_name + map_main_directory + closeloop_optimization_map_directory;
    output_pointcloud_file_path_ = project_directory + project_directory_name + map_main_directory + output_map_directory;

    std::cout << std::endl;
    std::cout << "\033[32m" << "----------MapPartition parameter:----------" << "\033[0m" << std::endl;
    std::cout << "\033[32m" << "map_partition_parameter_.voxel_filter_size:" << map_partition_parameter_.voxel_filter_size << "\033[0m" << std::endl;
    std::cout << "\033[32m" << "map_partition_parameter_.step_range_x:" << map_partition_parameter_.step_range_x << "\033[0m" << std::endl;
    std::cout << "\033[32m" << "map_partition_parameter_.step_range_y:" << map_partition_parameter_.step_range_y << "\033[0m" << std::endl;
}


MapPartition::~MapPartition()
{
}

void MapPartition::RunMapPartition()
{
    pointcloud_file_list_ = common::read_file_list(raw_pointcloud_file_path_.data());

    for(int i=0; i<pointcloud_file_list_.size(); i++)
    {
      std::cout << i << "\033[33m" << ":" << raw_pointcloud_file_path_ << pointcloud_file_list_.at(i) << "\033[0m" << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);

    // 读入所有 PCD文件中的点云
    for(int i = 0; i < pointcloud_file_list_.size(); i++)
    {
        // Loading input_cloud.
        std::string input = pointcloud_file_list_.at(i);

        if(pcl::io::loadPCDFile<pcl::PointXYZI>(raw_pointcloud_file_path_ + input,
                                                *input_cloud_ptr) == -1)          //* load the file
        {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        }

        std::cout << "i:" << i << " :Loaded "
                  << input_cloud_ptr->width * input_cloud_ptr->height
                  << " data points from test_pcd.pcd "
                  << std::endl;

        // 针对每一块小地图，进行体素滤波
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;

        voxel_grid_filter.setLeafSize(map_partition_parameter_.voxel_filter_size,
                                      map_partition_parameter_.voxel_filter_size,
                                      map_partition_parameter_.voxel_filter_size);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(*input_cloud_ptr));
        voxel_grid_filter.setInputCloud(cloud_ptr);
        voxel_grid_filter.filter(*cloud_ptr);

        std::cout << "after voxel filter-------i:" << i << " :Loaded "
                    << cloud_ptr->width * cloud_ptr->height
                    << " data points from test_pcd.pcd "
                    << std::endl;

        *all_cloud_ptr += *cloud_ptr;
	}

    std::cout << "all_cloud_ptr.size: "
              << all_cloud_ptr->width * all_cloud_ptr->height
              << std::endl;

    //遍历寻找各坐标轴的最大最小值：
    struct Area area;

    //在输入点云中寻找个坐标轴的最大最小值
    find_min_max(all_cloud_ptr, area);
    
    // x轴的子图个数
    int x_num = static_cast<int>((area.x_max - area.x_min)/map_partition_parameter_.step_range_x) + 1;

	// y轴的子图个数
    int y_num = static_cast<int>((area.y_max - area.y_min)/map_partition_parameter_.step_range_y) + 1;

    std::cout << "x_num: " << x_num << "  y_num: " << y_num << std::endl;

    std::vector<int> x_index;
    std::vector<int> y_index;
    // 保存每个小子图点云
    pcl::PointCloud<pcl::PointXYZI> area_cloud[x_num*y_num];

        // 获取输入点云的初始和结束的迭代器
    pcl::PointCloud<pcl::PointXYZI>::iterator it = all_cloud_ptr->begin();

    pcl::PointCloud<pcl::PointXYZI>::iterator end = all_cloud_ptr->end();

    int file_num = 0;

    //用于显示进度条
    char bar[102];
    const char* flag = "-\\|/";
    bar[0] = '\0';

    int size_point = (all_cloud_ptr->width * all_cloud_ptr->height);
    int progress_step =  size_point / 100;
    int bar_i = 0;

    file_num++;

    // 根据点的坐标，计算该点所在区域的索引值
    for(it++; it != end; it++)
    {
        double x = it->x;
        double y = it->y;

        int x_index = (int)((x - area.x_min)/map_partition_parameter_.step_range_x);
        int y_index = (int)((y - area.y_min)/map_partition_parameter_.step_range_y);

        int index = y_index * x_num + x_index;

        area_cloud[index].push_back(*it);

        // 显示进度条
        bar_i++;

        int ii = bar_i / progress_step;

        printf("\033[1;32;40m [file:%d/%d][%-100s][%d%%] [%c] \r",
                file_num, x_num*y_num, bar, ii, flag[ii%4]);

        fflush(stdout);
        bar[ii++] = '#';
        bar[ii] = '\0';
    }

    printf("\033[0m \n");


    	// 遍历每一个小区域
	for(int i=0; i< x_num*y_num; i++)
	{
		if(area_cloud[i].size() != 0)
		{
			int y_index = (int)(i/map_partition_parameter_.step_range_x);
			int x_index = i - y_index * map_partition_parameter_.step_range_x;

            // 定义文件输出路径：
            char s_i[64];
            char s_j[64];

            snprintf(s_i, sizeof(s_i), "%04d", x_index);   //"%04d"是格式控制字符串，其中04表示，输出字符串宽度为4位，不足的前面补0
            snprintf(s_j, sizeof(s_j), "%04d", y_index);

            std::string output_path = output_pointcloud_file_path_ + "bin_" + std::string(s_i) + "_" + std::string(s_j) + ".pcd";

            std::cout << "--------------output_path:" << output_path << std::endl;
            // 将该 area区域中的点云保存到文件中：
            if(pcl::io::savePCDFileBinary(output_path, area_cloud[i]) == -1)
            {
                std::cout << "Failed saving " << output_path << std::endl;
            }


            std::cout << "[" << i << "/" << x_num*y_num-1 << "]" << std::endl;
		}
	}
}

void MapPartition::find_min_max(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr,
	                            struct Area &area)
{
  // 获取输入点云的初始和结束的迭代器
  pcl::PointCloud<pcl::PointXYZI>::iterator it = input_cloud_ptr->begin();
  pcl::PointCloud<pcl::PointXYZI>::iterator end = input_cloud_ptr->end();

  // 保存所有点的最大最小值
	// 初始时最小最大值都是第一个点的坐标值
  area.x_min = area.x_max = it->x;
  area.y_min = area.y_max = it->y;
  area.z_min = area.z_max = it->z;


  // 用于显示进度条
  char bar[102];
  const char* flag = "-\\|/";
  bar[0] = '\0';

  int size_point = (input_cloud_ptr->width * input_cloud_ptr->height);
  int progress_step =  size_point / 100;
  int bar_i = 0;

	// 遍历所有点的坐标
  for(it++; it != end; it++)
  {
    double x = it->x;
    double y = it->y;
    double z = it->z;

		//寻找所有点的最小最大值
    if(x < area.x_min) area.x_min = x;
    if(x > area.x_max) area.x_max = x;
    if(y < area.y_min) area.y_min = y;
    if(y > area.y_max) area.y_max = y;
    if(z < area.z_min) area.z_min = z;
    if(z > area.z_max) area.z_max = z;

    //显示进度条
    bar_i++;

    int ii = bar_i / progress_step;
    printf("\033[1;32;40m reading...[%-100s][%d%%] [%c] \r", bar, ii, flag[ii%4]);
    fflush(stdout);
    bar[ii++] = '#';
    bar[ii] = '\0';
  }

  printf("\033[0m \n");

  std::cout << "x_min: " << area.x_min << "  x_max: " << area.x_max
	          << "  delta_x: " << area.x_max - area.x_min << std::endl
            << "y_min: " << area.y_min << "  y_max: " << area.y_max
						<< "  delta_y: " << area.y_max - area.y_min << std::endl
            << "z_min: " << area.z_min << "  z_max: " << area.z_max
						<< "  delta_z: " << area.z_max - area.z_min << std::endl
            << std::endl;
}


} // namespace processing
