///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_12_03
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "src/processing/pointcloud_map_arealist.h"
#include "pcl/filters/voxel_grid.h"

#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>


namespace processing
{

PointcloudMapArealist::PointcloudMapArealist(std::string cfg_file_path,
                                             std::string project_directory_name)
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");

    std::string project_directory = cfg_file_["directory"]["project_directory"].as<std::string>();
    std::string map_main_directory = cfg_file_["directory"]["map"]["main_directory"].as<std::string>();
    std::string output_map_directory = cfg_file_["directory"]["map_partition"]["output_map_directory"].as<std::string>();
    arealist_filename_ = cfg_file_["directory"]["pointcloud_map_arealist"]["arealist_filename"].as<std::string>();
    output_map_directory_ = project_directory + project_directory_name + map_main_directory + output_map_directory;
    
}


PointcloudMapArealist::~PointcloudMapArealist()
{
}

void PointcloudMapArealist::GenerateArealistFile()
{
    std::vector<std::string> pointcoud_map_files;
    AreaList area_list;
    FindPointcloudMapfiles(output_map_directory_, pointcoud_map_files);
    CalculateAreaList(pointcoud_map_files, area_list);
    WriteArealist(output_map_directory_ + arealist_filename_, area_list);
}

void PointcloudMapArealist::FindPointcloudMapfiles(const std::string& path, 
                                                   std::vector<std::string>& pointcoud_map_files)
{
  std::string cmd = "find " + path + " -name '*.pcd' | sort";
  FILE *fp = popen(cmd.c_str(), "r");
  char line[ PATH_MAX ];

  while(fgets(line, sizeof(line), fp))
  {
    std::string buf(line);
    buf.erase(--buf.end()); // cut tail '\n'
    pointcoud_map_files.push_back(buf);
  }

  pclose(fp);
}

void PointcloudMapArealist::CalculateAreaList(std::vector<std::string>& pointcoud_map_files, AreaList& area_list)
{
  struct AreaWithPath area_with_path;
  for(int i=0; i<pointcoud_map_files.size(); i++)
  {
    std::cout << "i:" << pointcoud_map_files.at(i) << std::endl;

    if(CalculateArea(pointcoud_map_files.at(i), area_with_path) == 0)
    {
      area_list.push_back(area_with_path);
    }
  }
}

int PointcloudMapArealist::CalculateArea(const std::string& path, struct AreaWithPath& area_with_path)
{
	pcl::PointCloud<pcl::PointXYZI> pointcloud_map;

	if(pcl::io::loadPCDFile(path.c_str(), pointcloud_map) == -1)
	{
		std::cerr << "load failed " << path << std::endl;
		return -1;
	}

  int filename_index = path.find_last_of('/');
  std::string filename(path.substr(filename_index + 1) );

	area_with_path.path = filename;

	pcl::PointCloud<pcl::PointXYZI>::iterator it = pointcloud_map.begin();
	pcl::PointCloud<pcl::PointXYZI>::iterator end = pointcloud_map.end();

	//保存所有点的最大最小值
	area_with_path.x_min = area_with_path.x_max = it->x;   //最小最大值都是第一个点的坐标值
	area_with_path.y_min = area_with_path.y_max = it->y;
	area_with_path.z_min = area_with_path.z_max = it->z;

	for(it++; it != end; it++)
	{
		double x = it->x;
		double y = it->y;
		double z = it->z;
		if (x < area_with_path.x_min) area_with_path.x_min = x;   //寻找所有点的最小最大值
		if (x > area_with_path.x_max) area_with_path.x_max = x;
		if (y < area_with_path.y_min) area_with_path.y_min = y;
		if (y > area_with_path.y_max) area_with_path.y_max = y;
		if (z < area_with_path.z_min) area_with_path.z_min = z;
		if (z > area_with_path.z_max) area_with_path.z_max = z;
	}

	return 0;
}

//将double类型数据格式化成字符串类型
std::string PointcloudMapArealist::DoubleFormatToString(double value)
{
	char s[64];
	snprintf(s, sizeof(s), "%.3f", value);
	return std::string(s);
}

void PointcloudMapArealist::WriteTableTofile(const std::string& path, 
                                             const Table& table)
{
	std::ofstream ofs(path != "-" ? path.c_str() : "/dev/null");
	std::ostream& os = (path != "-") ? ofs : (std::cout);   //输出流对象重定位至文件流对象还是终端输出

	for(const std::vector<std::string>& columns : table)
	{
		std::string line;
		for(size_t i = 0; i < columns.size(); ++i)
		{
			if (i > 0) line += ",";   //每个字符串后面跟一个逗号分隔符","
			line += columns[i];
		}
		os << line << std::endl;   //将一整行发送到输出流对象
	}
}

void PointcloudMapArealist::WriteArealist(const std::string& path, const AreaList& area_list)
{
	Table table;
	for(const AreaWithPath& area_whith_path : area_list)
	{
		std::vector<std::string> columns;
		columns.push_back(area_whith_path.path);
		columns.push_back(DoubleFormatToString(area_whith_path.x_min));
		columns.push_back(DoubleFormatToString(area_whith_path.y_min));
		columns.push_back(DoubleFormatToString(area_whith_path.z_min));
		columns.push_back(DoubleFormatToString(area_whith_path.x_max));
		columns.push_back(DoubleFormatToString(area_whith_path.y_max));
		columns.push_back(DoubleFormatToString(area_whith_path.z_max));
		table.push_back(columns);
	}

	WriteTableTofile(path, table);
}


} // namespace processing
