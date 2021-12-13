
#include "src/common/common.h"

namespace common
{

double degree_to_radian(double degree)
{
    return (degree / 180.0 * M_PI);
}

double radian_to_degree(double radian)
{
    return (radian / M_PI * 180.0);
}


double Clamp(const double value, const double min, const double max)
{
  if (value > max)
  {
    return max;
  }
  if (value < min)
  {
    return min;
  }
  return value;
}

// 读一个目录下所有文件名
std::vector<std::string> read_file_list(const char *base_path_ptr)
{
    std::vector<std::string> result;
    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if((dir = opendir(base_path_ptr)) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while((ptr = readdir(dir)) != NULL)
    {
			  // current dir OR parrent dir
        if(strcmp(ptr->d_name, ".")==0 || strcmp(ptr->d_name,"..")==0)
				{
					continue;
				}
        else if(ptr->d_type == 8)    //file
        {
          result.push_back(std::string(ptr->d_name));
        }
        else if(ptr->d_type == 10)    //link file
        {
          result.push_back(std::string(ptr->d_name));
        }
        else if(ptr->d_type == 4)    //dir
        {
            memset(base,'\0',sizeof(base));
            strcpy(base,base_path_ptr);
            strcat(base,"/");
            strcat(base,ptr->d_name);
            result.push_back(std::string(ptr->d_name));

						// 递归找这个目录下子目录
            read_file_list(base);
        }
    }

    closedir(dir);
    return result;
}


}  // namespace common
