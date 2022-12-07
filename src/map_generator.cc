///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_23
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>
#include <fstream>
// #include "src/map_builder.h"
#include "pcl/filters/voxel_grid.h"
#include "src/map_generator.h"


namespace mapping
{

MapGenerator::MapGenerator()
{
}

MapGenerator::~MapGenerator()
{
}

void MapGenerator::SetPathName(std::string filelists_path,
                               std::string map_files_path)
{
    filelists_path_ = filelists_path;
    map_files_path_ = map_files_path;
}

void MapGenerator::SetParameter(double min_keyframe_distance,
                                double min_map_horizontal_radius,
                                int num_keyframe_submap,
                                double min_intensity,
                                double max_intensity)
{
    min_keyframe_distance_ = min_keyframe_distance;
    min_map_horizontal_radius_ = min_map_horizontal_radius;
    num_keyframe_submap_ = num_keyframe_submap;
    min_intensity_ = min_intensity;
    max_intensity_ = max_intensity;
}

void MapGenerator::SetMapVectorSize(int map_cloud_vector_size)
{
    map_cloud_vector_size_ = map_cloud_vector_size;
}

void MapGenerator::LoadRegistrationFileLists()
{
    std::vector<RegistrationResultsFilelist> registration_results_filelists;

    std::ifstream filelists;
    filelists.open(filelists_path_);
    RegistrationResultsFilelist registration_results_filelist;
    long long time;
    double x_last = 0, y_last = 0;

    while(filelists >> registration_results_filelist.allframe_id
                    >> time
                    >> registration_results_filelist.is_gnss_constraint
                    >> registration_results_filelist.pcd_file_path
                    >> registration_results_filelist.alignment_result.is_converged
                    >> registration_results_filelist.alignment_result.fitness_score
                    >> registration_results_filelist.alignment_result.time_duration_ms
                    >> registration_results_filelist.alignment_result.final_num_iteration
                    >> registration_results_filelist.alignment_result.final_transform(0,0) >> registration_results_filelist.alignment_result.final_transform(0,1) >> registration_results_filelist.alignment_result.final_transform(0,2) >> registration_results_filelist.alignment_result.final_transform(0,3)
                    >> registration_results_filelist.alignment_result.final_transform(1,0) >> registration_results_filelist.alignment_result.final_transform(1,1) >> registration_results_filelist.alignment_result.final_transform(1,2) >> registration_results_filelist.alignment_result.final_transform(1,3)
                    >> registration_results_filelist.alignment_result.final_transform(2,0) >> registration_results_filelist.alignment_result.final_transform(2,1) >> registration_results_filelist.alignment_result.final_transform(2,2) >> registration_results_filelist.alignment_result.final_transform(2,3)
                    >> registration_results_filelist.alignment_result.final_transform(3,0) >> registration_results_filelist.alignment_result.final_transform(3,1) >> registration_results_filelist.alignment_result.final_transform(3,2) >> registration_results_filelist.alignment_result.final_transform(3,3)
    )
    {
        double x = registration_results_filelist.alignment_result.final_transform(0,3);
        double y = registration_results_filelist.alignment_result.final_transform(1,3);
        double shift_distance = sqrt(pow(x-x_last, 2) + pow(y - y_last,2));

        if(shift_distance > min_keyframe_distance_)
        {
            x_last = x;
            y_last = y;
            registration_results_filelist.time = common::FromUniversal(time);
            registration_results_filelists_.push_back(registration_results_filelist);

            // std::cout << "\033[33m size:" << registration_results_filelists_.size()
            //             << " allframe_id: " << registration_results_filelist.allframe_id
            //             << " time: " << registration_results_filelist.time
            //             << " pcd_file_path: " << registration_results_filelist.pcd_file_path
            //             << " final_transform: " << registration_results_filelist.alignment_result.final_transform
            //             << "\033[0m"
            //             << std::endl;
        }
    }
    filelists.close();

    // 获取初始迭代器
    registration_results_iterator_ = registration_results_filelists_.begin();

    for(int i=0; i<map_cloud_vector_size_; i++)
    {
        pcl::PointCloud<registration::PointType>::Ptr cloud_ptr(new pcl::PointCloud<registration::PointType>);
        map_cloud_vector_ptr_.push_back(cloud_ptr);
    }
}

int MapGenerator::UpdateMapCloudVector()
{
    if(registration_results_iterator_ != registration_results_filelists_.end())
    {
        pcl::PointCloud<registration::PointType>::Ptr cloud_raw_ptr(new pcl::PointCloud<registration::PointType>);
        pcl::PointCloud<registration::PointType>::Ptr cloud_filtered_ptr(new pcl::PointCloud<registration::PointType>);
        pcl::PointCloud<registration::PointType>::Ptr cloud_transformed_ptr(new pcl::PointCloud<registration::PointType>);

        if(pcl::io::loadPCDFile<registration::PointType>(registration_results_iterator_->pcd_file_path, *cloud_raw_ptr) == -1)
        {
          PCL_ERROR("Couldn't load pcd file \n");
        }

        registration::PointType point;
        for(pcl::PointCloud<registration::PointType>::const_iterator item = cloud_raw_ptr->begin();
            item != cloud_raw_ptr->end();
            item++)
        {
            point.x = (double)item->x;
            point.y = (double)item->y;
            point.z = (double)item->z;

            if(typeid(registration::PointType) == typeid(pcl::PointXYZI))
            {
                point.intensity = (double)item->intensity;
            }

            double horizontal_radius = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));

            if(horizontal_radius > min_map_horizontal_radius_)
            {
                cloud_filtered_ptr->push_back(point);
            }
        }


        if(frame_curent_cnt_>=map_cloud_vector_size_)
        {
          frame_curent_cnt_ = 0;
        }

        pcl::transformPointCloud(*cloud_filtered_ptr,
                                 *map_cloud_vector_ptr_.at(frame_curent_cnt_),
                                 registration_results_iterator_->alignment_result.final_transform);

        std::cout << "-------------map_cloud_vector_ptr_.at(i)->size():" << map_cloud_vector_ptr_.at(frame_curent_cnt_)->size() << std::endl;



        registration_results_iterator_++;

        std::cout << "\033[34m" << "frame_curent_cnt_ :" << frame_curent_cnt_++ << "\033[0m" << std::endl;

        return frame_curent_cnt_ - 1;

      // if(index++ > num_keyframe_submap_)
      // {
      //     std::cout << "\033[34m" << "save map :" << map_files_path_ << pcd_cnt << ".pcd" << "\033[0m" << std::endl;
      //     index = 0;
      //     std::stringstream pcd_file_name;
      //     pcd_file_name << map_files_path_ << pcd_cnt++ << ".pcd";
      //     pcl::io::savePCDFileBinary(pcd_file_name.str().c_str(), *map_cloud_ptr);
      //     map_cloud_ptr->clear();
      // }
    }
    else
    {
       std::cout << "\033[34m" << "--------------added all lidar frame to map!---------------" << "\033[0m" << std::endl;
       return -1;
    }

}


void MapGenerator::GeneratePointCloudMap()
{
    std::vector<RegistrationResultsFilelist> registration_results_filelists;

    std::ifstream filelists;
    filelists.open(filelists_path_);
    RegistrationResultsFilelist registration_results_filelist;
    long long time;
    double x_last = 0, y_last = 0;

    std::cout << "----------------------------------filelists_path_:" << filelists_path_ << std::endl;

    while(filelists >> registration_results_filelist.allframe_id
                    >> time
                    >> registration_results_filelist.is_gnss_constraint
                    >> registration_results_filelist.pcd_file_path
                    >> registration_results_filelist.alignment_result.is_converged
                    >> registration_results_filelist.alignment_result.fitness_score
                    >> registration_results_filelist.alignment_result.time_duration_ms
                    >> registration_results_filelist.alignment_result.final_num_iteration
                    >> registration_results_filelist.alignment_result.final_transform(0,0) >> registration_results_filelist.alignment_result.final_transform(0,1) >> registration_results_filelist.alignment_result.final_transform(0,2) >> registration_results_filelist.alignment_result.final_transform(0,3)
                    >> registration_results_filelist.alignment_result.final_transform(1,0) >> registration_results_filelist.alignment_result.final_transform(1,1) >> registration_results_filelist.alignment_result.final_transform(1,2) >> registration_results_filelist.alignment_result.final_transform(1,3)
                    >> registration_results_filelist.alignment_result.final_transform(2,0) >> registration_results_filelist.alignment_result.final_transform(2,1) >> registration_results_filelist.alignment_result.final_transform(2,2) >> registration_results_filelist.alignment_result.final_transform(2,3)
                    >> registration_results_filelist.alignment_result.final_transform(3,0) >> registration_results_filelist.alignment_result.final_transform(3,1) >> registration_results_filelist.alignment_result.final_transform(3,2) >> registration_results_filelist.alignment_result.final_transform(3,3)
    )
    {
        double x = registration_results_filelist.alignment_result.final_transform(0,3);
        double y = registration_results_filelist.alignment_result.final_transform(1,3);
        double shift_distance = sqrt(pow(x-x_last, 2) + pow(y - y_last,2));

        if(shift_distance > min_keyframe_distance_)
        {
            x_last = x;
            y_last = y;
            registration_results_filelist.time = common::FromUniversal(time);
            registration_results_filelists.push_back(registration_results_filelist);

            // std::cout << "\033[33m size:" << registration_results_filelists.size()
            //             << " allframe_id: " << registration_results_filelist.allframe_id
            //             << " time: " << registration_results_filelist.time
            //             << " pcd_file_path: " << registration_results_filelist.pcd_file_path
            //             << " final_transform: " << registration_results_filelist.alignment_result.final_transform
            //             << "\033[0m"
            //             << std::endl;
        }
    }
    filelists.close();


    int index = 0;
    int pcd_cnt = 0;
    pcl::PointCloud<registration::PointType>::Ptr map_cloud_ptr(new pcl::PointCloud<registration::PointType>);

    for(std::vector<RegistrationResultsFilelist>::iterator item_registration_results = registration_results_filelists.begin();
                                                           item_registration_results != registration_results_filelists.end();
                                                           item_registration_results++)
    {
        pcl::PointCloud<registration::PointType>::Ptr cloud_raw_ptr(new pcl::PointCloud<registration::PointType>);
        pcl::PointCloud<registration::PointType>::Ptr cloud_filtered_ptr(new pcl::PointCloud<registration::PointType>);
        pcl::PointCloud<registration::PointType>::Ptr cloud_transformed_ptr(new pcl::PointCloud<registration::PointType>);

        if(pcl::io::loadPCDFile<registration::PointType>(item_registration_results->pcd_file_path, *cloud_raw_ptr) == -1)
        {
            PCL_ERROR("Couldn't load pcd file \n");
        }




        registration::PointType point;
        for(pcl::PointCloud<registration::PointType>::const_iterator item = cloud_raw_ptr->begin();
            item != cloud_raw_ptr->end();
            item++)
        {
            point.x = (double)item->x;
            point.y = (double)item->y;
            point.z = (double)item->z;

            if(typeid(registration::PointType) == typeid(pcl::PointXYZI))
            {
                point.intensity = (double)item->intensity;
            }

            double horizontal_radius = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));

            if(horizontal_radius > min_map_horizontal_radius_)
            {
                cloud_filtered_ptr->push_back(point);
            }
        }

        pcl::transformPointCloud(*cloud_filtered_ptr,
                                 *cloud_transformed_ptr,
                                 item_registration_results->alignment_result.final_transform);

        *map_cloud_ptr += *cloud_transformed_ptr;

        // std::cout << "\033[34m" << "index :" << index << "\033[0m" << std::endl;

        if(index++ > num_keyframe_submap_)
        {
            std::cout << "\033[34m" << "save map :" << map_files_path_ << pcd_cnt << ".pcd" << "\033[0m" << std::endl;
            index = 0;
            std::stringstream pcd_file_name;
            pcd_file_name << map_files_path_ << pcd_cnt++ << ".pcd";
			      pcl::io::savePCDFileBinary(pcd_file_name.str().c_str(), *map_cloud_ptr);
            map_cloud_ptr->clear();
        }

    }
}

void MapGenerator::GeneratePointCloudRGBMap()
{
    std::vector<RegistrationResultsFilelist> registration_results_filelists;

    std::ifstream filelists;
    filelists.open(filelists_path_);
    RegistrationResultsFilelist registration_results_filelist;
    long long time;
    double x_last = 0, y_last = 0;

    while(filelists >> registration_results_filelist.allframe_id
                    >> time
                    >> registration_results_filelist.is_gnss_constraint
                    >> registration_results_filelist.pcd_file_path
                    >> registration_results_filelist.alignment_result.is_converged
                    >> registration_results_filelist.alignment_result.fitness_score
                    >> registration_results_filelist.alignment_result.time_duration_ms
                    >> registration_results_filelist.alignment_result.final_num_iteration
                    >> registration_results_filelist.alignment_result.final_transform(0,0) >> registration_results_filelist.alignment_result.final_transform(0,1) >> registration_results_filelist.alignment_result.final_transform(0,2) >> registration_results_filelist.alignment_result.final_transform(0,3)
                    >> registration_results_filelist.alignment_result.final_transform(1,0) >> registration_results_filelist.alignment_result.final_transform(1,1) >> registration_results_filelist.alignment_result.final_transform(1,2) >> registration_results_filelist.alignment_result.final_transform(1,3)
                    >> registration_results_filelist.alignment_result.final_transform(2,0) >> registration_results_filelist.alignment_result.final_transform(2,1) >> registration_results_filelist.alignment_result.final_transform(2,2) >> registration_results_filelist.alignment_result.final_transform(2,3)
                    >> registration_results_filelist.alignment_result.final_transform(3,0) >> registration_results_filelist.alignment_result.final_transform(3,1) >> registration_results_filelist.alignment_result.final_transform(3,2) >> registration_results_filelist.alignment_result.final_transform(3,3)
    )
    {
        double x = registration_results_filelist.alignment_result.final_transform(0,3);
        double y = registration_results_filelist.alignment_result.final_transform(1,3);
        double shift_distance = sqrt(pow(x-x_last, 2) + pow(y - y_last,2));

        if(shift_distance > min_keyframe_distance_)
        {
            x_last = x;
            y_last = y;
            registration_results_filelist.time = common::FromUniversal(time);
            registration_results_filelists.push_back(registration_results_filelist);

            // std::cout << "\033[33m size:" << registration_results_filelists.size()
            //             << " allframe_id: " << registration_results_filelist.allframe_id
            //             << " time: " << registration_results_filelist.time
            //             << " pcd_file_path: " << registration_results_filelist.pcd_file_path
            //             << " final_transform: " << registration_results_filelist.alignment_result.final_transform
            //             << "\033[0m"
            //             << std::endl;
        }
    }
    filelists.close();


    int index = 0;
    int pcd_cnt = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    for(std::vector<RegistrationResultsFilelist>::iterator item_registration_results = registration_results_filelists.begin();
                                                           item_registration_results != registration_results_filelists.end();
                                                           item_registration_results++)
    {
        pcl::PointCloud<registration::PointType>::Ptr cloud_raw_ptr(new pcl::PointCloud<registration::PointType>);
        pcl::PointCloud<registration::PointType>::Ptr cloud_filtered_ptr(new pcl::PointCloud<registration::PointType>);
        pcl::PointCloud<registration::PointType>::Ptr cloud_transformed_ptr(new pcl::PointCloud<registration::PointType>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed_rgba_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        if(pcl::io::loadPCDFile<registration::PointType>(item_registration_results->pcd_file_path, *cloud_raw_ptr) == -1)
        {
            PCL_ERROR("Couldn't load pcd file \n");
        }




        registration::PointType point;
        for(pcl::PointCloud<registration::PointType>::const_iterator item = cloud_raw_ptr->begin();
            item != cloud_raw_ptr->end();
            item++)
        {
            point.x = (double)item->x;
            point.y = (double)item->y;
            point.z = (double)item->z;

            if(typeid(registration::PointType) == typeid(pcl::PointXYZI))
            {
                point.intensity = (double)item->intensity;
            }

            double horizontal_radius = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));

            if(horizontal_radius > min_map_horizontal_radius_)
            {
                cloud_filtered_ptr->push_back(point);
            }
        }

        pcl::transformPointCloud(*cloud_filtered_ptr,
                                 *cloud_transformed_ptr,
                                 item_registration_results->alignment_result.final_transform);

        pcl::PointXYZRGB point_xyz_rgb;
        for(pcl::PointCloud<registration::PointType>::const_iterator item = cloud_transformed_ptr->begin();
            item != cloud_transformed_ptr->end();
            item++)
        {
            point_xyz_rgb.x = (double)item->x;
            point_xyz_rgb.y = (double)item->y;
            point_xyz_rgb.z = (double)item->z + z_offset_;
            double gray = 0.0;

            if(typeid(registration::PointType) == typeid(pcl::PointXYZI))
            {
                point.intensity = (double)item->intensity;
                gray = common::Clamp(((double)item->intensity - min_intensity_) / (max_intensity_ - min_intensity_),
                                     0.0,
                                     1.0);
            }

            point_xyz_rgb.r = gray*255;
            point_xyz_rgb.g = gray*255;
            point_xyz_rgb.b = gray*255;

            double horizontal_radius = sqrt(pow(point_xyz_rgb.x, 2.0) + pow(point_xyz_rgb.y, 2.0));

            if(horizontal_radius > min_map_horizontal_radius_)
            {
                cloud_transformed_rgba_ptr->push_back(point_xyz_rgb);
            }
        }


        *map_cloud_ptr += *cloud_transformed_rgba_ptr;

        // std::cout << "\033[34m" << "index :" << index << "\033[0m" << std::endl;

        if(index++ > num_keyframe_submap_)
        {
            std::cout << "\033[34m" << "save map :" << map_files_path_ << pcd_cnt << ".pcd" << "\033[0m" << std::endl;
            index = 0;
            std::stringstream pcd_file_name;
            pcd_file_name << map_files_path_ << pcd_cnt++ << ".pcd";
            pcl::io::savePCDFileBinary(pcd_file_name.str().c_str(), *map_cloud_ptr);
            map_cloud_ptr->clear();
        }

    }
}

void MapGenerator::GeneratePointCloudRGBAMap()
{
    std::vector<RegistrationResultsFilelist> registration_results_filelists;

    std::ifstream filelists;
    filelists.open(filelists_path_);
    RegistrationResultsFilelist registration_results_filelist;
    long long time;
    double x_last = 0, y_last = 0;

    while(filelists >> registration_results_filelist.allframe_id
                    >> time
                    >> registration_results_filelist.is_gnss_constraint
                    >> registration_results_filelist.pcd_file_path
                    >> registration_results_filelist.alignment_result.is_converged
                    >> registration_results_filelist.alignment_result.fitness_score
                    >> registration_results_filelist.alignment_result.time_duration_ms
                    >> registration_results_filelist.alignment_result.final_num_iteration
                    >> registration_results_filelist.alignment_result.final_transform(0,0) >> registration_results_filelist.alignment_result.final_transform(0,1) >> registration_results_filelist.alignment_result.final_transform(0,2) >> registration_results_filelist.alignment_result.final_transform(0,3)
                    >> registration_results_filelist.alignment_result.final_transform(1,0) >> registration_results_filelist.alignment_result.final_transform(1,1) >> registration_results_filelist.alignment_result.final_transform(1,2) >> registration_results_filelist.alignment_result.final_transform(1,3)
                    >> registration_results_filelist.alignment_result.final_transform(2,0) >> registration_results_filelist.alignment_result.final_transform(2,1) >> registration_results_filelist.alignment_result.final_transform(2,2) >> registration_results_filelist.alignment_result.final_transform(2,3)
                    >> registration_results_filelist.alignment_result.final_transform(3,0) >> registration_results_filelist.alignment_result.final_transform(3,1) >> registration_results_filelist.alignment_result.final_transform(3,2) >> registration_results_filelist.alignment_result.final_transform(3,3)
    )
    {
        double x = registration_results_filelist.alignment_result.final_transform(0,3);
        double y = registration_results_filelist.alignment_result.final_transform(1,3);
        double shift_distance = sqrt(pow(x-x_last, 2) + pow(y - y_last,2));

        if(shift_distance > min_keyframe_distance_)
        {
            x_last = x;
            y_last = y;
            registration_results_filelist.time = common::FromUniversal(time);
            registration_results_filelists.push_back(registration_results_filelist);

            // std::cout << "\033[33m size:" << registration_results_filelists.size()
            //             << " allframe_id: " << registration_results_filelist.allframe_id
            //             << " time: " << registration_results_filelist.time
            //             << " pcd_file_path: " << registration_results_filelist.pcd_file_path
            //             << " final_transform: " << registration_results_filelist.alignment_result.final_transform
            //             << "\033[0m"
            //             << std::endl;
        }
    }
    filelists.close();


    int index = 0;
    int pcd_cnt = 0;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

    for(std::vector<RegistrationResultsFilelist>::iterator item_registration_results = registration_results_filelists.begin();
                                                           item_registration_results != registration_results_filelists.end();
                                                           item_registration_results++)
    {
        pcl::PointCloud<registration::PointType>::Ptr cloud_raw_ptr(new pcl::PointCloud<registration::PointType>);
        pcl::PointCloud<registration::PointType>::Ptr cloud_filtered_ptr(new pcl::PointCloud<registration::PointType>);
        pcl::PointCloud<registration::PointType>::Ptr cloud_transformed_ptr(new pcl::PointCloud<registration::PointType>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_transformed_rgba_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

        if(pcl::io::loadPCDFile<registration::PointType>(item_registration_results->pcd_file_path, *cloud_raw_ptr) == -1)
    		{
    			PCL_ERROR("Couldn't load pcd file \n");
    		}




        registration::PointType point;
        for(pcl::PointCloud<registration::PointType>::const_iterator item = cloud_raw_ptr->begin();
            item != cloud_raw_ptr->end();
            item++)
        {
            point.x = (double)item->x;
            point.y = (double)item->y;
            point.z = (double)item->z;

            if(typeid(registration::PointType) == typeid(pcl::PointXYZI))
            {
                point.intensity = (double)item->intensity;
            }

            double horizontal_radius = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));

            if(horizontal_radius > min_map_horizontal_radius_)
            {
                cloud_filtered_ptr->push_back(point);
            }
        }

        pcl::transformPointCloud(*cloud_filtered_ptr,
                								 *cloud_transformed_ptr,
                								 item_registration_results->alignment_result.final_transform);

        double alpha = 255.0;
        pcl::PointXYZRGBA point_xyz_rgba;
        for(pcl::PointCloud<registration::PointType>::const_iterator item = cloud_transformed_ptr->begin();
            item != cloud_transformed_ptr->end();
            item++)
        {
            point_xyz_rgba.x = (double)item->x;
            point_xyz_rgba.y = (double)item->y;
            point_xyz_rgba.z = (double)item->z;
            double gray = 0.0;

            if(typeid(registration::PointType) == typeid(pcl::PointXYZI))
            {
                point.intensity = (double)item->intensity;
                gray = common::Clamp(((double)item->intensity - min_intensity_) / (max_intensity_ - min_intensity_),
                                     0.0,
                                     1.0);
            }

            point_xyz_rgba.r = gray*255;
            point_xyz_rgba.g = gray*255;
            point_xyz_rgba.b = gray*255;
            point_xyz_rgba.a = alpha;

            double horizontal_radius = sqrt(pow(point_xyz_rgba.x, 2.0) + pow(point_xyz_rgba.y, 2.0));

            if(horizontal_radius > min_map_horizontal_radius_)
            {
                cloud_transformed_rgba_ptr->push_back(point_xyz_rgba);
            }
        }


        *map_cloud_ptr += *cloud_transformed_rgba_ptr;

        // std::cout << "\033[34m" << "index :" << index << "\033[0m" << std::endl;

        if(index++ > num_keyframe_submap_)
        {
            std::cout << "\033[34m" << "save map :" << map_files_path_ << pcd_cnt << ".pcd" << "\033[0m" << std::endl;
            index = 0;
            std::stringstream pcd_file_name;
            pcd_file_name << map_files_path_ << pcd_cnt++ << ".pcd";
            pcl::io::savePCDFileBinary(pcd_file_name.str().c_str(), *map_cloud_ptr);
            map_cloud_ptr->clear();
        }

    }
}


}  // namespace mapping
