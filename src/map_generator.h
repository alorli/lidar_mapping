///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_1_24
///////////////////////////////////////////////////////////////////////////////
#ifndef MAP_GENERATOR_H_
#define MAP_GENERATOR_H_

#include "src/registration/ndt_registration.h"
#include "geometry_msgs/TransformStamped.h"
#include "src/common/common.h"
#include <vector>

#include <string>


namespace mapping
{

struct RegistrationResultsFilelist
{
    long long allframe_id;
    common::Time time;
    bool is_gnss_constraint;
    std::string pcd_file_path;
    registration::AlignmentResult alignment_result;
};

class MapGenerator
{
public:
    MapGenerator();
    ~MapGenerator();

    void SetPathName(std::string filelists_path,
                     std::string map_files_path);

    void SetParameter(double min_keyframe_distance,
                      double min_map_horizontal_radius,
                      int num_keyframe_submap,
                      double min_intensity,
                      double max_intensity);

    void SetZOffset(double z_offset)
    {
      z_offset_ = z_offset;
    }

    // added by lichunjing 2021-03-27, 建图动态可视化用
    void SetMapVectorSize(int map_cloud_vector_size);

    void GeneratePointCloudMap();
    void GeneratePointCloudRGBMap();
    void GeneratePointCloudRGBAMap();

    // added by lichunjing 2021-03-27, 建图动态可视化用
    void LoadRegistrationFileLists();
    int UpdateMapCloudVector();

    const std::vector<pcl::PointCloud<registration::PointType>::Ptr>& GetMapCloudVector()
    {
      return map_cloud_vector_ptr_;
    }

    registration::AlignmentResult GetCurrentAlignmentResult()
    {
      return registration_results_iterator_->alignment_result;
    }


private:
    std::string filelists_path_;
    std::string map_files_path_;

    double min_keyframe_distance_;
    double min_map_horizontal_radius_;
    int num_keyframe_submap_;

    double z_offset_ = 0.0;

    double min_intensity_;
    double max_intensity_;

    // added by lichunjing 2021-03-27, 建图动态可视化用
    std::vector<RegistrationResultsFilelist> registration_results_filelists_;
    std::vector<RegistrationResultsFilelist>::iterator registration_results_iterator_;

    std::vector<pcl::PointCloud<registration::PointType>::Ptr> map_cloud_vector_ptr_;
    int map_cloud_vector_size_;
    long long frame_curent_cnt_ = 0;

};

} // namespace mapping

#endif
