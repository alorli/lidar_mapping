///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_1_24
///////////////////////////////////////////////////////////////////////////////
#ifndef MAP_GENERATOR_H_
#define MAP_GENERATOR_H_

#include "src/registration/ndt_registration.h"
#include "geometry_msgs/TransformStamped.h"
#include "src/common/common.h"

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

    void GeneratePointCloudMap();
    void GeneratePointCloudRGBAMap();

private:
    std::string filelists_path_;
    std::string map_files_path_;

    double min_keyframe_distance_;
    double min_map_horizontal_radius_;
    int num_keyframe_submap_;

    double min_intensity_;
    double max_intensity_;

};

} // namespace mapping

#endif