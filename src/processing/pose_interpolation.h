///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_02_24
///////////////////////////////////////////////////////////////////////////////
#ifndef POSE_INTERPOLATION_H_
#define POSE_INTERPOLATION_H_


#include "src/map_generator.h"
#include "src/common/common.h"

#include "yaml-cpp/yaml.h"
#include <string>

namespace processing
{

struct InterpolationParameter
{
    double extract_frame_distance;
    double min_delta_sick_time;
};

class PoseInterpolation
{
public:
    PoseInterpolation(std::string cfg_file_path,
                      std::string project_directory_name);
    ~PoseInterpolation();

    void LoadReferencePoses();    
    void LoadInterpolationPoses();
    void RunIterpolation();
    void LinearInterpolation();
    void SaveInterpolationResultsFile();


private:
    YAML::Node cfg_file_;

    InterpolationParameter interpolation_parameter_;

    std::vector<mapping::RegistrationResultsFilelist> reference_poses_filelists_;
    std::vector<mapping::RegistrationResultsFilelist> interpolation_poses_filelists_;
    std::vector<mapping::RegistrationResultsFilelist> interpolation_poses_results_filelists_;

    Eigen::Matrix4f  transform_sick_to_vlp_;

    std::string reference_poses_file_path_;
    std::string interpolation_poses_file_path_;
    std::string interpolation_results_file_path_;

    std::ofstream interpolation_results_filelist_;
};

} // namespace processing

#endif