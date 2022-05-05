///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2022_04_25
///////////////////////////////////////////////////////////////////////////////
#ifndef EKF_PROCESSOR_H_
#define EKF_PROCESSOR_H_

#include "src/common/time.h"
#include "src/common/time_conversion.h"

#include "src/registration/ekf_registration/common_types.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// #include<Eigen/Core>
#include <Eigen/Geometry>
// #include <vector>
#include <string>
#include "yaml-cpp/yaml.h"

namespace registration
{

class EkfProcessor
{
public:
    EkfProcessor(std::string cfg_file_path);
    ~EkfProcessor();

    void SetEkfState(EkfState ekf_state);
    EkfState GetEkfState();
    Eigen::Matrix<double, 23, 23> GetProcessCovariance();
    void SetProcessCovariance(Eigen::Matrix<double, 23, 23> process_covariance);

    void PredictState(double& delta_time, 
                      Eigen::Matrix<double, 12, 12>& Q, 
                      InputState& input_state);

private:
    YAML::Node cfg_file_;

    EkfState state_;
    Eigen::Matrix<double, 23, 23> process_covariance_;

    EkfParameter ekf_parameter_;
};

} // namespace registration

#endif
