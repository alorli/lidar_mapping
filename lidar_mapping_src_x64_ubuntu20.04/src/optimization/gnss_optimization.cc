///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_23
///////////////////////////////////////////////////////////////////////////////

// #include <iostream>
// #include <sstream>
// #include <fstream>
#include "src/optimization/gnss_optimization.h"

namespace optimization
{

GnssOptimization::GnssOptimization(std::string cfg_file_path)
    : cfg_file_path_(cfg_file_path),
      initial_rotation_matrix_(Eigen::Matrix3d::Identity()),
      gnss_position_init_(Eigen::Vector3d::Zero()),
      is_get_gnss_position_init_(false)
{
    // 雷达里程计信息矩阵
    lidar_odom_information_ << 320.0,          0.0,         0.0,          0.0,          0.0,           0.0,
                               0.0,            320.0,       0.0,          0.0,          0.0,           0.0,
                               0.0,            0.0,         500.0,        0.0,          0.0,           0.0,
                               0.0,            0.0,         0.0,          20000.0,      0.0,           0.0,
                               0.0,            0.0,         0.0,          0.0,          20000.0,       0.0,
                               0.0,            0.0,         0.0,          0.0,          0.0,           20000.0;

    // 雷达-gnss信息矩阵
    lidar_gnss_information_ << 25.0,          0.0,           0.0,
                               0.0,            25.0,         0.0,
                               0.0,            0.0,           10.0;

}

GnssOptimization::~GnssOptimization()
{
    g2o_file_before_optimization_.close();
    g2o_file_after_optimization_.close();
    gnss_optimization_results_filelist_.close();
    gnss_init_pose_file_.close();
}

void GnssOptimization::LoadConfig()
{
    cfg_file_ = YAML::LoadFile(cfg_file_path_ + "/cfg.yaml");

    std::string project_directory = cfg_file_["directory"]["project_directory"].as<std::string>();
    std::string optimization_main_directory = cfg_file_["directory"]["optimization"]["main_directory"].as<std::string>();

    gnss_optimization_parameter_.lidar_keyframe_distance = cfg_file_["gnss_optimization"]["gnss_optimization_parameter"]["lidar_keyframe_distance"].as<double>();
    gnss_optimization_parameter_.use_distance_lidar_pose_estimate_init_angle = cfg_file_["gnss_optimization"]["gnss_optimization_parameter"]["use_distance_lidar_pose_estimate_init_angle"].as<double>();
    gnss_optimization_parameter_.distance_interval_add_prior_gnss_pose = cfg_file_["gnss_optimization"]["gnss_optimization_parameter"]["distance_interval_add_prior_gnss_pose"].as<double>();
    gnss_optimization_parameter_.max_iterations = cfg_file_["gnss_optimization"]["gnss_optimization_parameter"]["max_iterations"].as<int>();

    lever_arm_parameter_.use_lever_arm = cfg_file_["gnss_optimization"]["lever_arm_parameter"]["use_lever_arm"].as<bool>();
    lever_arm_parameter_.lever_arm_x = cfg_file_["gnss_optimization"]["lever_arm_parameter"]["lever_arm_x"].as<double>();
    lever_arm_parameter_.lever_arm_y = cfg_file_["gnss_optimization"]["lever_arm_parameter"]["lever_arm_y"].as<double>();
    lever_arm_parameter_.lever_arm_z = cfg_file_["gnss_optimization"]["lever_arm_parameter"]["lever_arm_z"].as<double>();
    
    std::string gnss_before_optimization_file = cfg_file_["directory"]["optimization"]["gnss_before_optimization_file"].as<std::string>();
    std::string gnss_after_optimization_file = cfg_file_["directory"]["optimization"]["gnss_after_optimization_file"].as<std::string>();
    std::string gnss_optimization_results_file = cfg_file_["directory"]["optimization"]["gnss_optimization_results_file"].as<std::string>();
    std::string gnss_main_directory = cfg_file_["directory"]["gnss"]["main_directory"].as<std::string>();
    std::string gnss_init_pose_file = cfg_file_["directory"]["gnss"]["gnss_init_pose_file"].as<std::string>();
    std::string g2o_file_before_optimization_path = project_directory + optimization_main_directory + gnss_before_optimization_file;
    std::string g2o_file_after_optimization_path = project_directory + optimization_main_directory + gnss_after_optimization_file;
    std::string gnss_optimization_results_file_path = project_directory + optimization_main_directory + gnss_optimization_results_file;

    std::cout << std::endl;
    std::cout << "\033[32m" << "----------GnssOptimizationParameter parameter:----------" << "\033[0m" << std::endl;
    std::cout << "gnss_optimization_parameter_.lidar_keyframe_distance:" << "\033[33m"  << gnss_optimization_parameter_.lidar_keyframe_distance  << "\033[0m" << std::endl;
    std::cout << "gnss_optimization_parameter_.use_distance_lidar_pose_estimate_init_angle:" << "\033[33m"  << gnss_optimization_parameter_.use_distance_lidar_pose_estimate_init_angle  << "\033[0m" << std::endl;
    std::cout << "gnss_optimization_parameter_.distance_interval_add_prior_gnss_pose:" << "\033[33m"  << gnss_optimization_parameter_.distance_interval_add_prior_gnss_pose  << "\033[0m" << std::endl;
    std::cout << "gnss_optimization_parameter_.max_iterations:" << "\033[33m"  << gnss_optimization_parameter_.max_iterations  << "\033[0m" << std::endl;
    std::cout << "g2o_file_before_optimization_path:" << "\033[33m"  << g2o_file_before_optimization_path  << "\033[0m" << std::endl;
    std::cout << "g2o_file_after_optimization_path:" << "\033[33m"  << g2o_file_after_optimization_path  << "\033[0m" << std::endl;
    std::cout << "gnss_optimization_results_file_path:" << "\033[33m"  << gnss_optimization_results_file_path  << "\033[0m" << std::endl;

    g2o_file_before_optimization_.open(g2o_file_before_optimization_path.c_str());
    g2o_file_after_optimization_.open(g2o_file_after_optimization_path.c_str());
    gnss_optimization_results_filelist_.open(gnss_optimization_results_file_path); 
    gnss_init_pose_file_.open(project_directory + gnss_main_directory + gnss_init_pose_file);
}

bool GnssOptimization::Initialize(std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists,
                                  std::vector<constraints::GnssPoseConstraint> gnss_pose_constraints)
{
    LoadConfig();

    registration_results_filelists_ = registration_results_filelists;
    gnss_pose_constraints_ = gnss_pose_constraints;

    // 如果有gnss位姿消息，进行gnss优化
    if(gnss_pose_constraints_.size() > 0)
    {
        SelectOptimizationFrame();

        AddVertexes();

        AddEdges();

        EstimateInitAngle();

        AlignToGnss();

        return true;
    }
    // // 如果没有gnss位姿消息，直接输出关键帧位姿
    else if(gnss_pose_constraints_.size() == 0)
    {
        OutputNotOptimizationResultsFile();
        return false;
    }


}

// 通过 gnss 走过的距离选取关键帧
void GnssOptimization::SelectOptimizationFrame()
{
    Eigen::Vector3d pose_previous = gnss_pose_constraints_.at(0).prior_position;
    
    for(std::vector<constraints::GnssPoseConstraint>::iterator item = gnss_pose_constraints_.begin();
                                                               item != gnss_pose_constraints_.end();
                                                               item++)
    {
        double delta_x = pose_previous[0] - item->prior_position[0];
        double delta_y = pose_previous[1] - item->prior_position[1];
        double delta_z = pose_previous[2] - item->prior_position[2];

        double distance = sqrt(delta_x*delta_x + delta_y*delta_y);

        // std::cout << "\033[32m" << "distance:" << distance 
        //           << "\033[0m"
        //           << std::endl;

        if(distance > gnss_optimization_parameter_.lidar_keyframe_distance)
        {
            pose_previous = item->prior_position;
            gnss_optimization_pose_constraints_.push_back(*item);
        }       
    }

    std::cout << "\033[32m" << "gnss_optimization_pose_constraints_.size:" << gnss_optimization_pose_constraints_.size()
              << "\033[0m"
              << std::endl;
}



void GnssOptimization::AddVertexes()
{
    for(std::vector<constraints::GnssPoseConstraint>::iterator item_gnss = gnss_optimization_pose_constraints_.begin();
                                                               item_gnss != gnss_optimization_pose_constraints_.end();
                                                               item_gnss++)
    {
        long long lidar_allframe_id = item_gnss->lidar_allframe_id;

        // 遍历所有雷达里程计位姿，找到与gnss约束位姿中 lidar_allframe_id相同的帧
        std::vector<mapping::RegistrationResultsFilelist>::iterator item_lidar = registration_results_filelists_.begin();
        for(;
            item_lidar != registration_results_filelists_.end();
            item_lidar++)
        {
            if(lidar_allframe_id == item_lidar->allframe_id)
            {
                break;
            }
        }

        // 没有遍历到结尾，说明找到了对应的帧
        if(item_lidar != registration_results_filelists_.end())
        {
            if(!is_get_gnss_position_init_)
            {
                is_get_gnss_position_init_ = true;
                gnss_position_init_ = item_gnss->prior_position;
                lidar_position_init_ = Eigen::Vector3f(item_lidar->alignment_result.final_transform(0,3),    // 第1个点必须有GNSS，后面路段平移到原点
                                                       item_lidar->alignment_result.final_transform(1,3),
                                                       item_lidar->alignment_result.final_transform(2,3));

                gnss_init_pose_file_ << std::setprecision(15)
                                     << gnss_position_init_[0] << " "
                                     << gnss_position_init_[1] << " "
                                     << gnss_position_init_[2] 
                                     << std::endl;
            }

            // 添加 gnss 位姿顶点
            AddGnssVertex(item_gnss->prior_position,
                          lidar_allframe_id);
        }                                                                       
    }

    // 添加激光里程计节点
    std::vector<mapping::RegistrationResultsFilelist>::iterator item_lidar = registration_results_filelists_.begin();
    Eigen::Vector3f translation_previous = Eigen::Vector3f(item_lidar->alignment_result.final_transform(0,3),
                                                           item_lidar->alignment_result.final_transform(1,3),
                                                           item_lidar->alignment_result.final_transform(2,3))
                                           - lidar_position_init_;

    for(;
        item_lidar != registration_results_filelists_.end();
        item_lidar++)
    {
        bool is_gnss_constraint_vertex = false;
        for(size_t i = 0; i < gnss_vertices_.size(); ++i)
        {
            if(gnss_vertices_.at(i)->id() == item_lidar->allframe_id)
            {
                is_gnss_constraint_vertex = true;
                break;
            }
        }

        Eigen::Vector3f translation_current = Eigen::Vector3f(item_lidar->alignment_result.final_transform(0,3),
                                                              item_lidar->alignment_result.final_transform(1,3),
                                                              item_lidar->alignment_result.final_transform(2,3))
                                              - lidar_position_init_;

        Eigen::Matrix3f rotation_current = item_lidar->alignment_result.final_transform.block<3,3>(0,0);
        double delta_x = translation_current[0] - translation_previous[0];
        double delta_y = translation_current[1] - translation_previous[1];
        double delta_distance = sqrt(delta_x*delta_x + delta_y*delta_y);

        if((delta_distance > gnss_optimization_parameter_.lidar_keyframe_distance) || (is_gnss_constraint_vertex))
        {
            translation_previous = translation_current;

            // 如果考虑杆臂补偿
            if(lever_arm_parameter_.use_lever_arm)
            {
                Eigen::Matrix<float, 3, 1> lidar_translation;

                lidar_translation(0,0) = item_lidar->alignment_result.final_transform(0,3) - lidar_position_init_[0];
                lidar_translation(1,0) = item_lidar->alignment_result.final_transform(1,3) - lidar_position_init_[1];
                lidar_translation(2,0) = item_lidar->alignment_result.final_transform(2,3) - lidar_position_init_[2];


                Eigen::Isometry3f transform_lidar_to_map = Eigen::Isometry3f::Identity();
                transform_lidar_to_map.rotate(rotation_current);
                transform_lidar_to_map.pretranslate(lidar_translation);

                Eigen::Matrix<float, 3, 1> rtk_antenna_in_lidar(lever_arm_parameter_.lever_arm_x,
                                                                lever_arm_parameter_.lever_arm_y,
                                                                lever_arm_parameter_.lever_arm_z);

                Eigen::Matrix<float, 3, 1> rtk_antenna_in_map = transform_lidar_to_map * rtk_antenna_in_lidar;

                translation_current[0] = rtk_antenna_in_map(0,0);
                translation_current[1] = rtk_antenna_in_map(1,0);
                translation_current[2] = rtk_antenna_in_map(2,0);
            }

            AddLidarOdomVertex(translation_current,
                               rotation_current,
                               item_lidar->allframe_id);

            if(is_gnss_constraint_vertex)
            {
                item_lidar->is_gnss_constraint = true;
            }

            registration_optimization_results_filelists_.push_back(*item_lidar);
        }
    }

    std::cout << "\033[32m" 
              << "lidar_keyframe_vertices_.size:" << lidar_keyframe_vertices_.size()
              << "  gnss_vertices_.size:" << gnss_vertices_.size()
              << "  registration_optimization_results_filelists_.size:" << registration_optimization_results_filelists_.size()
              << "\033[0m"
              << std::endl;
}

// 添加雷达里程计顶点
void GnssOptimization::AddLidarOdomVertex(Eigen::Vector3f translation,
                                          Eigen::Matrix3f rotation, 
                                          long long vertex_id
                                          )
{
    Eigen::Isometry3f transform;

    transform = rotation;

    transform.translation() = translation;  //给欧式变换设置平移变换部分

    Eigen::Isometry3d transform_d = static_cast<Eigen::Isometry3d>(transform);
    //定义顶点类型的变量
    g2o::VertexSE3* lidar_odom_pose_vertex = new g2o::VertexSE3;

    //设置顶点ID
    lidar_odom_pose_vertex->setId(vertex_id);

    //设置顶点位姿，一个顶点是一个机器人pose点，既有3维位置又有3维姿态
    lidar_odom_pose_vertex->setEstimate(transform_d);

    //将边加入边的容器中
    lidar_keyframe_vertices_.push_back(lidar_odom_pose_vertex);
}

// 添加gnss位姿顶点
void GnssOptimization::AddGnssVertex(Eigen::Vector3d translation,
                                     long long vertex_id
                                    )
{

    Eigen::Vector3d gnss_position = translation - gnss_position_init_;

    //定义顶点类型的变量
    g2o::VertexPointXYZ* gnss_vertex = new g2o::VertexPointXYZ;

    //设置顶点ID
    gnss_vertex->setId(vertex_id);

    //设置顶点位姿，一个顶点是一个机器人pose点，既有3维位置又有3维姿态
    gnss_vertex->setEstimate(gnss_position);

    //将边加入边的容器中
    gnss_vertices_.push_back(gnss_vertex);
}


void GnssOptimization::AddEdges()
{
    // 添加雷达里程计约束边
    AddLidarOdomEdges();

    // 添加雷达位姿和gnss位姿约束边
    AddLidarGnssEdges();

}

// 添加雷达里程计约束边
void GnssOptimization::AddLidarOdomEdges()
{
    for(int i=1; i<lidar_keyframe_vertices_.size(); i++)
    {
        g2o::VertexSE3* vertex_previous = lidar_keyframe_vertices_.at(i-1);
        g2o::VertexSE3* vertex_current  = lidar_keyframe_vertices_.at(i);

        // TODO: why?
        Eigen::Isometry3d t = vertex_previous->estimate().inverse() * vertex_current->estimate();

        //边
        g2o::EdgeSE3* edge = new g2o::EdgeSE3;

        //设置顶点ID
        edge->setVertex(0, vertex_previous);
        edge->setVertex(1, vertex_current);

        //设置观测量
        edge->setMeasurement(t);

        //设置信息矩阵
        edge->setInformation(lidar_odom_information_);

        lidar_keyframe_edges_.push_back(edge);
    }
}

// 添加雷达位姿和gnss位姿约束边
void GnssOptimization::AddLidarGnssEdges()
{
    for(int i=0; i<gnss_vertices_.size(); i++)
    {
        for(int j=0; j<lidar_keyframe_vertices_.size(); j++)
        {
            if(gnss_vertices_.at(i)->id() == lidar_keyframe_vertices_.at(j)->id())
            {
                Eigen::Vector3d  gnss_position = gnss_vertices_.at(i)->estimate(); 

                const Eigen::Vector3d& gnss_position_ptr = gnss_position;

                EdgeSE3PriorXYZ* lidar_gnss_edge(new EdgeSE3PriorXYZ());
                lidar_gnss_edge->setMeasurement(gnss_position_ptr);
                lidar_gnss_edge->setInformation(lidar_gnss_information_);

                lidar_gnss_edge->vertices()[0] = lidar_keyframe_vertices_.at(j);

                static bool isFisrtFrame = true;
                if(isFisrtFrame)
                {
                    isFisrtFrame = false;
                    lidar_keyframe_gnss_edges_.push_back(lidar_gnss_edge);

                    continue;
                }

                // //计算当前段的轨迹长度：
                static double sum_current_segment_trajectory_distance = 0;
                current_segment_gnss_pose_arrary_.push_back(gnss_position);

                for(int index_current_segment=1; index_current_segment<current_segment_gnss_pose_arrary_.size(); index_current_segment++)
                {
                    double delta_x = current_segment_gnss_pose_arrary_.at(index_current_segment)[0] 
                                   - current_segment_gnss_pose_arrary_.at(index_current_segment-1)[0];
                    double delta_y = current_segment_gnss_pose_arrary_.at(index_current_segment)[1] 
                                   - current_segment_gnss_pose_arrary_.at(index_current_segment-1)[1];
                    double delta_z = current_segment_gnss_pose_arrary_.at(index_current_segment)[2] 
                                   - current_segment_gnss_pose_arrary_.at(index_current_segment-1)[2];

                    sum_current_segment_trajectory_distance += sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
                }

                //如果当前段的轨迹长度大于80米(可调)，将最早压入队列的元素放到地图队列里面
                if(sum_current_segment_trajectory_distance > gnss_optimization_parameter_.distance_interval_add_prior_gnss_pose)
                {
                    sum_current_segment_trajectory_distance = 0.0;

                    current_segment_gnss_pose_arrary_.clear();
                    lidar_keyframe_gnss_edges_.push_back(lidar_gnss_edge);
                }


            }
        }
    }    
}

void GnssOptimization::EstimateInitAngle()
{
  // N*3
  Eigen::MatrixXd AA;
  Eigen::MatrixXd BB;

  AA.setOnes(lidar_keyframe_vertices_.size(),3);
  BB.setOnes(lidar_keyframe_vertices_.size(),3);

  for(int i=0; i<gnss_vertices_.size(); i++)
  {
    for(int j=0; j<lidar_keyframe_vertices_.size(); j++)
    {
      if(gnss_vertices_.at(i)->id() == lidar_keyframe_vertices_.at(j)->id())
      {
          Eigen::Vector3d  gnss_position = gnss_vertices_.at(i)->estimate();

          AA(i,0) = gnss_position(0);
          AA(i,1) = gnss_position(1);
          AA(i,2) = gnss_position(2);

          Eigen::Isometry3d transform = lidar_keyframe_vertices_.at(j)->estimate();

          Eigen::Vector3d  lidar_position = transform.translation();
          BB(i,0) = lidar_position(0);
          BB(i,1) = lidar_position(1);
          BB(i,2) = lidar_position(2);

      }
    }
  }

  //rotation matrix
  Eigen::MatrixXd H = (AA.transpose())*(BB);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV );
  Eigen::MatrixXd U = svd.matrixU();
  Eigen::MatrixXd Vt = svd.matrixV();
  Eigen::MatrixXd S = svd.singularValues();

  Eigen::MatrixXd R = Vt*U.transpose();

  //special reflection case
  if(R.determinant()< 0)
  {
      for(int i= 0; i< Vt.cols(); i++)
      {
          Vt(i, 2)= (-1) * Vt(i, 2);
      }
      R = Vt* U.transpose();
  }

  initial_rotation_matrix_ = R.block<3,3>(0,0);
}


void GnssOptimization::AlignToGnss()
{
    // 雷达里程计节点
    for(size_t i = 0; i < lidar_keyframe_vertices_.size(); i++)
    {
        Eigen::Isometry3d transform = lidar_keyframe_vertices_.at(i)->estimate();

        transform = initial_rotation_matrix_.inverse() * transform;
        lidar_keyframe_vertices_.at(i)->setEstimate(transform);
    }    
}


void GnssOptimization::OutputG2oFile(std::ofstream& g2o_file)
{
    std::string lidar_vertex_tag = g2o::Factory::instance()->tag(lidar_keyframe_vertices_.at(0)); 
    std::string gnss_vertex_tag = g2o::Factory::instance()->tag(gnss_vertices_.at(0)); 
    std::string edge_tag = g2o::Factory::instance()->tag(lidar_keyframe_edges_.at(0));

    // 雷达里程计节点
    for(size_t i = 0; i < lidar_keyframe_vertices_.size(); i++)
    {
        g2o::VertexSE3* lidar_vertex = lidar_keyframe_vertices_.at(i);

        g2o_file << lidar_vertex_tag << " " << lidar_vertex->id() << " ";

        lidar_vertex->write(g2o_file);

        g2o_file << std::endl;
    }

    // 雷达里程计边
    for(size_t i = 0; i < lidar_keyframe_edges_.size(); ++i)
    {
        g2o::EdgeSE3* edge = lidar_keyframe_edges_.at(i);

        g2o::VertexSE3* from = static_cast<g2o::VertexSE3*>(edge->vertex(0));
        g2o::VertexSE3* to = static_cast<g2o::VertexSE3*>(edge->vertex(1));

        g2o_file << edge_tag << " " << from->id() << " " << to->id() << " ";

        edge->write(g2o_file);

        g2o_file << std::endl;
    }

    // gnss 位置节点
    for(size_t i = 0; i < gnss_vertices_.size(); ++i)
    {
        g2o::VertexPointXYZ* gnss_vertex = gnss_vertices_.at(i);

        g2o_file << gnss_vertex_tag << " " 
                                    << gnss_vertex->id() + lidar_keyframe_vertices_.at(lidar_keyframe_vertices_.size() - 1)->id() << " ";

        gnss_vertex->write(g2o_file);

        g2o_file << std::endl;
    }
}


void GnssOptimization::OutputOptimizationResultsFile()
{
    for(int i=0; i<registration_optimization_results_filelists_.size(); i++)
    {
        mapping::RegistrationResultsFilelist& registration_results_filelist = registration_optimization_results_filelists_.at(i);

        Eigen::Isometry3d optimization_transform = lidar_keyframe_vertices_.at(i)->estimate();
        Eigen::Matrix4f transform;

        if(lever_arm_parameter_.use_lever_arm)
        {
            Eigen::Vector3d translation = optimization_transform.translation();                     //取出平移部分
            Eigen::Quaterniond quat = Eigen::Quaterniond(optimization_transform.linear());          //取出旋转部分

            Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
            Eigen::Isometry3d transform_lidar_to_map = Eigen::Isometry3d::Identity();// 虽然称为3d，实质上是4＊4的矩阵　　齐次坐标

            Eigen::Matrix<double, 3, 1> translation_lidar;
            translation_lidar <<  translation(0), translation(1), translation(2);

            // 为欧式变换设置平移矩阵和旋转矩阵：
            transform_lidar_to_map.rotate(rotation_matrix);       //旋转和平移都是在map坐标系下localizer的位姿
            transform_lidar_to_map.pretranslate(translation_lidar);     //得到的欧式变换是localizer到map的欧式坐标变换矩阵

            Eigen::Matrix<double, 3, 1> rtk_antenna_in_lidar(-1 * lever_arm_parameter_.lever_arm_x,
                                                             -1 * lever_arm_parameter_.lever_arm_y,
                                                             -1 * lever_arm_parameter_.lever_arm_z);

            //将RTK天线的位置转换到map坐标系下：
            Eigen::Matrix<double, 3, 1> rtk_antenna_in_map = transform_lidar_to_map * rtk_antenna_in_lidar;    //将c1坐标系下的点p1变换到world坐标系下

            optimization_transform.translation() = Eigen::Vector3d(rtk_antenna_in_map(0,0), 
                                                                   rtk_antenna_in_map(1,0), 
                                                                   rtk_antenna_in_map(2,0));            
        }


        transform(0,0) = optimization_transform(0,0); transform(0,1) = optimization_transform(0,1); transform(0,2) = optimization_transform(0,2); transform(0,3) = optimization_transform(0,3);
        transform(1,0) = optimization_transform(1,0); transform(1,1) = optimization_transform(1,1); transform(1,2) = optimization_transform(1,2); transform(1,3) = optimization_transform(1,3);
        transform(2,0) = optimization_transform(2,0); transform(2,1) = optimization_transform(2,1); transform(2,2) = optimization_transform(2,2); transform(2,3) = optimization_transform(2,3);
        transform(3,0) = optimization_transform(3,0); transform(3,1) = optimization_transform(3,1); transform(3,2) = optimization_transform(3,2); transform(3,3) = optimization_transform(3,3);

        registration_results_filelist.alignment_result.final_transform = transform;

        gnss_optimization_results_filelist_ << std::setprecision(15)
                                            << registration_results_filelist.allframe_id
                                            << " " << registration_results_filelist.time
                                            << " " << registration_results_filelist.is_gnss_constraint
                                            << " " << registration_results_filelist.pcd_file_path
                                            << " " << registration_results_filelist.alignment_result.is_converged
                                            << " " << registration_results_filelist.alignment_result.fitness_score
                                            << " " << registration_results_filelist.alignment_result.time_duration_ms
                                            << " " << registration_results_filelist.alignment_result.final_num_iteration
                                            << " " << registration_results_filelist.alignment_result.final_transform(0,0) << " " << registration_results_filelist.alignment_result.final_transform(0,1) << " " << registration_results_filelist.alignment_result.final_transform(0,2) << " " << registration_results_filelist.alignment_result.final_transform(0,3)
                                            << " " << registration_results_filelist.alignment_result.final_transform(1,0) << " " << registration_results_filelist.alignment_result.final_transform(1,1) << " " << registration_results_filelist.alignment_result.final_transform(1,2) << " " << registration_results_filelist.alignment_result.final_transform(1,3)
                                            << " " << registration_results_filelist.alignment_result.final_transform(2,0) << " " << registration_results_filelist.alignment_result.final_transform(2,1) << " " << registration_results_filelist.alignment_result.final_transform(2,2) << " " << registration_results_filelist.alignment_result.final_transform(2,3)
                                            << " " << registration_results_filelist.alignment_result.final_transform(3,0) << " " << registration_results_filelist.alignment_result.final_transform(3,1) << " " << registration_results_filelist.alignment_result.final_transform(3,2) << " " << registration_results_filelist.alignment_result.final_transform(3,3)
                                            << std::endl;
    }


}


void GnssOptimization::OutputNotOptimizationResultsFile()
{
    // 添加激光里程计节点
    std::vector<mapping::RegistrationResultsFilelist>::iterator item_lidar = registration_results_filelists_.begin();
    Eigen::Vector3f translation_previous = Eigen::Vector3f(item_lidar->alignment_result.final_transform(0,3),
                                                           item_lidar->alignment_result.final_transform(1,3),
                                                           item_lidar->alignment_result.final_transform(2,3));

    for(;
        item_lidar != registration_results_filelists_.end();
        item_lidar++)
    {
        Eigen::Vector3f translation_current = Eigen::Vector3f(item_lidar->alignment_result.final_transform(0,3),
                                                              item_lidar->alignment_result.final_transform(1,3),
                                                              item_lidar->alignment_result.final_transform(2,3));

        Eigen::Matrix3f rotation_current = item_lidar->alignment_result.final_transform.block<3,3>(0,0);
        double delta_x = translation_current[0] - translation_previous[0];
        double delta_y = translation_current[1] - translation_previous[1];
        double delta_distance = sqrt(delta_x*delta_x + delta_y*delta_y);

        if(delta_distance > gnss_optimization_parameter_.lidar_keyframe_distance)
        {
            translation_previous = translation_current;

            gnss_optimization_results_filelist_ << std::setprecision(15)
                                                << item_lidar->allframe_id
                                                << " " << item_lidar->time
                                                << " " << 0
                                                << " " << item_lidar->pcd_file_path
                                                << " " << item_lidar->alignment_result.is_converged
                                                << " " << item_lidar->alignment_result.fitness_score
                                                << " " << item_lidar->alignment_result.time_duration_ms
                                                << " " << item_lidar->alignment_result.final_num_iteration
                                                << " " << item_lidar->alignment_result.final_transform(0,0) << " " << item_lidar->alignment_result.final_transform(0,1) << " " << item_lidar->alignment_result.final_transform(0,2) << " " << item_lidar->alignment_result.final_transform(0,3)
                                                << " " << item_lidar->alignment_result.final_transform(1,0) << " " << item_lidar->alignment_result.final_transform(1,1) << " " << item_lidar->alignment_result.final_transform(1,2) << " " << item_lidar->alignment_result.final_transform(1,3)
                                                << " " << item_lidar->alignment_result.final_transform(2,0) << " " << item_lidar->alignment_result.final_transform(2,1) << " " << item_lidar->alignment_result.final_transform(2,2) << " " << item_lidar->alignment_result.final_transform(2,3)
                                                << " " << item_lidar->alignment_result.final_transform(3,0) << " " << item_lidar->alignment_result.final_transform(3,1) << " " << item_lidar->alignment_result.final_transform(3,2) << " " << item_lidar->alignment_result.final_transform(3,3)
                                                << std::endl;
        }
    }
}


void GnssOptimization::AddVertexesAndEdgesToOptimizer(g2o::SparseOptimizer& optimizer)
{
    for(int i=0; i<lidar_keyframe_vertices_.size(); i++)
    {
        optimizer.addVertex(lidar_keyframe_vertices_.at(i));
    }

    for(int i=0; i<lidar_keyframe_edges_.size(); i++)
    {
        optimizer.addEdge(lidar_keyframe_edges_.at(i));
    }


    for(int i=0; i<lidar_keyframe_gnss_edges_.size();)
    {
        if(i < lidar_keyframe_gnss_edges_.size())
        {
            optimizer.addEdge(lidar_keyframe_gnss_edges_.at(i));
        }

        i += 1;
    }
}

void GnssOptimization::RunOptimization(std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists,
                                       std::vector<constraints::GnssPoseConstraint> gnss_pose_constraints)
{
    if(Initialize(registration_results_filelists,
                  gnss_pose_constraints))
    {
        OutputG2oFile(g2o_file_before_optimization_);


        g2o::BlockSolverX::LinearSolverType * linearSolver 
            = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();

        // create the block solver on the top of the linear solver
        g2o::BlockSolverX* blockSolver = new g2o::BlockSolverX(linearSolver);

        //create the algorithm to carry out the optimization
        g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm 
            = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

        // create the optimizer
        g2o::SparseOptimizer optimizer;

        AddVertexesAndEdgesToOptimizer(optimizer);

        optimizer.setAlgorithm(optimizationAlgorithm);
        optimizer.setVerbose(true);

        //初始化
        optimizer.initializeOptimization();

        std::cerr << "Optimizing ..." << std::endl;

        //执行优化
        optimizer.optimize(gnss_optimization_parameter_.max_iterations);

        std::cerr << "done." << std::endl;

        //求解器自动输出优化结果的g2o文件
        // optimizer.save("/home/alorli/test_data/opt_after.g2o"); 

        OutputG2oFile(g2o_file_after_optimization_);
        OutputOptimizationResultsFile();
    }
}



}  // namespace optimization


