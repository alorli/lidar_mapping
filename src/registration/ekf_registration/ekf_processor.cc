///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2022_04_25
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>
#include "src/registration/ekf_registration/ekf_processor.h"

#include <tf/transform_datatypes.h>



namespace registration
{

// 内部常用函数
namespace math
{

Eigen::Matrix3d hat(const Eigen::Vector3d& vector3d) 
{
    Eigen::Matrix3d value;

	value << 0.0,           -vector3d[2],  vector3d[1],
		     vector3d[2],   0.0,           -vector3d[0],
		     -vector3d[1],  vector3d[0],   0.0;

	return value;
} 

std::pair<double, double> cos_sinc_sqrt(const double &x2)
{
	using std::sqrt;
	using std::cos;
	using std::sin;
	static double const taylor_0_bound = boost::math::tools::epsilon<double>();
	static double const taylor_2_bound = sqrt(taylor_0_bound);
	static double const taylor_n_bound = sqrt(taylor_2_bound);

	// -----------taylor_0_bound: 2.22045e-16
	// std::cout << "-----------taylor_0_bound: " << taylor_0_bound << std::endl;
	
	// -----------taylor_n_bound: 0.00012207  x2: 7.86132e-17(静止)
	// x2是陀螺测得的角速度的平方和除以 2
	// std::cout << "-----------taylor_n_bound: " << taylor_n_bound 
	// 			  << "  x2: " << x2 
	// 			  << std::endl;

	assert( x2 >= 0 && "argument must be non-negative");
	
	// FIXME check if bigger bounds are possible
	// 当 x2值很小的时候，sin(x)/x的计算会出现 nan 值
	// x2可能出现 0值
	if(x2 >= taylor_n_bound) 
	{
		// slow fall-back solution, 备用算法方案：直接计算 sin(),cos()会非常耗资源，因为imu的计算频率一般比较高，比如 400hz的imu，需要在 2.5毫秒内完成所有运算。
		double x = sqrt(x2);
		return std::make_pair(cos(x), sin(x)/x); // x is greater than 0.
	}
	
	// std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

	// FIXME Replace by Horner-Scheme (4 instead of 5 FLOP/term, numerically more stable, theoretically cos and sinc can be calculated in parallel using SSE2 mulpd/addpd)
	// TODO Find optimal coefficients using Remez algorithm
	// 使用泰勒展开式近似计算 sin和cos的值，可以大幅节省算力
	static double const inv[] = {1/3., 1/4., 1/5., 1/6., 1/7., 1/8., 1/9.};
	double cosi = 1., sinc=1;
	double term = -1/2. * x2;

	for(int i=0; i<3; ++i) 
	{
		cosi += term;
		term *= inv[2*i];
		sinc += term;
		term *= -inv[2*i+1] * x2;
	}

	// std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

	// double x = sqrt(x2);
	// cosi = cos(x);
	// sinc = sin(x)/x;

	// std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
	
	// std::chrono::duration<double> t12
    //         = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

	// std::chrono::duration<double> t23
	// 			= std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2);

	// 计算耗时减少了100多倍
	// -----------t12:  6.8e-05  t23:  0.009714
	// std::cout << "-----------t12:  " << t12.count()*1000.0
	// 		  << "  t23:  " << t23.count()*1000.0
	// 		  << std::endl;


	return std::make_pair(cosi, sinc);
}

Eigen::Quaterniond exp(Eigen::Vector3d vec, 
		               const double& scale = 1) 
{
	double norm2 = vec.squaredNorm();
	
	// 因为 cos_sinc_sqrt()函数内部要开方运算，所以这里是: (1/2 * 1/2)
	std::pair<double, double> cos_sinc = cos_sinc_sqrt(scale * scale * norm2);
	double mult = cos_sinc.second * scale; 
	Eigen::Vector3d xyz = mult * vec;

    return Eigen::Quaterniond(cos_sinc.first, xyz[0], xyz[1], xyz[2]);
}

Eigen::Matrix3d Exp(const Eigen::Vector3d &angle_velocity, const double &delta_time)
{
    double angle_velocity_normal = angle_velocity.norm();

    if(angle_velocity_normal > 0.0000001)
    {
        Eigen::Vector3d rotation_axis = angle_velocity / angle_velocity_normal;
        Eigen::Matrix3d k = hat(rotation_axis);
        double rotation_angle = angle_velocity_normal * delta_time;

        /// Roderigous Tranformation
        return Eigen::Matrix3d::Identity() 
              + std::sin(rotation_angle) * k 
              + (1.0 - std::cos(rotation_angle)) * k * k;
    }
    else
    {
        return Eigen::Matrix3d::Identity();
    }
}


Eigen::Matrix<double, 3, 3> AMatrix(Eigen::Vector3d& v)
{
    Eigen::Matrix<double, 3, 3> res;
    double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	double norm = std::sqrt(squaredNorm);

	if(norm < kDoubleLimit)
	{
		res = Eigen::Matrix<double, 3, 3>::Identity();
	}
	else
	{
		res = Eigen::Matrix<double, 3, 3>::Identity() 
		    + (1 - std::cos(norm)) / squaredNorm * math::hat(v) 
			+ (1 - std::sin(norm) / norm) / squaredNorm * math::hat(v) * math::hat(v);
	}

    return res;
}


struct GravityManifold
{
    GravityManifold(Eigen::Vector3d gravity_in)
    {
        gravity = gravity_in;
        CalculateBasis();
    };

    Eigen::Matrix<double, 3, 2> GetBasis()
    {
        return basis;
    }

    void CalculateBasis()
    {
        if(gravity[0] + kGravityBase > kDoubleLimit)
        {
            
            basis << -gravity[1], -gravity[2],
                     kGravityBase - gravity[1]*gravity[1]/(kGravityBase+gravity[0]), -gravity[2]*gravity[1]/(kGravityBase+gravity[0]),
                    -gravity[2]*gravity[1]/(kGravityBase+gravity[0]), kGravityBase-gravity[2]*gravity[2]/(kGravityBase+gravity[0]);

            basis /= kGravityBase;
        }
        else
        {
            basis = Eigen::Matrix<double, 3, 2>::Zero();
            basis(1, 1) = -1;
            basis(2, 0) = 1;
        }        
    }

    Eigen::Matrix<double, 3, 2> CalculateMx(Eigen::Vector2d delta)
    {
        Eigen::Matrix<double, 3, 2> value;

		if(delta.norm() < kDoubleLimit)
		{
			value = -math::hat(gravity)*basis;
		}
		else
		{
			Eigen::Vector3d bu = basis*delta;
            Eigen::Quaterniond delta_rotation = math::exp(bu, 0.5);
            value = -delta_rotation.toRotationMatrix()*math::hat(gravity)*math::AMatrix(bu).transpose()*basis;

		}

        return value;
    }
        
    Eigen::Matrix<double, 2, 3> CalculateNx()
    {
        return 1.0/kGravityBase/kGravityBase*basis.transpose()*math::hat(gravity);
    }

    Eigen::Vector3d gravity;
    Eigen::Matrix<double, 3, 2> basis;
};


Eigen::Vector3d BoxMinusSO3(Eigen::Quaterniond& rotation1,
                            Eigen::Quaterniond& rotation2
                            )
{
    Eigen::Quaterniond delta_rotation = rotation2.conjugate() * rotation1;

    Eigen::Vector3d vector_part = Eigen::Vector3d(delta_rotation.x(), delta_rotation.y(), delta_rotation.z());

    double normal = vector_part.norm();
    if(normal < kDoubleLimit) 
	{
		normal = kDoubleLimit;
	}

    double scale = 2.0 / normal * (std::atan(normal / delta_rotation.w()));

    return scale*vector_part;
}


Eigen::Vector2d BoxMinusS2(Eigen::Vector3d& gravity1,
                           Eigen::Vector3d& gravity2
                           )
{
    Eigen::Vector2d result;
    double v_sin = (hat(gravity1) * gravity2).norm();
    double v_cos = gravity1.transpose() * gravity2;
    double theta = std::atan2(v_sin, v_cos);

    if(v_sin < kDoubleLimit)
    {
        if(std::fabs(theta) > kDoubleLimit)
        {
            result[0] = 3.1415926;
            result[1] = 0;
        }
        else
        {
            result[0] = 0;
            result[1] = 0;
        }
    }
    else
    {
        Eigen::Vector3d gravity2_copy = gravity2;
        math::GravityManifold gravity2_copy_manifold(gravity2_copy);

        Eigen::Matrix<double, 3, 2> Bx = gravity2_copy_manifold.GetBasis();

        result = theta/v_sin * Bx.transpose() * hat(gravity2)*gravity1;
    }

    return result;
}

Eigen::Vector3d BoxPlusS2(Eigen::Vector3d& gravity,
                          Eigen::Vector2d& delta
                          )
{
    math::GravityManifold gravity_manifold(gravity);
    Eigen::Matrix<double, 3, 2> basis = gravity_manifold.GetBasis();
    Eigen::Matrix<double, 3, 1> basis_u = basis * delta;
    Eigen::Quaterniond delta_rotation = math::exp(basis_u, 0.5);
    Eigen::Vector3d result = delta_rotation.toRotationMatrix() * gravity;

    return result;
}

}


EkfProcessor::EkfProcessor(std::string cfg_file_path)
    :state_(),
     process_covariance_(Eigen::Matrix<double, 23, 23>::Identity())
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");
    
    ekf_parameter_.max_iteration = cfg_file_["ekf_registration"]["ekf_parameter"]["max_iteration"].as<int>();
    ekf_parameter_.converge_threshold = cfg_file_["ekf_registration"]["ekf_parameter"]["converge_threshold"].as<double>();
    ekf_parameter_.converge_limit = cfg_file_["ekf_registration"]["ekf_parameter"]["converge_limit"].as<double>();
    ekf_parameter_.extrinsic_estimate_enable = cfg_file_["ekf_registration"]["ekf_parameter"]["extrinsic_estimate_enable"].as<bool>();
    ekf_parameter_.omp_num_threads_calculate_measurement = cfg_file_["ekf_registration"]["ekf_parameter"]["omp_num_threads_calculate_measurement"].as<int>();
    

    std::fill(converge_limit, converge_limit+23, 0.001);

    laser_cloud_origin_.reset(new pcl::PointCloud<LidarPointType>(100000, 1));
    corresponding_normal_vector_.reset(new pcl::PointCloud<LidarPointType>(100000, 1));
    normal_vector_.reset(new pcl::PointCloud<LidarPointType>(100000, 1));

    
    memset(point_selected_surf_, true, sizeof(point_selected_surf_));
    memset(residual_last_, -1000.0f, sizeof(residual_last_));


    std::cout << std::endl;
    std::cout << "\033[32m" << "----------EkfProcessor parameter:----------" << "\033[0m" << std::endl;
    std::cout << "ekf_parameter_.max_iteration:" << "\033[33m"  << ekf_parameter_.max_iteration  << "\033[0m" << std::endl;
    std::cout << "ekf_parameter_.converge_threshold:" << "\033[33m"  << ekf_parameter_.converge_threshold  << "\033[0m" << std::endl;
    std::cout << "ekf_parameter_.converge_limit:" << "\033[33m"  << ekf_parameter_.converge_limit  << "\033[0m" << std::endl;
    std::cout << "ekf_parameter_.extrinsic_estimate_enable:" << "\033[33m"  << ekf_parameter_.extrinsic_estimate_enable  << "\033[0m" << std::endl;
    std::cout << "ekf_parameter_.omp_num_threads_calculate_measurement:" << "\033[33m"  << ekf_parameter_.omp_num_threads_calculate_measurement  << "\033[0m" << std::endl;
}

EkfProcessor::~EkfProcessor()
{
}

void EkfProcessor::SetEkfState(EkfState ekf_state)
{
    state_ = ekf_state;
}

EkfState EkfProcessor::GetEkfState()
{
    return state_;
}

Eigen::Matrix<double, 23, 23> EkfProcessor::GetProcessCovariance()
{
    return process_covariance_;
}

void EkfProcessor::SetProcessCovariance(Eigen::Matrix<double, 23, 23> process_covariance)
{
    process_covariance_ = process_covariance;
}

void EkfProcessor::PredictState(double& delta_time, 
                                Eigen::Matrix<double, 12, 12>& Q, 
                                InputState& input_state)
{
    // 计算 f
    Eigen::Matrix<double, 24, 1> f = Eigen::Matrix<double, 24, 1>::Zero();
    f.block<3,1>(3,0) = input_state.gyro - state_.bias_gyro;
    f.block<3,1>(0,0) = state_.velocity;
    f.block<3,1>(12,0) = state_.rotation*(input_state.acc - state_.bias_acc) + state_.gravity;

    // static bool print_1st = true;
    bool print_1st = true;
    if(print_1st)
    {
        // std::cout << "----f:" << f.transpose() << std::endl;
    }
    

    // 计算 df_dx
    Eigen::Matrix<double, 24, 23> df_dx = Eigen::Matrix<double, 24, 23>::Zero();
    df_dx.block<3,3>(3,15) = -Eigen::Matrix3d::Identity();
    df_dx.block<3,3>(0,12) = Eigen::Matrix3d::Identity();
    df_dx.block<3,3>(12,3) = -1*state_.rotation.toRotationMatrix()*math::hat(input_state.acc - state_.bias_acc);
    df_dx.block<3,3>(12,18) = -1*state_.rotation.toRotationMatrix();

    math::GravityManifold gravity_manifold_old(state_.gravity);
    Eigen::Matrix<double, 3, 2> mx_old = gravity_manifold_old.CalculateMx(Eigen::Vector2d::Zero());
    df_dx.block<3,2>(12,21) = mx_old;

    if(print_1st)
    {
        // std::cout << "----df_dx:" << std::endl << df_dx << std::endl;
    }

    // 计算 df_dw
    Eigen::Matrix<double, 24, 12> df_dw = Eigen::Matrix<double, 24, 12>::Zero();
    df_dw.block<3,3>(3,3) = -Eigen::Matrix3d::Identity();
    df_dw.block<3,3>(12,0) = -1*state_.rotation.toRotationMatrix();
    df_dw.block<3,3>(18,6) = Eigen::Matrix3d::Identity();
    df_dw.block<3,3>(15,9) = Eigen::Matrix3d::Identity();

    if(print_1st)
    {
        // std::cout << "----df_dw:" << std::endl << df_dw << std::endl;
    }


    // 用 f 预测状态
    EkfState state_old = state_;
    state_.rotation = state_.rotation*math::exp(f.block<3,1>(3,0), delta_time/2.0);
    state_.position += f.block<3,1>(0,0) * delta_time;
    state_.velocity += f.block<3,1>(12,0) * delta_time;
    state_.bias_acc += f.block<3,1>(18,0) * delta_time;
    state_.bias_gyro += f.block<3,1>(15,0) * delta_time;
    state_.gravity = math::exp(f.block<3,1>(21,0), delta_time/2.0)*state_.gravity;
    state_.extrinsic_rotation = state_.extrinsic_rotation*math::exp(f.block<3,1>(6,0), delta_time/2.0);
    state_.extrinsic_translation += f.block<3,1>(9,0) * delta_time;
    
    // 计算 F_x 和 F_w
    Eigen::Matrix<double, 23, 23> F_x = Eigen::Matrix<double, 23, 23>::Identity();
    Eigen::Matrix<double, 23, 23> f_x = Eigen::Matrix<double, 23, 23>::Zero();
    Eigen::Matrix<double, 23, 12> f_w = Eigen::Matrix<double, 23, 12>::Zero();

    // 计算 F_x 和 F_w  ----- state: rotation
    Eigen::Vector3d rotation_vector = -1.0*f.block<3,1>(3,0)*delta_time;

    if(print_1st)
    {
        // std::cout << "----delta_time:" << delta_time << std::endl;
    }

    Eigen::Quaterniond rotation = math::exp(rotation_vector, 0.5);
    F_x.block<3,3>(3,3) = rotation.toRotationMatrix();

    Eigen::Matrix<double, 3, 3> amatrix_rotation = math::AMatrix(rotation_vector);
    for(int j=0; j<23; j++)
    {
        f_x.block<3,1>(3,j) = amatrix_rotation * df_dx.block<3,1>(3,j);
    }

    for(int j=0; j<12; j++)
    {
        f_w.block<3,1>(3,j) = amatrix_rotation * df_dw.block<3,1>(3,j);
    }
    
    

    if(print_1st)
    {
        // std::cout << "----delta_time:" << std::endl << delta_time << std::endl;
        // std::cout << "----rotation_vector:" << std::endl << rotation_vector << std::endl;
        // std::cout << "----rotation:x" << rotation.x() << " y:" << rotation.y()<< " z:" << rotation.z()<< " w:" << rotation.w() << std::endl;
        // std::cout << "----amatrix_rotation:" << std::endl << amatrix_rotation << std::endl;
    }

    // 计算 F_x 和 F_w  ----- state: position
    f_x.block<3,23>(0,0) = df_dx.block<3,23>(0,0);
    f_w.block<3,12>(0,0) = df_dw.block<3,12>(0,0);    

    // 计算 F_x 和 F_w  ----- state: velocity
    f_x.block<3,23>(12,0) = df_dx.block<3,23>(12,0);
    f_w.block<3,12>(12,0) = df_dw.block<3,12>(12,0);     

    // 计算 F_x 和 F_w  ----- state: bias_acc
    f_x.block<3,23>(18,0) = df_dx.block<3,23>(18,0);
    f_w.block<3,12>(18,0) = df_dw.block<3,12>(18,0);  

    // 计算 F_x 和 F_w  ----- state: bias_gyro
    f_x.block<3,23>(15,0) = df_dx.block<3,23>(15,0);
    f_w.block<3,12>(15,0) = df_dw.block<3,12>(15,0);  

    // 计算 F_x 和 F_w  ----- state: gravity
    Eigen::Vector3d gravity_vector = f.block<3,1>(21,0)*delta_time;
    Eigen::Matrix3d delta_gravity_matrix = math::exp(gravity_vector, 0.5).toRotationMatrix();

    math::GravityManifold gravity_manifold_new(state_.gravity);
    Eigen::Matrix<double, 2, 3> nx = gravity_manifold_new.CalculateNx();
    Eigen::Matrix<double, 3, 2> mx = mx_old;
    F_x.block<2,2>(21,21) = nx * delta_gravity_matrix * mx;

    Eigen::Matrix<double, 2, 3> temp_matrix_1 = -nx * delta_gravity_matrix * math::hat(state_old.gravity) * math::AMatrix(gravity_vector).transpose();

    for(int j=0; j<23; j++)
    {
        f_x.block<2,1>(21,j) = temp_matrix_1 * df_dx.block<3,1>(21,j);
    }
    
    for(int j=0; j<12; j++)
    {
        f_w.block<2,1>(21,j) = temp_matrix_1 * df_dw.block<3,1>(21,j); 

    }
    
    // 计算 F_x 和 F_w  ----- state: extrinsic_rotation
    Eigen::Vector3d extrinsic_rotation_vector = -1.0*f.block<3,1>(6,0)*delta_time;
    Eigen::Quaterniond extrinsic_rotation = math::exp(extrinsic_rotation_vector, 0.5);
    F_x.block<3,3>(6,6) = extrinsic_rotation.toRotationMatrix();

    Eigen::Matrix<double, 3, 3> amatrix_extrinsic_rotation = math::AMatrix(extrinsic_rotation_vector);
    for(int j=0; j<23; j++)
    {
        f_x.block<3,1>(6,j) = amatrix_extrinsic_rotation * df_dx.block<3,1>(6,j);
    }
    
    for(int j=0; j<12; j++)
    {
        f_w.block<3,1>(6,j) = amatrix_extrinsic_rotation * df_dw.block<3,1>(6,j);
    }
    

    // 计算 F_x 和 F_w  ----- state: extrinsic_translation
    f_x.block<3,23>(9,0) = df_dx.block<3,23>(9,0);
    f_w.block<3,12>(9,0) = df_dw.block<3,12>(9,0);  

    if(print_1st)
    {
        // std::cout << "****f_x:" << std::endl << f_x << std::endl;
        // std::cout << "****f_w:" << std::endl << f_w << std::endl;
        // std::cout << "****F_x:" << std::endl << F_x << std::endl;
        // std::cout << "****Q:" << std::endl << Q << std::endl;
        // std::cout << "****process_covariance_:" << std::endl << process_covariance_ << std::endl;
    }

    // 更新一步预测误差方差阵
    F_x += f_x * delta_time;
    process_covariance_ = F_x*process_covariance_*F_x.transpose() + (f_w*delta_time) * Q * (f_w*delta_time).transpose();


    if(print_1st)
    {
        // std::cout << "----F_x:" << std::endl << F_x << std::endl;
        // std::cout << "----process_covariance_:" << std::endl << process_covariance_ << std::endl;
    }

    print_1st = false;
}


void EkfProcessor::UpdateState(ikd_tree::KD_TREE& global_ikd_tree,
                               std::vector<PointVector>& nearest_points_vector,
                               TimedIdLidarPointCloud& compensationed_features_pointcloud_downsize_lidar,
                               TimedIdLidarPointCloud& compensationed_features_pointcloud_downsize_world)
{
    measurement_data_.is_valid = true;
    measurement_data_.is_converge = true;

    int t = 0;
    EkfState state_propagated = state_;
    Eigen::Matrix<double, 23, 23> process_covariance_propagated = process_covariance_;
    int dof_measurement;   

    Eigen::Matrix<double, 23, 1> K_h;
    Eigen::Matrix<double, 23, 23> K_x;   

    // 流形变量的差值少一维，因为重力向量S2的差值是2维
    Eigen::Matrix<double, 23, 1> dx_new = Eigen::Matrix<double, 23, 1>::Zero();

    normal_vector_->resize(compensationed_features_pointcloud_downsize_lidar.pointcloud_ptr->size());

    for(int i=-1; i<ekf_parameter_.max_iteration; i++)
    {
        measurement_data_.is_valid = true;

        CalculateMeasurement(global_ikd_tree,
                             nearest_points_vector,
                             compensationed_features_pointcloud_downsize_lidar,
                             compensationed_features_pointcloud_downsize_world
                            );

        // std::cout << "------------measurement_data_.h_x first_line:" << std::endl
        //           << "  " << measurement_data_.h_x(0,0) 
        //           << "  " << measurement_data_.h_x(0,1) 
        //           << "  " << measurement_data_.h_x(0,2) 
        //           << std::endl;
        
        // std::cout << "------------measurement_data_.h_x last_line:" << std::endl
        //           << "  " << measurement_data_.h_x(measurement_data_.h_x.rows()-1,0) 
        //           << "  " << measurement_data_.h_x(measurement_data_.h_x.rows()-1,1) 
        //           << "  " << measurement_data_.h_x(measurement_data_.h_x.rows()-1,2) 
        //           << std::endl << std::endl;


        if(!measurement_data_.is_valid)
        {
            continue; 
        }
        
        Eigen::Matrix<double, Eigen::Dynamic, 12> h_x_ = measurement_data_.h_x;
        dof_measurement = h_x_.rows();

        // dx: 上一次状态变量估计值与递推值之间的差值
        Eigen::Matrix<double, 23, 1> dx = CalculateStateBoxMinus(state_propagated);
        dx_new = dx;

        // std::cout << "\033[32m" << "------------i:" << i << "  dof_measurement:" << dof_measurement << "\033[0m" << std::endl;
        // std::cout << "\033[32m" << "------------i:" << i << "  dx:" << "\033[0m" << std::endl;
        // for(int j=0; j<23; j++)
        // {
        //     std::cout << "  " << dx(j, 0);
        // }
        // std::cout << std::endl;


        process_covariance_ = process_covariance_propagated;

        // std::cout << "\033[32m" << "------------i:" << i << "  process_covariance_propagated:" << "\033[0m" << std::endl;
        // std::cout << "  " << process_covariance_propagated << std::endl;

        // 根据dx更新状态
        // 更新状态SO3----rotation:3
        Eigen::Vector3d rotation_vector1 = dx.block<3,1>(3,0);
        Eigen::Matrix<double, 3, 3> amatrix_rotation1 = math::AMatrix(rotation_vector1).transpose();
        dx_new.block<3,1>(3,0) = amatrix_rotation1 * dx_new.block<3,1>(3,0);

        for(int j = 0; j < 23; j++)
        {
            process_covariance_.block<3,1>(3,j) = amatrix_rotation1 * process_covariance_.block<3,1>(3,j);
        }
        

        // std::cout << "\033[32m" << "------------i:" << i << "  amatrix_rotation1:" << "\033[0m" << std::endl;
        // std::cout << "  " << amatrix_rotation1 << std::endl;

        // std::cout << "\033[32m" << "------------i:" << i << "  process_covariance_1:" << "\033[0m" << std::endl;
        // std::cout << "  " << process_covariance_ << std::endl;

        // process_covariance_.block<23,3>(0,3) = process_covariance_.block<23,3>(0,3) * amatrix_rotation1.transpose();
        for(int j = 0; j < 23; j++)
        {
            process_covariance_.block<1,3>(j,3) = process_covariance_.block<1,3>(j,3) * amatrix_rotation1.transpose();
        }

        // std::cout << "\033[32m" << "------------i:" << i << "  amatrix_rotation1.transpose():" << "\033[0m" << std::endl;
        // std::cout << "  " << amatrix_rotation1.transpose() << std::endl;

        // std::cout << "\033[32m" << "------------i:" << i << "  process_covariance_2:" << "\033[0m" << std::endl;
        // std::cout << "  " << process_covariance_ << std::endl;

        // 更新状态SO3----extrinsic_rotation:6
        Eigen::Vector3d rotation_vector2 = dx.block<3,1>(6,0);
        Eigen::Matrix<double, 3, 3> amatrix_rotation2 = math::AMatrix(rotation_vector2).transpose();
        dx_new.block<3,1>(6,0) = amatrix_rotation2 * dx_new.block<3,1>(6,0);
        for(int j = 0; j < 23; j++)
        {
            process_covariance_.block<3,1>(6,j) = amatrix_rotation2 * process_covariance_.block<3,1>(6,j);
        }

        
        // process_covariance_.block<23,3>(0,6) = process_covariance_.block<23,3>(0,6) * amatrix_rotation2.transpose();

        for(int j = 0; j < 23; j++)
        {
            process_covariance_.block<1,3>(j,6) = process_covariance_.block<1,3>(j,6) * amatrix_rotation2.transpose();
        }
        // std::cout << "\033[32m" << "------------i:" << i << "  rotation_vector1:" << "\033[0m" << std::endl;
        // std::cout << "  " << rotation_vector1 << std::endl;

        // std::cout << "\033[32m" << "------------i:" << i << "  rotation_vector2:" << "\033[0m" << std::endl;
        // std::cout << "  " << rotation_vector2 << std::endl;



        // 更新状态S2----gravity:21
        Eigen::Vector2d delta_gravity = dx.block<2,1>(21,0);
        math::GravityManifold gravity_manifold(state_.gravity);
        Eigen::Matrix<double, 2, 3> nx = gravity_manifold.CalculateNx();
        math::GravityManifold gravity_manifold_propagated(state_propagated.gravity);
        Eigen::Matrix<double, 3, 2> mx_propagated = gravity_manifold_propagated.CalculateMx(delta_gravity);
        Eigen::Matrix<double, 2, 2> nx_mx = nx * mx_propagated;
        dx_new.block<2,1>(21,0) = nx_mx * dx_new.block<2,1>(21,0);
        for(int j = 0; j < 23; j++)
        {
            process_covariance_.block<2,1>(21,j) = nx_mx * process_covariance_.block<2,1>(21,j);
        }

        for(int j = 0; j < 23; j++)
        {
            process_covariance_.block<1,2>(j,21) = process_covariance_.block<1,2>(j,21) * nx_mx.transpose();
        }

        // std::cout << "\033[32m" << "------------i:" << i << "  dx_new:" << "\033[0m" << std::endl;
        // for(int j=0; j<23; j++)
        // {
        //     std::cout << "  " << dx_new(j, 0);
        // }
        // std::cout << std::endl;


        if(dof_measurement < 23)
        {
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_x_cur = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(dof_measurement, 23);
            h_x_cur.topLeftCorner(dof_measurement, 12) = h_x_;
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K_ = process_covariance_ * h_x_cur.transpose() 
                                                                    * (h_x_cur * process_covariance_ * h_x_cur.transpose()/ kLaserPointcovariance 
                                                                       + Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(dof_measurement, dof_measurement)).inverse()/kLaserPointcovariance;
            K_h = K_ * measurement_data_.h;
            K_x = K_ * h_x_cur;
        }
        else
        {
            Eigen::Matrix<double, 23, 23> process_covariance_temp = (process_covariance_/kLaserPointcovariance).inverse();
            Eigen::Matrix<double, 12, 12> HTH = h_x_.transpose() * h_x_; 
            process_covariance_temp.block<12, 12>(0, 0) += HTH;

            // std::cout << "\033[32m" << "------------i:" << i << "  HTH:" << "\033[0m" << std::endl;
            // std::cout << "  " << HTH << std::endl;

            // std::cout << "\033[32m" << "------------i:" << i << "  process_covariance_temp:" << "\033[0m" << std::endl;
            // std::cout << "  " << process_covariance_temp << std::endl;

            Eigen::Matrix<double, 23, 23> process_covariance_temp_inverse = process_covariance_temp.inverse();

            // std::cout << "\033[32m" << "------------i:" << i << "  process_covariance_temp_inverse:" << "\033[0m" << std::endl;
            // std::cout << "  " << process_covariance_temp_inverse << std::endl;


            K_h = process_covariance_temp_inverse.block<23, 12>(0, 0) * h_x_.transpose() * measurement_data_.h;
            K_x.setZero(); 
            K_x.block<23, 12>(0, 0) = process_covariance_temp_inverse.block<23, 12>(0, 0) * HTH;
        }

        Eigen::Matrix<double, 23, 1> dx_ = K_h + (K_x - Eigen::Matrix<double, 23, 23>::Identity()) * dx_new;


        // std::cout << "\033[32m" << "------------i:" << i << "  process_covariance_:" << "\033[0m" << std::endl;
        // std::cout << "  " << process_covariance_ << std::endl;

        // std::cout << "\033[32m" << "------------i:" << i << "  h_x_:" << "\033[0m" << std::endl;
        // for(int j=0; j<12; j++)
        // {
        //     std::cout << "  " << h_x_(0, j);
        // }
        // std::cout << "  " << h_x_(dof_measurement-3, 0);
        // std::cout << "  " << h_x_(dof_measurement-2, 0);
        // std::cout << "  " << h_x_(dof_measurement-1, 0);
        // std::cout << std::endl;

        // std::cout << "\033[32m" << "------------i:" << i << "  h:" << "\033[0m" << std::endl;
        // for(int j=0; j<5; j++)
        // {
        //     std::cout << "  " << measurement_data_.h(j, 0);
        // }
        // std::cout << "  " << measurement_data_.h(dof_measurement-5, 0);
        // std::cout << "  " << measurement_data_.h(dof_measurement-4, 0);
        // std::cout << "  " << measurement_data_.h(dof_measurement-3, 0);
        // std::cout << "  " << measurement_data_.h(dof_measurement-2, 0);
        // std::cout << "  " << measurement_data_.h(dof_measurement-1, 0);
        // std::cout << std::endl;


        // std::cout << "\033[32m" << "------------i:" << i << "  K_h:" << "\033[0m" << std::endl;
        // for(int j=0; j<23; j++)
        // {
        //     std::cout << "  " << K_h(j, 0);
        // }
        // std::cout << std::endl;


        // std::cout << "\033[32m" << "------------i:" << i << "  dx_:" << "\033[0m" << std::endl;
        // for(int j=0; j<23; j++)
        // {
        //     std::cout << "  " << dx_(j, 0);
        // }
        // std::cout << std::endl;

        // 使用dx_更新状态SO3----rotation:3
        state_.rotation = state_.rotation * math::exp(dx_.block<3,1>(3,0), 0.5);

        // 使用dx_更新状态VECTOR3----position:0
        state_.position += dx_.block<3,1>(0,0);

        // 使用dx_更新状态VECTOR3----velocity:12
        state_.velocity += dx_.block<3,1>(12,0);

        // 使用dx_更新状态VECTOR3----bias_acc:18
        state_.bias_acc += dx_.block<3,1>(18,0);

        // 使用dx_更新状态VECTOR3----bias_gyro:15
        state_.bias_gyro += dx_.block<3,1>(15,0);

        // 使用dx_更新状态S2----gravity:21
        Eigen::Vector2d delta_gravity1 = dx_.block<2,1>(21,0);
        state_.gravity = math::BoxPlusS2(state_.gravity, delta_gravity1);

        // 使用dx_更新状态SO3----extrinsic_rotation:6
        state_.extrinsic_rotation = state_.extrinsic_rotation * math::exp(dx_.block<3,1>(6,0), 0.5);

        // 使用dx_更新状态VECTOR3----extrinsic_translation:9
        state_.extrinsic_translation += dx_.block<3,1>(9,0);


        measurement_data_.is_converge = true;
        for(int i = 0; i < 23 ; i++)
        {
            if(std::fabs(dx_[i]) > converge_limit[i])
            {
                measurement_data_.is_converge = false;
                break;
            }
        }

        if(measurement_data_.is_converge) 
        {
            t++;
        }

        if(!t && i == ekf_parameter_.max_iteration - 2)
        {
            measurement_data_.is_converge = true;
        }

        if(t > 1 || i == ekf_parameter_.max_iteration - 1)
        {
            L_ = process_covariance_;

            // 更新 L 和 process_covariance-----SO3: rotation:3
            Eigen::Vector3d rotation_vector = dx_.block<3,1>(3,0);
            Eigen::Matrix3d amatrix_rotation = math::AMatrix(rotation_vector).transpose();
            for(int j = 0; j < 23; j++)
            {
                L_.block<3,1>(3,j) = amatrix_rotation * process_covariance_.block<3,1>(3,j);
            }
            
            for(int j = 0; j < 12; j++)
            {
                K_x.block<3,1>(3,j) = amatrix_rotation * K_x.block<3,1>(3,j);
            }

            for(int j = 0; j < 23; j++)
            {
                L_.block<1,3>(j,3) = L_.block<1,3>(j,3) * amatrix_rotation.transpose();
                process_covariance_.block<1,3>(j,3) = process_covariance_.block<1,3>(j,3) * amatrix_rotation.transpose();
            }

            // 更新 L 和 process_covariance-----SO3: extrinsic_rotation:6
            Eigen::Vector3d extrinsic_rotation_vector = dx_.block<3,1>(6,0);
            Eigen::Matrix3d amatrix_extrinsic_rotation = math::AMatrix(extrinsic_rotation_vector).transpose();
            for(int j = 0; j < 23; j++)
            {
                L_.block<3,1>(6,j) = amatrix_extrinsic_rotation * process_covariance_.block<3,1>(6,j);
            }
            
            for(int j = 0; j < 12; j++)
            {
                K_x.block<3,1>(6,j) = amatrix_extrinsic_rotation * K_x.block<3,1>(6,j);
            }

            for(int j = 0; j < 23; j++)
            {
                L_.block<1,3>(j,6) = L_.block<1,3>(j,6) * amatrix_extrinsic_rotation.transpose();
                process_covariance_.block<1,3>(j,6) = process_covariance_.block<1,3>(j,6) * amatrix_extrinsic_rotation.transpose();
            }

            // 更新 L 和 process_covariance-----S2: gravity:21
            Eigen::Vector2d delta_gravity = dx_.block<2,1>(21,0);
            math::GravityManifold gravity_manifold(state_.gravity);
            Eigen::Matrix<double, 2, 3> nx = gravity_manifold.CalculateNx();

            math::GravityManifold gravity_manifold_propagated(state_propagated.gravity);
            Eigen::Matrix<double, 3, 2> mx_propagated = gravity_manifold_propagated.CalculateMx(delta_gravity);
            Eigen::Matrix<double, 2, 2> nx_mx_propagated = nx * mx_propagated;

            for(int j = 0; j < 23; j++)
            {
                L_.block<2,1>(21,j) = nx_mx_propagated * process_covariance_.block<2,1>(21,j);
            }
            
            for(int j = 0; j < 12; j++)
            {
                K_x.block<2,1>(21,j) = nx_mx_propagated * K_x.block<2,1>(21,j);
            }

            for(int j = 0; j < 23; j++)
            {
                L_.block<1,2>(j,21) = L_.block<1,2>(j,21) * nx_mx_propagated.transpose();
                process_covariance_.block<1,2>(j,21) = process_covariance_.block<1,2>(j,21) * nx_mx_propagated.transpose();
            }

            process_covariance_ = L_ - K_x.block<23,12>(0,0) * process_covariance_.block<12,23>(0,0);

            return;
        }

    }
}


void EkfProcessor::CalculateMeasurement(ikd_tree::KD_TREE& global_ikd_tree,
                                        std::vector<PointVector>& nearest_points_vector,
                                        TimedIdLidarPointCloud& compensationed_features_pointcloud_downsize_lidar,
                                        TimedIdLidarPointCloud& compensationed_features_pointcloud_downsize_world)
{
    laser_cloud_origin_->clear(); 
    corresponding_normal_vector_->clear(); 
    double total_residual = 0.0; 
    int num_features_points_downsize = compensationed_features_pointcloud_downsize_lidar.pointcloud_ptr->points.size();

    omp_set_num_threads(ekf_parameter_.omp_num_threads_calculate_measurement);
    #pragma omp parallel for

    // 遍历每一个降采样点
    for(int i = 0; i < num_features_points_downsize; i++)
    {
        LidarPointType &point_lidar_reference  = compensationed_features_pointcloud_downsize_lidar.pointcloud_ptr->points[i]; 
        LidarPointType &point_world_reference = compensationed_features_pointcloud_downsize_world.pointcloud_ptr->points[i]; 

        /* transform to world frame */
        Eigen::Vector3d point_lidar(point_lidar_reference.x, point_lidar_reference.y, point_lidar_reference.z);
        Eigen::Vector3d point_world(state_.rotation * (state_.extrinsic_rotation*point_lidar + state_.extrinsic_translation) + state_.position);
        point_world_reference.x = point_world(0);
        point_world_reference.y = point_world(1);
        point_world_reference.z = point_world(2);
        point_world_reference.intensity = point_lidar_reference.intensity;

        std::vector<float> point_search_square_distance(KNumPointsEstimatePlane);

        auto &points_near = nearest_points_vector[i];

        if(measurement_data_.is_converge)
        {
            /** Find the closest surfaces in the map **/
            global_ikd_tree.Nearest_Search(point_world_reference, 
                                           KNumPointsEstimatePlane, 
                                           points_near, 
                                           point_search_square_distance);

            // 判断 point_world点附近点的特征是否可用于拟合平面：1. 点的个数是否够5个，2. 距离point_world最远的点的距离是否大于5米
            // point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
            point_selected_surf_[i] = true;
            if(points_near.size() < KNumPointsEstimatePlane 
            || point_search_square_distance[KNumPointsEstimatePlane - 1] > 5)
            {
                point_selected_surf_[i] = false;
            }

        }

        if(!point_selected_surf_[i]) 
        {
            continue;
        }

        Eigen::Matrix<float, 4, 1> pabcd;
        point_selected_surf_[i] = false;
        if(math::EstimatePlane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world_reference.x + pabcd(1) * point_world_reference.y + pabcd(2) * point_world_reference.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(point_lidar.norm());

            if(s > 0.9)
            {
                point_selected_surf_[i] = true;
                normal_vector_->points[i].x = pabcd(0);
                normal_vector_->points[i].y = pabcd(1);
                normal_vector_->points[i].z = pabcd(2);
                normal_vector_->points[i].intensity = pd2;
                residual_last_[i] = abs(pd2);
            }
        }
    }


    effective_features_num_ = 0;
    for(int i = 0; i < num_features_points_downsize; i++)
    {
        if(point_selected_surf_[i])
        {
            laser_cloud_origin_->points[effective_features_num_] = compensationed_features_pointcloud_downsize_lidar.pointcloud_ptr->points[i];
            corresponding_normal_vector_->points[effective_features_num_] = normal_vector_->points[i];
            total_residual += residual_last_[i];
            effective_features_num_++;
        }
    }

    if(effective_features_num_ < 1)
    {
        measurement_data_.is_valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }    

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    measurement_data_.h_x = Eigen::MatrixXd::Zero(effective_features_num_, 12); //23
    measurement_data_.h.resize(effective_features_num_);

    for(int i = 0; i < effective_features_num_; i++)
    {
        const LidarPointType &laser_point  = laser_cloud_origin_->points[i];
        Eigen::Vector3d point_this_be(laser_point.x, laser_point.y, laser_point.z);
        Eigen::Matrix3d point_be_crossmat;
        point_be_crossmat << math::hat(point_this_be);
        Eigen::Vector3d point_this = state_.extrinsic_rotation * point_this_be + state_.extrinsic_translation;
        Eigen::Matrix3d point_crossmat;
        point_crossmat<<math::hat(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const LidarPointType &norm_point = corresponding_normal_vector_->points[i];
        Eigen::Vector3d normal_vector(norm_point.x, norm_point.y, norm_point.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        Eigen::Vector3d C(state_.rotation.conjugate()*normal_vector);
        Eigen::Vector3d A(point_crossmat * C);

        if(ekf_parameter_.extrinsic_estimate_enable)
        {
            Eigen::Vector3d B(point_be_crossmat * state_.extrinsic_rotation.conjugate() * C);   //s.rot.conjugate()*normal_vector);
            measurement_data_.h_x.block<1, 12>(i,0) << norm_point.x, norm_point.y, norm_point.z, 
                                                       A[0], A[1], A[2],
                                                       B[0], B[1], B[2],
                                                       C[0], C[1], C[2];
        }
        else
        {
            measurement_data_.h_x.block<1, 12>(i,0) << norm_point.x, norm_point.y, norm_point.z, 
                                                       A[0], A[1], A[2],
                                                       0.0, 0.0, 0.0, 
                                                       0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        measurement_data_.h(i) = -norm_point.intensity;
    }

}


Eigen::Matrix<double, 23, 1> EkfProcessor::CalculateStateBoxMinus(EkfState& state_propagated)
{
    // dx: 上一次状态变量估计值与递推值之间的差值
    Eigen::Matrix<double, 23, 1> dx;

    // x_.boxminus(dx, x_propagated);
    // 计算流形的差值SO3----rotation:3
    dx.block<3,1>(3,0) = math::BoxMinusSO3(state_.rotation, 
                                           state_propagated.rotation);

    // 计算流形的差值VECTOR3----position:0
    dx.block<3,1>(0,0) = state_.position - state_propagated.position;

    // 计算流形的差值VECTOR3----velocity:12
    dx.block<3,1>(12,0) = state_.velocity - state_propagated.velocity;

    // 计算流形的差值VECTOR3----bias_acc:18
    dx.block<3,1>(18,0) = state_.bias_acc - state_propagated.bias_acc;

    // 计算流形的差值VECTOR3----bias_gyro:15
    dx.block<3,1>(15,0) = state_.bias_gyro - state_propagated.bias_gyro;

    // 计算流形的差值S2----gravity:21
    dx.block<2,1>(21,0) = math::BoxMinusS2(state_.gravity, 
                                           state_propagated.gravity);
    
    // 计算流形的差值SO3----extrinsic_rotation:6
    dx.block<3,1>(6,0) = math::BoxMinusSO3(state_.extrinsic_rotation, 
                                            state_propagated.extrinsic_rotation);

    // 计算流形的差值VECTOR3----extrinsic_translation:9
    dx.block<3,1>(9,0) = state_.extrinsic_translation - state_propagated.extrinsic_translation;

    return dx;
}




}  // namespace registration
