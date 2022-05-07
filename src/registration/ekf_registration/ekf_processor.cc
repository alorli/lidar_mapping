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
namespace
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
		    + (1 - std::cos(norm)) / squaredNorm * hat(v) 
			+ (1 - std::sin(norm) / norm) / squaredNorm * hat(v) * hat(v);
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
			value = -hat(gravity)*basis;
		}
		else
		{
			Eigen::Vector3d bu = basis*delta;
            Eigen::Quaterniond delta_rotation = exp(bu, 0.5);
            value = -delta_rotation.toRotationMatrix()*hat(gravity)*AMatrix(bu).transpose()*basis;

		}

        return value;
    }
        
    Eigen::Matrix<double, 2, 3> CalculateNx()
    {
        return 1.0/kGravityBase/kGravityBase*basis.transpose()*hat(gravity);
    }


    Eigen::Vector3d gravity;
    Eigen::Matrix<double, 3, 2> basis;
};

}


EkfProcessor::EkfProcessor(std::string cfg_file_path)
    :state_(EkfState()),
     process_covariance_(Eigen::Matrix<double, 23, 23>::Identity())
{
    cfg_file_ = YAML::LoadFile(cfg_file_path + "/cfg.yaml");
    
    ekf_parameter_.max_iteration = cfg_file_["ekf_registration"]["ekf_parameter"]["max_iteration"].as<int>();
    ekf_parameter_.converge_threshold = cfg_file_["ekf_registration"]["ekf_parameter"]["converge_threshold"].as<double>();

    std::cout << std::endl;
    std::cout << "\033[32m" << "----------EkfProcessor parameter:----------" << "\033[0m" << std::endl;
    std::cout << "ekf_parameter_.max_iteration:" << "\033[33m"  << ekf_parameter_.max_iteration  << "\033[0m" << std::endl;
    std::cout << "ekf_parameter_.converge_threshold:" << "\033[33m"  << ekf_parameter_.converge_threshold  << "\033[0m" << std::endl;
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
    f.block<3,1>(0,0) = input_state.gyro - state_.bias_gyro;
    f.block<3,1>(3,0) = state_.velocity;
    f.block<3,1>(6,0) = state_.rotation*(input_state.acc - state_.bias_acc) + state_.gravity;

    static bool print_1st = true;
    if(print_1st)
    {
        std::cout << "----f:" << f.transpose() << std::endl;
    }
    

    // 计算 df_dx
    Eigen::Matrix<double, 24, 23> df_dx = Eigen::Matrix<double, 24, 23>::Zero();
    df_dx.block<3,3>(0,12) = -Eigen::Matrix3d::Identity();
    df_dx.block<3,3>(3,6) = Eigen::Matrix3d::Identity();
    df_dx.block<3,3>(6,0) = -1*state_.rotation.toRotationMatrix()*hat(input_state.acc - state_.bias_acc);
    df_dx.block<3,3>(6,9) = -1*state_.rotation.toRotationMatrix();

    GravityManifold gravity_manifold_old(state_.gravity);
    Eigen::Matrix<double, 3, 2> mx_old = gravity_manifold_old.CalculateMx(Eigen::Vector2d::Zero());
    df_dx.block<3,2>(6,15) = mx_old;

    // if(print_1st)
    // {
    //     std::cout << "----df_dx:" << std::endl << df_dx << std::endl;
    // }

    // 计算 df_dw
    Eigen::Matrix<double, 24, 12> df_dw = Eigen::Matrix<double, 24, 12>::Zero();
    df_dw.block<3,3>(0,3) = -Eigen::Matrix3d::Identity();
    df_dw.block<3,3>(6,0) = -1*state_.rotation.toRotationMatrix();
    df_dw.block<3,3>(9,6) = Eigen::Matrix3d::Identity();
    df_dw.block<3,3>(12,9) = Eigen::Matrix3d::Identity();

    // if(print_1st)
    // {
    //     std::cout << "----df_dw:" << std::endl << df_dw << std::endl;
    // }


    // 用 f 预测状态
    EkfState state_old = state_;
    state_.rotation = state_.rotation*exp(f.block<3,1>(0,0), delta_time/2.0);
    state_.position += f.block<3,1>(3,0) * delta_time;
    state_.velocity += f.block<3,1>(6,0) * delta_time;
    state_.bias_acc += f.block<3,1>(9,0) * delta_time;
    state_.bias_gyro += f.block<3,1>(12,0) * delta_time;
    state_.gravity = exp(f.block<3,1>(15,0), delta_time/2.0)*state_.gravity;
    state_.extrinsic_rotation = state_.extrinsic_rotation*exp(f.block<3,1>(18,0), delta_time/2.0);
    state_.extrinsic_translation += f.block<3,1>(21,0) * delta_time;
    
    // 计算 F_x 和 F_w
    Eigen::Matrix<double, 23, 23> F_x = Eigen::Matrix<double, 23, 23>::Identity();
    Eigen::Matrix<double, 23, 23> f_x = Eigen::Matrix<double, 23, 23>::Zero();
    Eigen::Matrix<double, 23, 12> f_w = Eigen::Matrix<double, 23, 12>::Zero();

    // 计算 F_x 和 F_w  ----- state: rotation
    Eigen::Vector3d rotation_vector = -1.0*f.block<3,1>(0,0)*delta_time;

    if(print_1st)
    {
        std::cout << "####################:" << std::endl;
    }

    Eigen::Quaterniond rotation = exp(rotation_vector, 0.5);
    F_x.block<3,3>(0,0) = rotation.toRotationMatrix();

    Eigen::Matrix<double, 3, 3> amatrix_rotation = AMatrix(rotation_vector);
    f_x.block<3,23>(0,0) = amatrix_rotation * df_dx.block<3,23>(0,0);
    f_w.block<3,12>(0,0) = amatrix_rotation * df_dw.block<3,12>(0,0);

    if(print_1st)
    {
        std::cout << "----delta_time:" << std::endl << delta_time << std::endl;
        std::cout << "----rotation_vector:" << std::endl << rotation_vector << std::endl;
        std::cout << "----rotation:x" << rotation.x() << " y:" << rotation.y()<< " z:" << rotation.z()<< " w:" << rotation.w() << std::endl;
        // std::cout << "----amatrix_rotation:" << std::endl << amatrix_rotation << std::endl;
    }

    // 计算 F_x 和 F_w  ----- state: position
    f_x.block<3,23>(3,0) = df_dx.block<3,23>(3,0);
    f_w.block<3,12>(3,0) = df_dw.block<3,12>(3,0);    

    // 计算 F_x 和 F_w  ----- state: velocity
    f_x.block<3,23>(6,0) = df_dx.block<3,23>(6,0);
    f_w.block<3,12>(6,0) = df_dw.block<3,12>(6,0);     

    // 计算 F_x 和 F_w  ----- state: bias_acc
    f_x.block<3,23>(9,0) = df_dx.block<3,23>(9,0);
    f_w.block<3,12>(9,0) = df_dw.block<3,12>(9,0);  

    // 计算 F_x 和 F_w  ----- state: bias_gyro
    f_x.block<3,23>(12,0) = df_dx.block<3,23>(12,0);
    f_w.block<3,12>(12,0) = df_dw.block<3,12>(12,0);  

    // 计算 F_x 和 F_w  ----- state: gravity
    Eigen::Vector3d gravity_vector = f.block<3,1>(15,0)*delta_time;
    Eigen::Matrix3d delta_gravity_matrix = exp(gravity_vector, 0.5).toRotationMatrix();

    GravityManifold gravity_manifold_new(state_.gravity);
    Eigen::Matrix<double, 2, 3> nx = gravity_manifold_new.CalculateNx();
    Eigen::Matrix<double, 3, 2> mx = mx_old;
    F_x.block<2,2>(15,15) = nx * delta_gravity_matrix * mx;

    Eigen::Matrix<double, 2, 3> temp_matrix_1 = -nx * delta_gravity_matrix * hat(state_old.gravity) * AMatrix(gravity_vector).transpose();
    f_x.block<2,23>(15,0) = temp_matrix_1 * df_dx.block<3,23>(15,0);
    f_w.block<2,12>(15,0) = temp_matrix_1 * df_dw.block<3,12>(15,0); 

    // 计算 F_x 和 F_w  ----- state: extrinsic_rotation
    Eigen::Vector3d extrinsic_rotation_vector = -1.0*f.block<3,1>(18,0)*delta_time;
    Eigen::Quaterniond extrinsic_rotation = exp(extrinsic_rotation_vector, 0.5);
    F_x.block<3,3>(17,17) = extrinsic_rotation.toRotationMatrix();

    Eigen::Matrix<double, 3, 3> amatrix_extrinsic_rotation = AMatrix(extrinsic_rotation_vector);
    f_x.block<3,23>(17,0) = amatrix_extrinsic_rotation * df_dx.block<3,23>(17,0);
    f_w.block<3,12>(17,0) = amatrix_extrinsic_rotation * df_dw.block<3,12>(17,0);

    // 计算 F_x 和 F_w  ----- state: extrinsic_translation
    f_x.block<3,23>(20,0) = df_dx.block<3,23>(20,0);
    f_w.block<3,12>(20,0) = df_dw.block<3,12>(20,0);  

    // if(print_1st)
    // {
    //     std::cout << "****f_x:" << std::endl << f_x << std::endl;
    //     std::cout << "****f_w:" << std::endl << f_w << std::endl;
    //     std::cout << "****F_x:" << std::endl << F_x << std::endl;
    //     std::cout << "****Q:" << std::endl << Q << std::endl;
    //     std::cout << "****process_covariance_:" << std::endl << process_covariance_ << std::endl;
    // }

    // 更新一步预测误差方差阵
    F_x += f_x * delta_time;
    process_covariance_ = F_x*process_covariance_*F_x.transpose() + (f_w*delta_time) * Q * (f_w*delta_time).transpose();


    if(print_1st)
    {
        std::cout << "----F_x:" << std::endl << F_x << std::endl;
        std::cout << "----process_covariance_:" << std::endl << process_covariance_ << std::endl;
    }

    print_1st = false;
}


}  // namespace registration
