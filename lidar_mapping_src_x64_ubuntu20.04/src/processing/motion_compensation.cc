///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_01_20
///////////////////////////////////////////////////////////////////////////////

#include "src/processing/motion_compensation.h"
// #include <tf/transform_datatypes.h>

namespace processing
{

MotionCompensation::MotionCompensation(int lidar_n_scans)
    : lidar_n_scans_(lidar_n_scans)
{
}

MotionCompensation::~MotionCompensation()
{
}

// 如果调用时提供了预测位姿，使用提供的预测位姿进行配准
void MotionCompensation::LinearInterpolation(pcl::PointCloud<PointType>& pointcloud,
                                             pcl::PointCloud<PointType>& pointcloud_compensation,
                                             Eigen::Matrix4f transform_previous,
                                             Eigen::Matrix4f transform_current
                                             )
{       
      int pointcloud_size = pointcloud.points.size();
      float start_orientation = -atan2(pointcloud.points[0].y, 
                                       pointcloud.points[0].x);

      float end_orientation = -atan2(pointcloud.points[pointcloud_size - 1].y, 
                                     pointcloud.points[pointcloud_size - 1].x) + 2 * M_PI;

      // 结束方位角与开始方位角差值控制在(PI,3*PI)范围
      if(end_orientation - start_orientation > 3 * M_PI)
      {
          end_orientation -= 2 * M_PI;
      }
      else if(end_orientation - start_orientation < M_PI)
      {
          end_orientation += 2 * M_PI;
      }

      bool is_half_passed = false;
      int count = pointcloud_size;
      registration::PointType point;

      for(int i = 0; i < pointcloud_size; i++)
      {
         point.x = pointcloud.points[i].x;
         point.y = pointcloud.points[i].y;
         point.z = pointcloud.points[i].z;

         // 计算点的仰角(单位：度),根据仰角排列激光线号，velodyne每两个scan之间间隔2度
         float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
         int scan_id;

         // 四舍五入后的仰角(加减0.5截断效果等于四舍五入)  关于强制类型转换：float->int, 结果都是取最接近0的整数，如：(int)-0.2 = 0; (int)-1.2 = -1; (int)-1.8 = -1; (int)0.2 = 0; (int)1.2 = 1; (int)1.8 = 1;
         int rounded_angle = int(angle + (angle<0.0?-0.5:+0.5)); //这种简单算法实现了正负数的四舍五入取整，如：(int)-0.2-0.5 = 0; (int)-1.2-0.5 = -1; (int)-1.8-0.5 = -2; (int)0.2+0.5 = 0; (int)1.2+0.5 = 1; (int)1.8 +0.5= 2;

         if(rounded_angle > 0)
         {
             scan_id = rounded_angle; //大于0的ID：1,3,5,7,9,11,13,15，即水平面以上位置扫描线的ID号为奇数
         }
         else
         {
             scan_id = rounded_angle + (lidar_n_scans_ - 1); //小于0的ID：（-1,-3,-5,-7,-9,-11,-13,-15）+ 15 = 14,12,10,8,6,4,2,0，即水平面以下位置扫描线的ID号为偶数
         }

         if(scan_id > (lidar_n_scans_ - 1) || scan_id < 0 )  //如果算出的ID号大于15，或者小于0，说明该点的垂直角度不在正负15度以内，属于异常值，不参与运算
         {
             count--;   // 将16线以外的杂点剔除
             continue;  //将有效点数减去1，然后进入下一层for循环，判断下一个点的ID号。
         }

         //该点的旋转角
         float current_orientation = -atan2(point.y, point.x);

         //初始扫描角度是不固定的，也就是说，startOri可能在-180度到+180度之间的任何位置，而atan2()函数只能将计算角度放在（-180,180）之间换算到【startOri，endOri】之间
         if(!is_half_passed)  //根据扫描线是否旋转过半选择与起始位置还是终止位置进行差值计算，从而进行补偿，整段代码将ori换算到【startOri，endOri】之间
         {
           if(current_orientation < start_orientation - M_PI / 2) //确保-pi/2 < current_orientation - start_orientation < 3*pi/2
           {
                current_orientation += 2 * M_PI;
           }
           else if (current_orientation > start_orientation + M_PI * 3 / 2)
           {
                current_orientation -= 2 * M_PI;
           }

           if(current_orientation - start_orientation > M_PI) //已经旋转过半，进入else语句处理
           {
             is_half_passed = true;
           }
         }
         else
         {
            current_orientation += 2 * M_PI;  //确保-3*pi/2 < current_orientation - end_orientation < pi/2

            if(current_orientation < end_orientation - M_PI * 3 / 2)
            {
               current_orientation += 2 * M_PI;
            }
            else if (current_orientation > end_orientation + M_PI / 2)
            {
               current_orientation -= 2 * M_PI;
            }
         }

         // ROS_INFO("ID:%d  ori:%f start_orientation:%f  end_orientation:%f  ", scan_id, current_orientation / M_PI *180, start_orientation / M_PI *180.0, end_orientation / M_PI *180.0);

         //计算该点距离这一包数据的第一个点的相对时间
         float real_time_ratio = (current_orientation - start_orientation) / (end_orientation - start_orientation);

         double delta_x = transform_current(0,3) - transform_previous(0,3);
         double delta_y = transform_current(1,3) - transform_previous(1,3);
         double delta_z = transform_current(2,3) - transform_previous(2,3);
        
         float x_interpolation = transform_previous(0,3) + real_time_ratio*delta_x;
         float y_interpolation = transform_previous(1,3) + real_time_ratio*delta_y;
         float z_interpolation = transform_previous(2,3) + real_time_ratio*delta_z;

         Eigen::Quaternionf quaterniond_previous(transform_previous.block<3,3>(0,0));
         Eigen::Quaternionf quaterniond_current(transform_current.block<3,3>(0,0));
         Eigen::Quaternionf quaterniond_interpolation = quaterniond_previous.slerp(real_time_ratio, 
                                                                                   quaterniond_current);

        Eigen::Isometry3f transformation_point_to_map = Eigen::Isometry3f::Identity();
        transformation_point_to_map.rotate(quaterniond_interpolation.toRotationMatrix());
        transformation_point_to_map.pretranslate(Eigen::Vector3f(x_interpolation, 
                                                                 y_interpolation, 
                                                                 z_interpolation));

        Eigen::Vector3f point_in_lidar; 
        point_in_lidar << point.x, point.y, point.z;
        Eigen::Vector3f point_in_map = transformation_point_to_map * point_in_lidar;


        Eigen::Isometry3f transformation_map_to_init_lidar = Eigen::Isometry3f::Identity();

        // 如果雷达的时间戳是第一个点的时间戳，所有雷达的点要变换到前一帧的坐标系下
        // transformation_map_to_init_lidar.rotate(quaterniond_previous.toRotationMatrix());
        // transformation_map_to_init_lidar.pretranslate(Eigen::Vector3f(transform_previous(0,3), 
        //                                                               transform_previous(1,3), 
        //                                                               transform_previous(2,3)));

        // 如果雷达的时间戳是最后一个点的时间戳，所有雷达的点要变换到后一帧的坐标系下
        transformation_map_to_init_lidar.rotate(quaterniond_current.toRotationMatrix());
        transformation_map_to_init_lidar.pretranslate(Eigen::Vector3f(transform_current(0,3), 
                                                                      transform_current(1,3), 
                                                                      transform_current(2,3)));

        Eigen::Vector3f point_in_init_lidar = transformation_map_to_init_lidar.inverse() * point_in_map;

        registration::PointType point_compensation;
        point_compensation.x = point_in_init_lidar[0];
        point_compensation.y = point_in_init_lidar[1];
        point_compensation.z = point_in_init_lidar[2];

        if(typeid(registration::PointType) == typeid(pcl::PointXYZI))
        {
            point_compensation.intensity = pointcloud.points[i].intensity;
        }

        pointcloud_compensation.push_back(point_compensation);
      }
}


}  // namespace processing


