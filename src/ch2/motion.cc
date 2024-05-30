//
// Created by xiang on 22-12-29.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

/// 本节程序演示一个正在作圆周运动的车辆
/// 车辆的角速度与线速度可以在flags中设置

DEFINE_double(angular_velocity, 10.0, "角速度（角度）制");
DEFINE_double(linear_velocity, 5.0, "车辆前进线速度 m/s");
DEFINE_bool(use_quaternion, false, "是否使用四元数计算");
DEFINE_double(gravity, 9.8, "重力加速度 m/s^2");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    // 设置log文件保存路径及前缀
    // FLAGS_log_dir="PATH/prefix_"; 
    // 设置日志消息除了日志文件之外是否去标准输出
	// FLAGS_alsologtostderr = true; 
    // 设置是否在磁盘已满时避免日志记录到磁盘
    // FLAGS_stop_logging_if_full_disk = true;  
 
    // 设置log输出到终端等级
    FLAGS_stderrthreshold = google::INFO;
    // 设置log输出颜色
    FLAGS_colorlogtostderr = true; 

    // 用于解析GFLAG所所传进来的参数信息
    google::ParseCommandLineFlags(&argc, &argv, true);

    /// 可视化
    sad::ui::PangolinWindow ui;
    if (ui.Init() == false) {
        return -1;
    }

    // double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;  // 弧度制角速度
    // SE3 pose;                                                                    // TWB表示的位姿
    // Vec3d omega(0, 0, angular_velocity_rad);                                     // 角速度矢量
    // Vec3d v_body(FLAGS_linear_velocity, 0, 0);                                   // 本体系速度
    // const double dt = 0.05;                                                      // 每次更新的时间

    // while (ui.ShouldQuit() == false) {
    //     // 更新自身位置
    //     Vec3d v_world = pose.so3() * v_body;
    //     v_world(2) = -FLAGS_gravity * dt;
    //     pose.translation() += v_world * dt;
    //     // 更新自身旋转
    //     if (FLAGS_use_quaternion) {
    //         Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
    //         q.normalize();
    //         pose.so3() = SO3(q);
    //     } else {
    //         // 在这里可以看到旋转矩阵（四元数）等李群的李代数就是w^t(其实就是wt，w为角速度，t为时间也就是旋转角)
    //         pose.so3() = pose.so3() * SO3::exp(omega * dt);
    //     }

    //     LOG(INFO) << "pose: " << pose.translation().transpose();
    //     ui.UpdateNavState(sad::NavStated(0, pose, v_world));

    //     usleep(dt * 1e6);
    // }

    // ---------------------- 课后习题分割线-----------------------
    double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;  // 弧度制角速度
    SE3 pose;                                                                    // TWB表示的位姿
    Vec3d omega(0, 0, angular_velocity_rad);                                     // 角速度矢量
    Vec3d v_body(FLAGS_linear_velocity, 0, 0);                                   // 本体系速度
    Vec3d w_a(0,0,-FLAGS_gravity);
    const double dt = 0.05; 
    while (ui.ShouldQuit() == false) {
        // 更新自身位置
        Vec3d v_world = pose.so3() * v_body;
        Vec3d v_world1 = pose.so3() * v_body;
        LOG(INFO) << "111v_world: " << v_world(0) << " " << v_world(1) << " " << v_world(2);

        pose.translation() += v_world * dt + 0.5 * w_a * dt * dt;
        
        //Vec3d a_body = pose.so3().inverse() * w_a;
        
        v_world += w_a * dt;
        v_body = pose.so3().inverse() * v_world;
        
        //v_body += a_body * dt;
        
        
        // 更新自身旋转
        if (FLAGS_use_quaternion) {
            Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
            q.normalize();
            pose.so3() = SO3(q);
        } else {
            // 在这里可以看到旋转矩阵（四元数）等李群的李代数就是w^t(其实就是wt，w为角速度，t为时间也就是旋转角)
            pose.so3() = pose.so3() * SO3::exp(omega * dt);
        }
        
        LOG(INFO) << "v_body: " << v_body(0) << " " << v_body(1) << " " << v_body(2);
        // LOG(INFO) << "v_world: " << v_world(0) << " " << v_world(1) << " " << v_world(2);
        // LOG(INFO) << "pose: " << pose.translation().transpose();
        ui.UpdateNavState(sad::NavStated(0, pose, v_world1));
        usleep(dt * 1e6);
    }
    ui.Quit();
    return 0;
}