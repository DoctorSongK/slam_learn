#include "imu_preintegration.h"
#include <glog/logging.h>

namespace sad {

IMUPreintegration::IMUPreintegration(Options options) {
    bg_ = options.init_bg_;
    ba_ = options.init_ba_;
    const float ng2 = options.noise_gyro_ * options.noise_gyro_;
    const float na2 = options.noise_acce_ * options.noise_acce_;
    noise_gyro_acce_.diagonal() << ng2, ng2, ng2, na2, na2, na2;
}

void IMUPreintegration::Integrate(const IMU &imu, double dt) {
    // 去掉零偏的测量
    Vec3d gyr = imu.gyro_ - bg_;  // 陀螺
    Vec3d acc = imu.acce_ - ba_;  // 加计

    // 更新dv, dp, 见(4.13), (4.16)
    // ques: 为啥这里dR转换成了矩阵
    dp_ = dp_ + dv_ * dt + 0.5f * dR_.matrix() * acc * dt * dt;
    dv_ = dv_ + dR_ * acc * dt;

    // dR先不更新，因为A, B阵还需要现在的dR

    // 运动方程雅可比矩阵系数，A,B阵，见(4.29)
    // 另外两项在后面
    Eigen::Matrix<double, 9, 9> A;
    A.setIdentity();
    Eigen::Matrix<double, 9, 6> B;
    B.setZero();

    Mat3d acc_hat = SO3::hat(acc);
    double dt2 = dt * dt;

    // NOTE A, B左上角块与公式稍有不同，关于姿态的噪声在后面代码中完成填写
    A.block<3, 3>(3, 0) = -dR_.matrix() * dt * acc_hat;
    A.block<3, 3>(6, 0) = -0.5f * dR_.matrix() * acc_hat * dt2;
    A.block<3, 3>(6, 3) = dt * Mat3d::Identity();

    B.block<3, 3>(3, 3) = dR_.matrix() * dt;
    B.block<3, 3>(6, 3) = 0.5f * dR_.matrix() * dt2;

    // 更新各雅可比，见式(4.39)，这里更新的是关于零偏的偏导数，用于在零偏校正后的递推
    dP_dba_ = dP_dba_ + dV_dba_ * dt - 0.5f * dR_.matrix() * dt2;                      // (4.39d)
    dP_dbg_ = dP_dbg_ + dV_dbg_ * dt - 0.5f * dR_.matrix() * dt2 * acc_hat * dR_dbg_;  // (4.39e)
    dV_dba_ = dV_dba_ - dR_.matrix() * dt;                                             // (4.39b)
    dV_dbg_ = dV_dbg_ - dR_.matrix() * dt * acc_hat * dR_dbg_;                         // (4.39c)

    // 旋转部分
    Vec3d omega = gyr * dt;         // 转动量
    Mat3d rightJ = SO3::jr(omega);  // 右雅可比，用于求李代数的左、右雅可比矩阵（BCH公式）
    SO3 deltaR = SO3::exp(omega);   // exp后
    dR_ = dR_ * deltaR;             // (4.9)

    A.block<3, 3>(0, 0) = deltaR.matrix().transpose();                                  
    B.block<3, 3>(0, 0) = rightJ * dt;

    // 更新噪声项
    cov_ = A * cov_ * A.transpose() + B * noise_gyro_acce_ * B.transpose();

    // 更新dR_dbg
    dR_dbg_ = deltaR.matrix().transpose() * dR_dbg_ - rightJ * dt;  // (4.39a)

    // 增量积分时间
    dt_ += dt;
}

// IMU预积分中姿态关于零偏的更新
SO3 IMUPreintegration::GetDeltaRotation(const Vec3d &bg) { return dR_ * SO3::exp(dR_dbg_ * (bg - bg_)); }  // (4.33)

// IMU预积分中速度关于零偏的更新
Vec3d IMUPreintegration::GetDeltaVelocity(const Vec3d &bg, const Vec3d &ba) {   // (4.36)
    return dv_ + dV_dbg_ * (bg - bg_) + dV_dba_ * (ba - ba_);
}

// IMU预积分中位置关于零偏的更新
Vec3d IMUPreintegration::GetDeltaPosition(const Vec3d &bg, const Vec3d &ba) {
    return dp_ + dP_dbg_ * (bg - bg_) + dP_dba_ * (ba - ba_);
}

// note: 如果不进行优化，预积分和直接积分的效果是完全一致的，都是将IMU的数据积分起来
// 同时并未考虑噪声值
NavStated IMUPreintegration::Predict(const sad::NavStated &start, const Vec3d &grav) const {
    SO3 Rj = start.R_ * dR_;                                                            // (4.18a)--暂未考虑噪声，其中预积分结果为等式左侧
    Vec3d vj = start.R_ * dv_ + start.v_ + grav * dt_;                                  // (4.18b)
    Vec3d pj = start.R_ * dp_ + start.p_ + start.v_ * dt_ + 0.5f * grav * dt_ * dt_;    // (4.18c)

    auto state = NavStated(start.timestamp_ + dt_, Rj, pj, vj);
    state.bg_ = bg_;
    state.ba_ = ba_;
    return state;
}

}  // namespace sad
