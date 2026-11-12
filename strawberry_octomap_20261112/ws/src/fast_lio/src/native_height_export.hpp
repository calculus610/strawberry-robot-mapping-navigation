// Experiment adapters only. No changes to FAST-LIO's filter or ikd-tree updates.
#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

struct NativeImuStats
{
    size_t count = 0;
    Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();
    double sum_acc_squared_norm = 0.0;
    double max_gyro = 0.0;
    bool valid = true;

    void add(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro)
    {
        if (!acc.allFinite() || !gyro.allFinite()) { valid = false; return; }
        ++count;
        sum_acc += acc;
        sum_acc_squared_norm += acc.squaredNorm();
        max_gyro = std::max(max_gyro, gyro.norm());
    }

    void merge(const NativeImuStats &other)
    {
        count += other.count;
        sum_acc += other.sum_acc;
        sum_acc_squared_norm += other.sum_acc_squared_norm;
        max_gyro = std::max(max_gyro, other.max_gyro);
        valid = valid && other.valid;
    }

    double relative_rms() const
    {
        if (!valid || count < 2) return std::numeric_limits<double>::infinity();
        const Eigen::Vector3d mean = sum_acc / count;
        if (mean.norm() < 1e-6) return std::numeric_limits<double>::infinity();
        const double variance = std::max(0.0, sum_acc_squared_norm / count - mean.squaredNorm());
        return std::sqrt(variance) / mean.norm();
    }
};

struct NativeHeightReference
{
    bool ready = false;
    double start = -1.0;
    int samples = 0;
    Eigen::Vector3d start_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d start_up = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d sum_up = Eigen::Vector3d::Zero();
    Eigen::Vector3d sum_bottom = Eigen::Vector3d::Zero();
    Eigen::Vector3d ground = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    NativeImuStats window_imu;
    std::string reason = "waiting_for_imu";
    double elapsed = 0.0;

    // The existing extrinsic_T is lidar-origin-in-IMU. The MID360 lidar origin
    // is 47 mm above its bottom. At level mounting this gives IMU height
    // 0.20 + 0.047 - 0.04412 = 0.20288 m, NOT 0.20 or 0.247 m.
    bool observe(double stamp, const Eigen::Vector3d &pos,
                 const Eigen::Matrix3d &body_rotation,
                 const Eigen::Vector3d &gravity,
                 const Eigen::Vector3d &lidar_translation,
                 const Eigen::Matrix3d &lidar_rotation,
                 double velocity, const NativeImuStats &imu,
                 double bottom_height)
    {
        if (ready) return false;
        if (!gravity.allFinite() || gravity.norm() < 1.0 || !pos.allFinite() ||
            !std::isfinite(stamp) || !std::isfinite(velocity) ||
            !imu.valid || imu.count < 2 || !std::isfinite(imu.relative_rms()))
        {
            start = -1.0;
            reason = "invalid_imu_or_state";
            return false;
        }
        if (velocity > 0.05 || imu.max_gyro > 0.05)
        {
            start = -1.0;
            reason = velocity > 0.05 ? "velocity" : "angular_velocity";
            return false;
        }
        const Eigen::Vector3d up = -gravity.normalized();
        const Eigen::Vector3d bottom = pos + body_rotation *
            (lidar_translation + lidar_rotation * Eigen::Vector3d(0, 0, -0.047));
        if (start < 0.0 || stamp < start || (pos - start_pos).norm() > 0.03 ||
            up.dot(start_up) < std::cos(0.5 * M_PI / 180.0))
        {
            start = stamp;
            start_pos = pos;
            start_up = up;
            samples = 0;
            sum_up.setZero();
            sum_bottom.setZero();
            window_imu = NativeImuStats();
        }
        ++samples;
        sum_up += up;
        sum_bottom += bottom;
        window_imu.merge(imu);
        elapsed = stamp - start;
        reason = "collecting_stationary_window";
        if (elapsed < 3.0 || samples < 20) return false;
        // Evaluate the full window, not the single largest sample per frame.
        // Across-frame mean changes also contribute to the pooled variance.
        if (window_imu.relative_rms() > 0.02)
        {
            start = -1.0;
            reason = "imu_window_rms";
            return false;
        }
        const Eigen::Vector3d z = sum_up.normalized();
        Eigen::Vector3d x = Eigen::Vector3d::UnitX() - z.x() * z;
        if (x.norm() < 0.1) { start = -1.0; return false; }
        x.normalize();
        Eigen::Matrix3d basis;
        basis.col(0) = x;
        basis.col(1) = z.cross(x);
        basis.col(2) = z;
        rotation = Eigen::Quaterniond(basis).normalized();
        ground = sum_bottom / samples - bottom_height * z;
        ready = true;
        reason = "ready";
        return true;
    }
};

// Called after full-scan timing/IMU integration/deskew and before matching.
// Body +X is robot forward; the existing mounting has lidar/body axes aligned.
template <typename Cloud>
void keep_forward_half(Cloud &cloud, const Eigen::Matrix3d &lidar_to_body)
{
    auto &pts = cloud.points;
    pts.erase(std::remove_if(pts.begin(), pts.end(), [&](const auto &p) {
        return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            (lidar_to_body * Eigen::Vector3d(p.x, p.y, p.z)).x() < 0.0;
    }), pts.end());
    cloud.width = static_cast<uint32_t>(pts.size());
    cloud.height = 1;
    cloud.is_dense = true;
}
