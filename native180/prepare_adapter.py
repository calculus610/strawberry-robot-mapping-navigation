"""Apply narrowly scoped adapters to the frozen robot source copy."""
from pathlib import Path
import re

root = Path(__file__).resolve().parent
source = root / 'ws/src/fast_lio/src/laserMapping.cpp'
s = (root / 'baseline/fast_lio/src/laserMapping.cpp').read_text()

def replace(old, new):
    global s
    assert s.count(old) == 1, (old[:100], s.count(old))
    s = s.replace(old, new)

replace('#include "preprocess.h"', '#include "preprocess.h"\n#include "native_height_export.hpp"')
replace('vect3 pos_lid;', '''vect3 pos_lid;
bool native_map_export = false;
bool forward180 = false;
double native_bottom_height = 0.20;
NativeHeightReference native_reference;

bool snapshot_native_map()
{
    if (ikdtree.Root_Node == nullptr) return false;
    // Same native flatten API as the upstream map visualization example.
    // A fresh snapshot replaces the output; there is NO scan-history cache.
    // Invoked on the same single-threaded executor as map_incremental().
    PointVector points;
    ikdtree.flatten(ikdtree.Root_Node, points, NOT_RECORD);
    featsFromMap->points.swap(points);
    featsFromMap->width = static_cast<uint32_t>(featsFromMap->points.size());
    featsFromMap->height = 1;
    featsFromMap->is_dense = true;
    return true;
}''')
replace('''void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{''', '''void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    if (native_map_export)
    {
        if (!snapshot_native_map()) return;
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*featsFromMap, msg);
        msg.header.stamp = get_ros_time(lidar_end_time);
        msg.header.frame_id = "camera_init";
        pubLaserCloudMap->publish(msg);
        return;
    }''')
replace('''    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);''', '''    if (native_map_export)
    {
        if (!snapshot_native_map()) throw std::runtime_error("Native map is not initialized");
        if (pcd_writer.writeBinary(map_file_path, *featsFromMap) != 0)
            throw std::runtime_error("Could not write native PCD");
    }
    else pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);''')
replace('''        this->declare_parameter<bool>("publish.path_en", true);''', '''        native_map_export = this->declare_parameter<bool>("experiment.native_map_export", false);
        forward180 = this->declare_parameter<bool>("experiment.forward180", false);
        native_bottom_height = this->declare_parameter<double>("experiment.sensor_bottom_height", 0.20);
        native_reference_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/fastlio/native_height_reference", rclcpp::QoS(1).reliable().transient_local());
        RCLCPP_INFO(this->get_logger(),
            "EXPERIMENT native_ikdtree_export=%d forward180_after_deskew=%d bottom_height=%.3f; native map update unchanged",
            native_map_export, forward180, native_bottom_height);
        this->declare_parameter<bool>("publish.path_en", true);''')
replace('''            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time)''', '''            if (native_map_export && !native_reference.ready)
            {
                NativeImuStats imu_stats;
                for (const auto &m : Measures.imu)
                {
                    imu_stats.add(V3D(m->linear_acceleration.x, m->linear_acceleration.y,
                        m->linear_acceleration.z), V3D(m->angular_velocity.x,
                        m->angular_velocity.y, m->angular_velocity.z));
                }
                if (native_reference.observe(lidar_end_time, state_point.pos,
                    state_point.rot.toRotationMatrix(),
                    V3D(state_point.grav[0], state_point.grav[1], state_point.grav[2]),
                    state_point.offset_T_L_I, state_point.offset_R_L_I.toRotationMatrix(),
                    state_point.vel.norm(), imu_stats, native_bottom_height))
                {
                    RCLCPP_INFO(this->get_logger(),
                        "NATIVE_HEIGHT_REFERENCE_READY fixed ground=[%.6f %.6f %.6f] window_rms=%.6f imu_samples=%zu. No ground-plane fitting or runtime reset. Projection band is ground-relative.",
                        native_reference.ground.x(), native_reference.ground.y(), native_reference.ground.z(),
                        native_reference.window_imu.relative_rms(), native_reference.window_imu.count);
                }
                else
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "NATIVE_HEIGHT_REFERENCE_WAIT reason=%s elapsed=%.2f frames=%d velocity=%.5f max_gyro=%.5f frame_rms=%.5f window_rms=%.5f",
                        native_reference.reason.c_str(), native_reference.elapsed, native_reference.samples,
                        state_point.vel.norm(), imu_stats.max_gyro, imu_stats.relative_rms(),
                        native_reference.window_imu.relative_rms());
                }
            }
            if (forward180)
            {
                const auto before = feats_undistort->size();
                keep_forward_half(*feats_undistort, Lidar_R_wrt_IMU);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                    "FORWARD180 full_timing_preserved input=%zu kept=%zu", before, feats_undistort->size());
                if (feats_undistort->empty()) return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time)''')
replace('''        if (map_pub_en) publish_map(pubLaserCloudMap_);''', '''        if (native_map_export && native_reference.ready)
        {
            geometry_msgs::msg::PoseStamped ref;
            ref.header.stamp = get_ros_time(lidar_end_time);
            ref.header.frame_id = "camera_init";
            ref.pose.position.x = native_reference.ground.x();
            ref.pose.position.y = native_reference.ground.y();
            ref.pose.position.z = native_reference.ground.z();
            ref.pose.orientation.x = native_reference.rotation.x();
            ref.pose.orientation.y = native_reference.rotation.y();
            ref.pose.orientation.z = native_reference.rotation.z();
            ref.pose.orientation.w = native_reference.rotation.w();
            native_reference_pub_->publish(ref);
        }
        if (map_pub_en) publish_map(pubLaserCloudMap_);''')
replace('''            save_to_pcd();
            res->success = true;
            res->message = "Map saved.";''', '''            try
            {
                save_to_pcd();
                res->success = true;
                res->message = "Map saved.";
            }
            catch (const std::exception &e)
            {
                res->success = false;
                res->message = e.what();
            }''')
replace('''    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;''', '''    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr native_reference_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;''')
replace('''    if (pcl_wait_save->size() > 0 && pcd_save_en)''', '''    if (native_map_export && pcd_save_en && ikdtree.Root_Node != nullptr)
    {
        try { save_to_pcd(); std::cout << "NATIVE_MAP_SAVED " << map_file_path << std::endl; }
        catch (const std::exception &e) { std::cerr << "NATIVE_MAP_SAVE_FAILED " << e.what() << std::endl; return 1; }
    }
    if (pcl_wait_save->size() > 0 && pcd_save_en)''')

# Prove that the native estimator / insertion / moving-window implementation
# has not been rewritten by this adapter patch.
baseline = (root / 'baseline/fast_lio/src/laserMapping.cpp').read_text()
for name in ('map_incremental', 'lasermap_fov_segment', 'h_share_model'):
    pattern = rf'void {name}\([^\n]*\)\n\{{.*?\n\}}'
    before, after = re.search(pattern, baseline, re.S), re.search(pattern, s, re.S)
    assert before and after and before.group() == after.group(), name
source.write_text(s)
print('Patched adapters; native estimator, insertion and sliding-window functions unchanged.')
