// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <algorithm>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <sstream>
#include <csignal>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include "native_height_export.hpp"
#include "native_cloud_filters.hpp"
#include "native_temporal_filter.hpp"
#include <std_msgs/msg/string.hpp>
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool    is_first_lidar = true;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

namespace
{
using TimechainSteadyClock = std::chrono::steady_clock;

struct TimechainDiagState
{
    bool enabled = false;
    TimechainSteadyClock::time_point window_start;
    TimechainSteadyClock::time_point last_lidar_callback;
    TimechainSteadyClock::time_point last_imu_callback;
    TimechainSteadyClock::time_point sync_wait_start;
    bool have_lidar_callback = false;
    bool have_imu_callback = false;
    bool have_lidar_header = false;
    bool have_imu_header = false;
    bool have_point_count = false;
    bool have_lidar_header_delta = false;
    bool have_imu_header_delta = false;
    bool sync_wait_active = false;
    uint64_t lidar_callbacks = 0;
    uint64_t imu_callbacks = 0;
    uint64_t lidar_rollbacks = 0;
    uint64_t imu_rollbacks = 0;
    uint64_t sync_empty_returns = 0;
    uint64_t sync_wait_returns = 0;
    uint64_t synced_scans = 0;
    double latest_lidar_header = 0.0;
    double latest_imu_header = 0.0;
    double lidar_callback_gap_max = 0.0;
    double imu_callback_gap_max = 0.0;
    double lidar_header_delta_min = 0.0;
    double lidar_header_delta_max = 0.0;
    double imu_header_delta_min = 0.0;
    double imu_header_delta_max = 0.0;
    double header_difference_max = 0.0;
    double sync_wait_max = 0.0;
    uint64_t point_count_min = 0;
    uint64_t point_count_max = 0;
    size_t lidar_buffer_max = 0;
    size_t imu_buffer_max = 0;
    size_t time_buffer_max = 0;
    double last_lidar_begin = 0.0;
    double last_lidar_end = 0.0;
    size_t last_selected_imu_count = 0;
    double last_selected_imu_first = 0.0;
    double last_selected_imu_last = 0.0;

    void ResetWindow(const TimechainSteadyClock::time_point &now)
    {
        window_start = now;
        lidar_callbacks = 0;
        imu_callbacks = 0;
        lidar_rollbacks = 0;
        imu_rollbacks = 0;
        sync_empty_returns = 0;
        sync_wait_returns = 0;
        synced_scans = 0;
        lidar_callback_gap_max = 0.0;
        imu_callback_gap_max = 0.0;
        have_lidar_header_delta = false;
        have_imu_header_delta = false;
        lidar_header_delta_min = 0.0;
        lidar_header_delta_max = 0.0;
        imu_header_delta_min = 0.0;
        imu_header_delta_max = 0.0;
        header_difference_max = 0.0;
        sync_wait_max = 0.0;
        have_point_count = false;
        point_count_min = 0;
        point_count_max = 0;
        lidar_buffer_max = 0;
        imu_buffer_max = 0;
        time_buffer_max = 0;
    }
};

TimechainDiagState timechain_diag;

bool TimechainDiagEnabledFromEnvironment()
{
    const char *value = std::getenv("FAST_LIO_TIMECHAIN_DIAG");
    if (value == nullptr || value[0] == '\0')
    {
        return false;
    }
    return std::strcmp(value, "1") == 0 ||
           std::strcmp(value, "true") == 0 ||
           std::strcmp(value, "TRUE") == 0 ||
           std::strcmp(value, "on") == 0 ||
           std::strcmp(value, "ON") == 0 ||
           std::strcmp(value, "yes") == 0 ||
           std::strcmp(value, "YES") == 0;
}

void TimechainDiagRecordLidar(const double header, const uint64_t point_count)
{
    if (!timechain_diag.enabled)
    {
        return;
    }

    const auto now = TimechainSteadyClock::now();
    if (timechain_diag.have_lidar_callback)
    {
        timechain_diag.lidar_callback_gap_max = std::max(
            timechain_diag.lidar_callback_gap_max,
            std::chrono::duration<double>(now - timechain_diag.last_lidar_callback).count());
    }
    if (timechain_diag.have_lidar_header)
    {
        const double delta = header - timechain_diag.latest_lidar_header;
        if (!timechain_diag.have_lidar_header_delta)
        {
            timechain_diag.lidar_header_delta_min = delta;
            timechain_diag.lidar_header_delta_max = delta;
            timechain_diag.have_lidar_header_delta = true;
        }
        else
        {
            timechain_diag.lidar_header_delta_min = std::min(timechain_diag.lidar_header_delta_min, delta);
            timechain_diag.lidar_header_delta_max = std::max(timechain_diag.lidar_header_delta_max, delta);
        }
    }

    timechain_diag.last_lidar_callback = now;
    timechain_diag.have_lidar_callback = true;
    timechain_diag.latest_lidar_header = header;
    timechain_diag.have_lidar_header = true;
    timechain_diag.lidar_callbacks++;
    if (!timechain_diag.have_point_count)
    {
        timechain_diag.point_count_min = point_count;
        timechain_diag.point_count_max = point_count;
        timechain_diag.have_point_count = true;
    }
    else
    {
        timechain_diag.point_count_min = std::min(timechain_diag.point_count_min, point_count);
        timechain_diag.point_count_max = std::max(timechain_diag.point_count_max, point_count);
    }
    if (timechain_diag.have_imu_header)
    {
        timechain_diag.header_difference_max = std::max(
            timechain_diag.header_difference_max,
            std::abs(timechain_diag.latest_lidar_header - timechain_diag.latest_imu_header));
    }
}

void TimechainDiagRecordImu(const double header)
{
    if (!timechain_diag.enabled)
    {
        return;
    }

    const auto now = TimechainSteadyClock::now();
    if (timechain_diag.have_imu_callback)
    {
        timechain_diag.imu_callback_gap_max = std::max(
            timechain_diag.imu_callback_gap_max,
            std::chrono::duration<double>(now - timechain_diag.last_imu_callback).count());
    }
    if (timechain_diag.have_imu_header)
    {
        const double delta = header - timechain_diag.latest_imu_header;
        if (!timechain_diag.have_imu_header_delta)
        {
            timechain_diag.imu_header_delta_min = delta;
            timechain_diag.imu_header_delta_max = delta;
            timechain_diag.have_imu_header_delta = true;
        }
        else
        {
            timechain_diag.imu_header_delta_min = std::min(timechain_diag.imu_header_delta_min, delta);
            timechain_diag.imu_header_delta_max = std::max(timechain_diag.imu_header_delta_max, delta);
        }
    }

    timechain_diag.last_imu_callback = now;
    timechain_diag.have_imu_callback = true;
    timechain_diag.latest_imu_header = header;
    timechain_diag.have_imu_header = true;
    timechain_diag.imu_callbacks++;
    if (timechain_diag.have_lidar_header)
    {
        timechain_diag.header_difference_max = std::max(
            timechain_diag.header_difference_max,
            std::abs(timechain_diag.latest_lidar_header - timechain_diag.latest_imu_header));
    }
}

void TimechainDiagRecordBufferSizes()
{
    if (!timechain_diag.enabled)
    {
        return;
    }
    timechain_diag.lidar_buffer_max = std::max(timechain_diag.lidar_buffer_max, lidar_buffer.size());
    timechain_diag.imu_buffer_max = std::max(timechain_diag.imu_buffer_max, imu_buffer.size());
    timechain_diag.time_buffer_max = std::max(timechain_diag.time_buffer_max, time_buffer.size());
}
}  // namespace

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;
bool native_map_export = false;
bool forward180 = false;
double native_bottom_height = 0.20;
NativeHeightReference native_reference;
int optimization_stage = 0;
native_filter::Options optimization_options;
native_filter::Stats optimization_stats;
PointCloudXYZI::Ptr raw_forward_observation(new PointCloudXYZI());
std::size_t optimization_forward_points = 0;
std::unique_ptr<native_temporal::TemporalFilter> temporal_filter;
native_temporal::Stats temporal_stats;
PointVector temporal_delete_points;
std::size_t temporal_admitted = 0, temporal_removed = 0;
double temporal_previous_stamp = 0.0;
V3D temporal_previous_origin = V3D::Zero();
Eigen::Quaterniond temporal_previous_rotation = Eigen::Quaterniond::Identity();
double temporal_clear_translation = 0.05, temporal_clear_rotation = 0.10;
bool temporal_allow_clear = false;



bool snapshot_native_map()
{
    if (ikdtree.Root_Node == nullptr) return false;
    // Same native flatten API as the upstream map visualization example.
    // A fresh snapshot replaces the output; there is NO scan-history cache.
    // Invoked on the same single-threaded executor as map_incremental().
    PointVector points;
    if (optimization_stage == 3) ikdtree.Snapshot_Points(points);
    else ikdtree.flatten(ikdtree.Root_Node, points, NOT_RECORD);
    featsFromMap->points.swap(points);
    featsFromMap->width = static_cast<uint32_t>(featsFromMap->points.size());
    featsFromMap->height = 1;
    featsFromMap->is_dense = true;
    return true;
}

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double cur_time = get_time_sec(msg->header.stamp);
    TimechainDiagRecordLidar(cur_time, static_cast<uint64_t>(msg->width) * msg->height);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        if (timechain_diag.enabled)
        {
            timechain_diag.lidar_rollbacks++;
        }
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    TimechainDiagRecordBufferSizes();
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    mtx_buffer.lock();
    double cur_time = get_time_sec(msg->header.stamp);
    TimechainDiagRecordLidar(cur_time, msg->point_num);
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        if (timechain_diag.enabled)
        {
            timechain_diag.lidar_rollbacks++;
        }
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if(is_first_lidar)
    {
        is_first_lidar = false;
        printf("[livox_pcl_cbk] First lidar msg: point_num=%u, time=%.6f\n", msg->point_num, cur_time);
    }
    last_timestamp_lidar = cur_time;

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    // Keep the terminal responsive on embedded computers. Per-scan printing at
    // 10 Hz can delay TF and navigation callbacks when stdout is attached.
    if (scan_count % 100 == 0)
    {
        printf("[livox_pcl_cbk] scan=%d, point_num=%u, processed=%zu\n", scan_count, msg->point_num, ptr->points.size());
    }
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    TimechainDiagRecordBufferSizes();

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();
    TimechainDiagRecordImu(timestamp);

    if (timestamp < last_timestamp_imu)
    {
        if (timechain_diag.enabled)
        {
            timechain_diag.imu_rollbacks++;
        }
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    TimechainDiagRecordBufferSizes();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        if (timechain_diag.enabled)
        {
            timechain_diag.sync_empty_returns++;
        }
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        if (timechain_diag.enabled)
        {
            const auto now = TimechainSteadyClock::now();
            if (!timechain_diag.sync_wait_active)
            {
                timechain_diag.sync_wait_active = true;
                timechain_diag.sync_wait_start = now;
            }
            timechain_diag.sync_wait_returns++;
            timechain_diag.sync_wait_max = std::max(
                timechain_diag.sync_wait_max,
                std::chrono::duration<double>(now - timechain_diag.sync_wait_start).count());
        }
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    if (timechain_diag.enabled)
    {
        const auto now = TimechainSteadyClock::now();
        if (timechain_diag.sync_wait_active)
        {
            timechain_diag.sync_wait_max = std::max(
                timechain_diag.sync_wait_max,
                std::chrono::duration<double>(now - timechain_diag.sync_wait_start).count());
            timechain_diag.sync_wait_active = false;
        }
        timechain_diag.synced_scans++;
        timechain_diag.last_lidar_begin = meas.lidar_beg_time;
        timechain_diag.last_lidar_end = meas.lidar_end_time;
        timechain_diag.last_selected_imu_count = meas.imu.size();
        if (!meas.imu.empty())
        {
            timechain_diag.last_selected_imu_first = get_time_sec(meas.imu.front()->header.stamp);
            timechain_diag.last_selected_imu_last = get_time_sec(meas.imu.back()->header.stamp);
        }
        else
        {
            timechain_diag.last_selected_imu_first = 0.0;
            timechain_diag.last_selected_imu_last = 0.0;
        }
        TimechainDiagRecordBufferSizes();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

native_temporal::Point temporal_point(const PointType &p)
{
    return {p.x, p.y, p.z};
}

void observe_temporal_cloud(bool matched_pose = true)
{
    temporal_admitted = 0;
    temporal_removed = 0;
    temporal_delete_points.clear();
    const V3D origin = state_point.pos + state_point.rot * state_point.offset_T_L_I;
    const Eigen::Quaterniond rotation(state_point.rot.toRotationMatrix() *
        state_point.offset_R_L_I.toRotationMatrix());
    const double dt = lidar_end_time - temporal_previous_stamp;
    double max_gyro = 0.0;
    bool valid_imu = !Measures.imu.empty();
    for (const auto &imu : Measures.imu)
    {
        const V3D omega = V3D(imu->angular_velocity.x, imu->angular_velocity.y,
            imu->angular_velocity.z) - state_point.bg;
        if (!omega.allFinite()) valid_imu = false;
        else max_gyro = std::max(max_gyro, omega.norm());
    }
    temporal_allow_clear = matched_pose && valid_imu &&
        std::isfinite(state_point.vel.norm()) && state_point.vel.norm() <= temporal_clear_translation &&
        max_gyro <= temporal_clear_rotation && temporal_previous_stamp > 0.0 && dt > 0.0 && dt <= 0.5 &&
        (origin - temporal_previous_origin).norm() / dt <= temporal_clear_translation &&
        rotation.angularDistance(temporal_previous_rotation) / dt <= temporal_clear_rotation;
    temporal_previous_stamp = matched_pose ? lidar_end_time : 0.0;
    temporal_previous_origin = origin;
    temporal_previous_rotation = rotation;
    std::vector<native_temporal::Point> raw, candidates;
    raw.reserve(raw_forward_observation->size());
    candidates.reserve(feats_down_body->size());
    PointType world;
    for (const auto &point : raw_forward_observation->points)
    {
        pointBodyToWorld(&point, &world);
        raw.push_back(temporal_point(world));
    }
    for (const auto &point : feats_down_body->points)
    {
        pointBodyToWorld(&point, &world);
        candidates.push_back(temporal_point(world));
    }
    temporal_stats = temporal_filter->observe(raw, candidates,
        {origin.x(), origin.y(), origin.z()}, lidar_end_time, temporal_allow_clear);
    if (temporal_stats.status != native_temporal::Status::ok) return;
    if (temporal_filter->needsRemovalCandidates())
    {
        PointVector snapshot;
        PointType center;
        center.x = origin.x(); center.y = origin.y(); center.z = origin.z();
        ikdtree.Snapshot_Radius(center, static_cast<float>(optimization_options.max_range), snapshot);
        std::vector<native_temporal::Point> actual_points;
        actual_points.reserve(snapshot.size());
        for (const auto &point : snapshot) actual_points.push_back(temporal_point(point));
        for (const auto index : temporal_filter->collectRemovals(actual_points))
            temporal_delete_points.push_back(snapshot.at(index));
    }
}

void interrupt_temporal_frame()
{
    raw_forward_observation->clear();
    feats_down_body->clear();
    observe_temporal_cloud(false);
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        // Preserve native insertion cadence and the matching-neighbor cache.
        // Delaying these points can starve registration of spatial support;
        // temporal evidence is used only to remove witnessed stale points.
        if (optimization_stage == 3) ++temporal_admitted;
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
// A separate observation output, not input to the native estimator/map.
// Use deskewed front points BEFORE the surface voxel filter, with the matched
// end-of-scan optical-origin pose. Existing scan publication remains unchanged.
void publish_free_observation(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub)
{
    if (!native_map_export || !native_reference.ready) return;
    pcl::PointCloud<pcl::PointXYZ> points;
    // Free-space evidence always uses true, unthinned, unsmoothed observations.
    const auto &observation = optimization_stage ? raw_forward_observation : feats_undistort;
    points.reserve(observation->size());
    for (const auto &p : observation->points)
        points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(points, cloud);
    cloud.header.stamp = get_ros_time(lidar_end_time);
    cloud.header.frame_id = "native_scan_lidar";
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = cloud.header.stamp;
    pose.header.frame_id = "camera_init";
    const V3D origin = state_point.pos + state_point.rot * state_point.offset_T_L_I;
    const Eigen::Quaterniond rotation(state_point.rot.toRotationMatrix() *
        state_point.offset_R_L_I.toRotationMatrix());
    pose.pose.position.x = origin.x(); pose.pose.position.y = origin.y();
    pose.pose.position.z = origin.z();
    pose.pose.orientation.x = rotation.x(); pose.pose.orientation.y = rotation.y();
    pose.pose.orientation.z = rotation.z(); pose.pose.orientation.w = rotation.w();
    pose_pub->publish(pose);
    cloud_pub->publish(cloud);
}

void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    /*
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
    */
}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    // Match /cloud_registered semantics: dense_publish_en=false must also
    // downsample the body-frame cloud.  The old implementation always sent
    // feats_undistort here, so the pointcloud-to-laserscan pipeline received a
    // much larger cloud than requested and gradually fell behind real time.
    PointCloudXYZI::Ptr laserCloudBody(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudBody->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&laserCloudBody->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
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
    }
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                            &laserCloudWorld->points[i]);
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudmsg);

    // sensor_msgs::msg::PointCloud2 laserCloudMap;
    // pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    // laserCloudMap.header.frame_id = "camera_init";
    // pubLaserCloudMap->publish(laserCloudMap);
}

void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    if (native_map_export)
    {
        if (!snapshot_native_map()) throw std::runtime_error("Native map is not initialized");
        if (pcd_writer.writeBinary(map_file_path, *featsFromMap) != 0)
            throw std::runtime_error("Could not write native PCD");
    }
    else pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "camera_init";
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = "body";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";
    // A Path has its own header in addition to the individual PoseStamped
    // headers.  Keeping the constructor-time stamp made RViz repeatedly query
    // a transform that had already aged out of the TF cache.
    path.header.stamp = msg_body_pose.header.stamp;

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        timechain_diag.enabled = TimechainDiagEnabledFromEnvironment();
        if (timechain_diag.enabled)
        {
            timechain_diag.ResetWindow(TimechainSteadyClock::now());
            RCLCPP_INFO(
                this->get_logger(),
                "FAST-LIO timechain diagnostics enabled by FAST_LIO_TIMECHAIN_DIAG; emitting one scalar summary per second");
        }

        native_map_export = this->declare_parameter<bool>("experiment.native_map_export", false);
        forward180 = this->declare_parameter<bool>("experiment.forward180", false);
        native_bottom_height = this->declare_parameter<double>("experiment.sensor_bottom_height", 0.20);
        optimization_stage = this->declare_parameter<int>("optimization.stage", 0);
        optimization_options.max_range = this->declare_parameter<double>("optimization.max_range_m", 3.0);
        optimization_options.retention_percent = this->declare_parameter<int>("optimization.retention_percent", 100);
        optimization_options.smoothing = optimization_stage >= 2;
        if (optimization_stage < 0 || optimization_stage > 3 ||
            !std::isfinite(optimization_options.max_range) || optimization_options.max_range <= 0.0 ||
            optimization_options.retention_percent < 1 || optimization_options.retention_percent > 100 ||
            (optimization_stage && (!forward180 || !native_map_export)))
            throw std::invalid_argument("Invalid native mapping optimization parameters");
        if (optimization_stage == 3)
        {
            native_temporal::Options options;
            options.max_range = optimization_options.max_range;
            const int hit_frames = this->declare_parameter<int>("optimization.hit_frames", 5);
            const int free_frames = this->declare_parameter<int>("optimization.free_frames", 4);
            const int clear_rays = this->declare_parameter<int>("optimization.max_clear_rays", 2048);
            if (hit_frames <= 0 || free_frames <= 0 || clear_rays <= 0 || clear_rays > 40000)
                throw std::invalid_argument("Temporal counts and bounded ray budget must be positive");
            options.hit_frames = static_cast<unsigned>(hit_frames);
            options.free_frames = static_cast<unsigned>(free_frames);
            options.max_clear_rays = static_cast<std::size_t>(clear_rays);
            options.hit_seconds = this->declare_parameter<double>("optimization.hit_seconds", 1.0);
            options.free_seconds = this->declare_parameter<double>("optimization.free_seconds", 0.3);
            temporal_clear_translation = this->declare_parameter<double>("optimization.clear_max_translation_m_s", 0.05);
            temporal_clear_rotation = this->declare_parameter<double>("optimization.clear_max_rotation_rad_s", 0.10);
            if (!std::isfinite(temporal_clear_translation) || temporal_clear_translation < 0.0 ||
                !std::isfinite(temporal_clear_rotation) || temporal_clear_rotation < 0.0)
                throw std::invalid_argument("Invalid temporal clearing motion limits");
            temporal_filter.reset(new native_temporal::TemporalFilter(options));
        }
        optimization_metrics_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/fastlio/optimization_metrics", rclcpp::QoS(20).reliable());
        matching_input_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/fastlio/matching_input", rclcpp::SensorDataQoS().keep_last(5));
        native_reference_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/fastlio/native_height_reference", rclcpp::QoS(1).reliable().transient_local());
        free_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/fastlio/free_scan", rclcpp::SensorDataQoS().keep_last(5));
        free_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/fastlio/free_scan_pose", rclcpp::SensorDataQoS().keep_last(5));
        RCLCPP_INFO(this->get_logger(),
            "EXPERIMENT native_ikdtree_export=%d forward180_after_deskew=%d bottom_height=%.3f optimization_stage=%d",
            native_map_export, forward180, native_bottom_height, optimization_stage);
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner",filter_size_corner_min,0.5);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,200.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id ="camera_init";

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(),"w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA)
        {
            // Localization must prefer the newest scan over replaying a
            // growing reliable backlog.  FAST-LIO tolerates an occasional
            // dropped scan and integrates the intervening IMU samples.
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lid_topic, rclcpp::SensorDataQoS().keep_last(2), livox_pcl_cbk);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), imu_cbk);
        // These high-bandwidth point clouds are live sensor products.  A slow
        // visualizer or converter must not make old clouds queue behind the
        // robot's current surroundings.
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", rclcpp::SensorDataQoS().keep_last(1));
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered_body", rclcpp::SensorDataQoS().keep_last(1));
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cloud_effected", rclcpp::SensorDataQoS().keep_last(1));
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/Laser_map", rclcpp::SensorDataQoS().keep_last(1));
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        fout_out.close();
        fout_pre.close();
        fclose(fp);
    }

private:
    void MaybeEmitTimechainDiag()
    {
        if (!timechain_diag.enabled)
        {
            return;
        }

        const auto now = TimechainSteadyClock::now();
        const double window_seconds =
            std::chrono::duration<double>(now - timechain_diag.window_start).count();
        if (window_seconds < 1.0)
        {
            return;
        }

        size_t lidar_buffer_size = 0;
        size_t imu_buffer_size = 0;
        size_t time_buffer_size = 0;
        {
            std::lock_guard<mutex> lock(mtx_buffer);
            lidar_buffer_size = lidar_buffer.size();
            imu_buffer_size = imu_buffer.size();
            time_buffer_size = time_buffer.size();
            TimechainDiagRecordBufferSizes();
        }

        if (timechain_diag.sync_wait_active)
        {
            timechain_diag.sync_wait_max = std::max(
                timechain_diag.sync_wait_max,
                std::chrono::duration<double>(now - timechain_diag.sync_wait_start).count());
        }

        const double latest_header_difference =
            timechain_diag.have_lidar_header && timechain_diag.have_imu_header
                ? timechain_diag.latest_lidar_header - timechain_diag.latest_imu_header
                : 0.0;
        const uint64_t point_count_min =
            timechain_diag.have_point_count ? timechain_diag.point_count_min : 0;
        const uint64_t point_count_max =
            timechain_diag.have_point_count ? timechain_diag.point_count_max : 0;
        const double lidar_header_delta_min =
            timechain_diag.have_lidar_header_delta ? timechain_diag.lidar_header_delta_min : 0.0;
        const double lidar_header_delta_max =
            timechain_diag.have_lidar_header_delta ? timechain_diag.lidar_header_delta_max : 0.0;
        const double imu_header_delta_min =
            timechain_diag.have_imu_header_delta ? timechain_diag.imu_header_delta_min : 0.0;
        const double imu_header_delta_max =
            timechain_diag.have_imu_header_delta ? timechain_diag.imu_header_delta_max : 0.0;

        RCLCPP_INFO(
            this->get_logger(),
            "[TIMECHAIN] window=%.3f cb_hz[lidar=%.2f imu=%.2f] "
            "header[lidar=%.6f imu=%.6f diff=%.6f max_abs=%.6f] "
            "header_dt[lidar=%.6f/%.6f imu=%.6f/%.6f] "
            "cb_gap_max[lidar=%.6f imu=%.6f] points[%llu/%llu] "
            "buffer[cur=%zu/%zu/%zu max=%zu/%zu/%zu] rollback[lidar=%llu imu=%llu] "
            "sync[empty=%llu wait=%llu wait_max=%.6f scans=%llu] "
            "last_scan[beg=%.6f end=%.6f imu_n=%zu imu=%.6f/%.6f] "
            "state[pos_norm=%.6f vel_norm=%.6f effective=%d]",
            window_seconds,
            static_cast<double>(timechain_diag.lidar_callbacks) / window_seconds,
            static_cast<double>(timechain_diag.imu_callbacks) / window_seconds,
            timechain_diag.latest_lidar_header,
            timechain_diag.latest_imu_header,
            latest_header_difference,
            timechain_diag.header_difference_max,
            lidar_header_delta_min,
            lidar_header_delta_max,
            imu_header_delta_min,
            imu_header_delta_max,
            timechain_diag.lidar_callback_gap_max,
            timechain_diag.imu_callback_gap_max,
            static_cast<unsigned long long>(point_count_min),
            static_cast<unsigned long long>(point_count_max),
            lidar_buffer_size,
            imu_buffer_size,
            time_buffer_size,
            timechain_diag.lidar_buffer_max,
            timechain_diag.imu_buffer_max,
            timechain_diag.time_buffer_max,
            static_cast<unsigned long long>(timechain_diag.lidar_rollbacks),
            static_cast<unsigned long long>(timechain_diag.imu_rollbacks),
            static_cast<unsigned long long>(timechain_diag.sync_empty_returns),
            static_cast<unsigned long long>(timechain_diag.sync_wait_returns),
            timechain_diag.sync_wait_max,
            static_cast<unsigned long long>(timechain_diag.synced_scans),
            timechain_diag.last_lidar_begin,
            timechain_diag.last_lidar_end,
            timechain_diag.last_selected_imu_count,
            timechain_diag.last_selected_imu_first,
            timechain_diag.last_selected_imu_last,
            state_point.pos.norm(),
            state_point.vel.norm(),
            effct_feat_num);

        timechain_diag.ResetWindow(now);
        timechain_diag.lidar_buffer_max = lidar_buffer_size;
        timechain_diag.imu_buffer_max = imu_buffer_size;
        timechain_diag.time_buffer_max = time_buffer_size;
    }

    void timer_callback()
    {
        MaybeEmitTimechainDiag();
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            // Process may return without producing a cloud; never re-thin an old frame.
            if (optimization_stage > 0)
            {
                feats_undistort->clear();
                if (Measures.imu.empty())
                {
                    if (optimization_stage == 3) interrupt_temporal_frame();
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Discarding optimization frame without IMU; temporal evidence interrupted");
                    return;
                }
            }
            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                if (optimization_stage == 3) interrupt_temporal_frame();
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            if (native_map_export && !native_reference.ready)
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
                if (feats_undistort->empty())
                {
                    if (optimization_stage == 3) interrupt_temporal_frame();
                    return;
                }
            }

            optimization_forward_points = feats_undistort->size();
            optimization_stats = native_filter::Stats();
            if (optimization_stage > 0)
            {
                *raw_forward_observation = *feats_undistort;
                optimization_stats = native_filter::filter(*feats_undistort,
                    Lidar_R_wrt_IMU, optimization_options);
            }
            if (optimization_stage == 1 || optimization_stage == 2) publish_optimization_diagnostics();
            if (feats_undistort->empty())
            {
                if (optimization_stage == 3)
                {
                    feats_down_body->clear();
                    observe_temporal_cloud(false);
                    publish_optimization_diagnostics();
                }
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if (optimization_stage == 3) observe_temporal_cloud(false);
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    // Preserve native initialization: seed with all current
                    // filtered surface points. A tiny temporally gated seed
                    // can be geometrically degenerate and corrupt the first
                    // ESKF match. Temporal admission applies to later writes.
                    ikdtree.Build(feats_down_world->points);
                    if (optimization_stage == 3) temporal_admitted = feats_down_world->size();
                }
                if (optimization_stage == 3) publish_optimization_diagnostics();
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                if (optimization_stage == 3)
                {
                    observe_temporal_cloud(false);
                    publish_optimization_diagnostics();
                }
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            if (optimization_stage == 3) observe_temporal_cloud(effct_feat_num >= NUM_MATCH_POINTS);
            map_incremental();
            if (optimization_stage == 3)
            {
                // Insert with the existing matching-neighbor cache first.
                // Remove exact old points only after current raw-hit protection.
                if (!temporal_delete_points.empty())
                    temporal_removed = ikdtree.Delete_Exact_Points(temporal_delete_points);
                publish_optimization_diagnostics();
            }
            t5 = omp_get_wtime();
            publish_free_observation(free_scan_pub_, free_pose_pub_);
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath_);
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }
    }

    void publish_optimization_diagnostics()
    {
        double max_range = 0.0;
        std::size_t behind = 0;
        for (const auto &p : feats_undistort->points)
        {
            const V3D xyz(p.x, p.y, p.z);
            max_range = std::max(max_range, xyz.norm());
            if ((Lidar_R_wrt_IMU * xyz).x() < -1e-7) ++behind;
        }
        std::ostringstream out;
        out << std::setprecision(15)
            << "{\"stage\":" << optimization_stage
            << ",\"stamp_ns\":" << get_ros_time(lidar_end_time).nanoseconds()
            << ",\"forward_points\":" << optimization_forward_points
            << ",\"eligible_points\":" << optimization_stats.eligible_count
            << ",\"sampled_points\":" << optimization_stats.sampled_count
            << ",\"matching_input_points\":" << feats_undistort->size()
            << ",\"matching_behind_points\":" << behind
            << ",\"matching_max_range_m\":" << max_range
            << ",\"smoothing_moved\":" << optimization_stats.smoothed_count
            << ",\"smoothing_removed\":" << optimization_stats.outlier_removed
            << ",\"native_admitted\":" << temporal_admitted
            << ",\"native_removed\":" << temporal_removed
            << ",\"temporal_delete_requested\":" << temporal_delete_points.size()
            << ",\"temporal_status\":" << static_cast<int>(temporal_stats.status)
            << ",\"temporal_clear_allowed\":" << (temporal_allow_clear ? "true" : "false")
            << ",\"temporal_clear_rays\":" << temporal_stats.clear_rays
            << ",\"temporal_capacity_drops\":" << temporal_stats.capacity_drops
            << "}";
        std_msgs::msg::String message;
        message.data = out.str();
        optimization_metrics_pub_->publish(message);
        if (matching_input_pub_->get_subscription_count() > 0)
        {
            sensor_msgs::msg::PointCloud2 cloud;
            pcl::toROSMsg(*feats_undistort, cloud);
            cloud.header.stamp = get_ros_time(lidar_end_time);
            cloud.header.frame_id = "native_scan_lidar";
            matching_input_pub_->publish(cloud);
        }
    }

    void map_publish_callback()
    {
        if (native_map_export && native_reference.ready)
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
        if (map_pub_en) publish_map(pubLaserCloudMap_);
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en)
        {
            try
            {
                save_to_pcd();
                res->success = true;
                res->message = "Map saved.";
            }
            catch (const std::exception &e)
            {
                res->success = false;
                res->message = e.what();
            }
        }
        else
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr native_reference_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr free_scan_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr free_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr optimization_metrics_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr matching_input_pub_;
    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    rclcpp::spin(std::make_shared<LaserMappingNode>());

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (native_map_export && pcd_save_en && ikdtree.Root_Node != nullptr)
    {
        try { save_to_pcd(); std::cout << "NATIVE_MAP_SAVED " << map_file_path << std::endl; }
        catch (const std::exception &e) { std::cerr << "NATIVE_MAP_SAVE_FAILED " << e.what() << std::endl; return 1; }
    }
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
