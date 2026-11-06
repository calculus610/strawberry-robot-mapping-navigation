#!/usr/bin/env python3
"""Isolated ROS integration contract: never launch sensors or controllers."""
import argparse
from array import array
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', type=int, required=True, choices=(1, 2, 3))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fault', choices=('stamp','publisher'), default='stamp')
    args = parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 70:
        raise RuntimeError('Set an isolated ROS_DOMAIN_ID >= 70; do not run on the live robot domain.')
    args.output.mkdir(parents=True, exist_ok=True)
    launch_log = (args.output / f'v{args.stage}_{args.fault}_ros_launch.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'my_robot_bringup_fastlio', 'manual_mapping_filtered.launch.py',
        'start_sensors:=false', 'start_rviz:=false', 'enable_map_autosave:=false',
        f'filter_config:=manual_filter_v{args.stage}.yaml',
    ], stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('manual_filter_contract_test')
    metrics, frames, maps = [], [], []
    node.create_subscription(String, '/manual_mapping/filter_metrics', lambda m: metrics.append(json.loads(m.data)), 100)
    node.create_subscription(PointCloud2, '/manual_mapping/filtered_frame', frames.append, qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, '/manual_mapping/map', maps.append,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered_body', qos_profile_sensor_data)
    origin_pub = node.create_publisher(PoseStamped, '/manual_mapping/lidar_origin_body', qos_profile_sensor_data)
    tf_pub = TransformBroadcaster(node)
    fields = [PointField(name=n, offset=i*4, datatype=7, count=1) for i,n in enumerate(('x','y','z'))]
    y,z = np.meshgrid(np.linspace(-.5,.5,40), np.linspace(0,.4,25))
    near = np.column_stack((np.full(1000,1.5),y.ravel(),z.ravel()))
    points = np.concatenate((near, np.array([[4.,0.,0.]]*200), [[np.nan,0.,0.]] )).astype('<f4')
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
    def publish(stamp, origin=True):
        transform = TransformStamped(header=Header(stamp=stamp,frame_id='camera_init'),child_frame_id='body')
        transform.transform.translation.x=10.
        transform.transform.translation.y=-2.
        transform.transform.rotation.w=1.
        tf_pub.sendTransform(transform)
        pose = PoseStamped(header=Header(stamp=stamp,frame_id='body'))
        pose.pose.position.x=.05
        pose.pose.position.z=.04
        pose.pose.orientation.w=1.
        if origin: origin_pub.publish(pose)
        cloud_pub.publish(PointCloud2(header=pose.header,height=1,width=len(points),fields=fields,
            is_bigendian=False,point_step=12,row_step=len(points)*12,data=array('B',points.tobytes()),is_dense=False))
    try:
        spin(4.)
        assert process.poll() is None, 'launch exited before test'
        for _ in range(25):
            last_stamp=node.get_clock().now().to_msg()
            publish(last_stamp)
            spin(.11)
        spin(1.2)
        ok=[m for m in metrics if m.get('status')=='processed']
        assert len(ok)>=15, (len(ok),metrics[-3:])
        assert all(m['counts']['in_range']==1000 and m['counts']['sampled']==700 for m in ok)
        assert all(m['counts']['nonfinite']==1 and m['counts']['outside_range']==200 for m in ok)
        assert all(m['filtered_range_max_m'] <=3. for m in ok)
        assert frames and maps and any(100 in m.data for m in maps), 'cloud/map missing'
        xyz=np.frombuffer(frames[-1].data,dtype='<f4').reshape(-1,3)
        assert np.max(np.linalg.norm(xyz-np.array([10.05,-2.,.04]),axis=1))<=3.+1e-5
        assert frames[-1].header.frame_id=='camera_init'
        assert all(m.info.resolution>0 and m.header.frame_id=='camera_init' for m in maps)
        # A missing origin must be dropped, never paired with a nearby time.
        publish(node.get_clock().now().to_msg(),origin=False)
        spin(1.3)
        assert any(m.get('reason')=='origin_timeout' for m in metrics)
        if args.fault == 'stamp':
            # A restarted clock must not append a new coordinate epoch to this map.
            publish(last_stamp)
            spin(.2)
            publish(node.get_clock().now().to_msg())
            spin(.2)
            assert any(m.get('reason')=='stamp_regression' and m.get('requires_restart') for m in metrics)
        else:
            # A new publisher can use increasing time but a different world origin.
            node.destroy_publisher(cloud_pub)
            spin(.4)
            cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered_body', qos_profile_sensor_data)
            spin(.8)
            publish(node.get_clock().now().to_msg())
            spin(.3)
            assert any(m.get('reason')=='cloud_publisher_changed' and m.get('requires_restart') for m in metrics), metrics[-5:]
            publish(node.get_clock().now().to_msg())
            spin(.2)
        assert any(m.get('reason')=='restart_required' for m in metrics)
        result={'stage':args.stage,'passed':True,'processed_frames':len(ok),
            'points_in_range':1000,'sampled_each_frame':700,'projected_maps':len(maps),
            'map_voxels':ok[-1]['map_voxels'],'processing_p95_ms':float(np.percentile([m['timing_ms']['total_processing'] for m in ok],95)),
            'fault':args.fault,
            'checked':['exact_sampling','radius_at_translated_origin','nan_and_far_rejection','tf_stamp',
                       'cloud_and_occupancy_grid','missing_origin_timeout',args.fault+'_restart_latch']}
        (args.output/f'v{args.stage}_{args.fault}_ros_contract.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                process.wait(timeout=5)
        launch_log.close()

if __name__=='__main__': main()
