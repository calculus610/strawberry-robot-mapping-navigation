#!/usr/bin/env python3
"""Capture live sensors in an isolated DDS domain, without mapper or motion nodes."""
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

ROOT=Path('/home/wheeltec/manual_native180_backups/20261109_v1_3m/fresh_static_scene')
DURATION=90.0

def stop(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM)
        process.wait(timeout=10)

def main():
    assert os.environ['ROS_DOMAIN_ID']=='83' and os.environ['ROS_LOCALHOST_ONLY']=='1'
    ROOT.mkdir(exist_ok=False)
    report={'purpose':'current forward180 scene reported static by user; full raw lidar+imu captured for replay',
            'record_seconds_requested':DURATION,'motion_commands_sent':False,'mapper_started':False}
    driver=recorder=None
    import rclpy
    rclpy.init()
    node=rclpy.create_node('native180_stationary_capture_probe')
    try:
        with (ROOT/'driver.log').open('w') as driver_log, (ROOT/'recorder.log').open('w') as record_log:
            driver=subprocess.Popen(['ros2','launch','livox_ros_driver2','msg_MID360_launch.py'],stdout=driver_log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+25
            while time.monotonic()<deadline:
                if driver.poll() is not None: raise RuntimeError('LiDAR launch exited early')
                if all(node.count_publishers(topic)==1 for topic in ['/livox/lidar','/livox/imu']): break
                rclpy.spin_once(node,timeout_sec=0.1)
            else: raise RuntimeError('LiDAR/IMU publishers were not discovered')
            # Allow driver startup before recording both streams together.
            deadline=time.monotonic()+5
            while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=0.1)
            recorder=subprocess.Popen(['ros2','bag','record','--storage','sqlite3','-o',str(ROOT/'bag'),'/livox/lidar','/livox/imu'],stdout=record_log,stderr=subprocess.STDOUT,start_new_session=True)
            started=time.time(); deadline=time.monotonic()+DURATION
            report['record_start_unix_s']=started
            print('CAPTURE_RECORDING_90_SECONDS',flush=True)
            while time.monotonic()<deadline:
                if driver.poll() is not None or recorder.poll() is not None: raise RuntimeError('Capture process exited early')
                rclpy.spin_once(node,timeout_sec=0.1)
            stop(recorder); stop(driver)
            report['record_end_unix_s']=time.time()
            report['recorder_returncode']=recorder.returncode; report['driver_returncode']=driver.returncode
        database=next((ROOT/'bag').glob('*.db3'))
        with sqlite3.connect(f'file:{database}?mode=ro',uri=True) as connection:
            report['topics']=[{'name':name,'type':typ,'messages':count,'first_record_ns':first,'last_record_ns':last}
                for name,typ,count,first,last in connection.execute('SELECT t.name,t.type,count(m.id),min(m.timestamp),max(m.timestamp) FROM topics t LEFT JOIN messages m ON m.topic_id=t.id GROUP BY t.id')]
        assert {t['name'] for t in report['topics']}=={'/livox/lidar','/livox/imu'}
        for t in report['topics']:
            assert t['messages']>(700 if t['name']=='/livox/lidar' else 12000),t
        report['database']=str(database); report['status']='captured'
    except Exception as error:
        report['status']='failed'; report['error']=str(error)
        raise
    finally:
        stop(recorder); stop(driver)
        node.destroy_node(); rclpy.shutdown()
        (ROOT/'capture_report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report),flush=True)

if __name__=='__main__': main()
