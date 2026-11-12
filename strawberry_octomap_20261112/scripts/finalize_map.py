#!/usr/bin/env python3
"""Ask this OctoMap session to freeze input and finish saving."""
import argparse
import math
import time

import rclpy
from std_srvs.srv import Trigger

SERVICE = '/strawberry_octomap/finalize'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be positive and finite')

    deadline = time.monotonic() + args.timeout
    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node('strawberry_octomap_finalize_client')
        client = node.create_client(Trigger, SERVICE)
        remaining = max(0.0, deadline - time.monotonic())
        if not client.wait_for_service(timeout_sec=remaining):
            print('FINALIZE_FAILED: 服务不可用', flush=True)
            return 1
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.0, deadline - time.monotonic()))
        if not future.done():
            print('FINALIZE_FAILED: 保存服务超时', flush=True)
            return 1
        response = future.result()
        print(f'{"FINALIZE_COMPLETE" if response.success else "FINALIZE_FAILED"}: {response.message}', flush=True)
        return 0 if response.success else 1
    except Exception as error:
        print(f'FINALIZE_FAILED: {error}', flush=True)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
