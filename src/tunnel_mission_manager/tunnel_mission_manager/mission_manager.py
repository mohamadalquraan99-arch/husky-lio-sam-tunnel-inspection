import json
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from lio_sam.srv import SaveMap
from slam_toolbox.srv import SerializePoseGraph


class TunnelMissionManager(Node):
    def __init__(self):
        super().__init__('tunnel_mission_manager')

        self.mission_mode = self.declare_parameter(
            'mission_mode', 'inspection').value
        self.data_root = Path(self.declare_parameter(
            'data_root', '/home/momo1/husky_ws/inspection_data').value).expanduser()
        self.baseline_id = self.declare_parameter(
            'baseline_id', 'baseline_01').value
        self.mission_id = self.declare_parameter(
            'mission_id', 'mission_03').value
        self.map_resolution = float(self.declare_parameter(
            'map_resolution', 0.1).value)
        self.overwrite_output = bool(self.declare_parameter(
            'overwrite_output', False).value)
        self.mission_status_topic = self.declare_parameter(
            'mission_status_topic', '/exploration/mission_status').value
        self.detection_trigger_topic = self.declare_parameter(
            'detection_trigger_topic', '/inspection/run_detection').value

        if self.mission_mode not in ('baseline', 'inspection'):
            raise RuntimeError("mission_mode must be 'baseline' or 'inspection'")

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.status_sub = self.create_subscription(
            String,
            self.mission_status_topic,
            self._mission_status_callback,
            latched_qos)
        self.detection_trigger_pub = self.create_publisher(
            String, self.detection_trigger_topic, latched_qos)
        self.system_status_pub = self.create_publisher(
            String, '/inspection/system_status', latched_qos)

        self.lio_save_client = self.create_client(SaveMap, '/lio_sam/save_map')
        self.posegraph_client = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map')

        self.finalization_started = False
        self.seen_active_mission = False
        self._publish_status('WAITING_FOR_MISSION_COMPLETION')
        self.get_logger().info(
            f"Mission manager ready: mode={self.mission_mode}, "
            f"baseline={self.baseline_id}, mission={self.mission_id}")

    def _publish_status(self, value):
        message = String()
        message.data = value
        self.system_status_pub.publish(message)
        self.get_logger().info(f'System status: {value}')

    def _mission_status_callback(self, message):
        self.get_logger().info(f'Explorer status received: {message.data}')
        if message.data in ('EXPLORING', 'RETURNING_HOME', 'RETURN_HOME_RETRY'):
            self.seen_active_mission = True
        if message.data not in ('HOME_REACHED', 'MISSION_COMPLETE'):
            return
        if not self.seen_active_mission:
            self.get_logger().warning(
                'Ignoring completion status because this manager has not observed '
                'an active mission; it may be stale discovery data.')
            return
        if self.finalization_started:
            return

        self.finalization_started = True
        worker = threading.Thread(target=self._finalize_mission, daemon=True)
        worker.start()

    def _output_directory(self):
        identifier = self.baseline_id if self.mission_mode == 'baseline' else self.mission_id
        return self.data_root / identifier

    def _ensure_safe_output(self, output_directory):
        global_map = output_directory / 'lio_sam_3d' / 'GlobalMap.pcd'
        if global_map.exists() and not self.overwrite_output:
            raise RuntimeError(
                f"Refusing to overwrite existing mission data: {global_map}. "
                "Use a new mission_id or explicitly set overwrite_output:=true.")
        output_directory.mkdir(parents=True, exist_ok=True)

    def _save_2d_map(self, output_directory):
        map_prefix = output_directory / 'tunnel_2d_map'
        command = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', str(map_prefix),
        ]
        result = subprocess.run(
            command, check=False, capture_output=True, text=True, timeout=90)
        if result.returncode != 0:
            raise RuntimeError(
                '2D map saving failed: ' + (result.stderr or result.stdout))
        self.get_logger().info(f'2D map saved to {map_prefix}')

    @staticmethod
    def _wait_for_future(future, timeout_seconds):
        deadline = time.monotonic() + timeout_seconds
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError('ROS service call timed out')
            time.sleep(0.1)
        if not future.done():
            raise RuntimeError('ROS shutdown interrupted the service call')
        if future.exception() is not None:
            raise future.exception()
        return future.result()

    def _save_lio_map(self, output_directory):
        if not self.lio_save_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError('/lio_sam/save_map is unavailable')

        lio_directory = output_directory / 'lio_sam_3d'
        home = Path.home().resolve()
        resolved_lio_directory = lio_directory.resolve()
        try:
            relative = resolved_lio_directory.relative_to(home)
        except ValueError as error:
            raise RuntimeError('LIO-SAM output must be located below the home directory') from error

        request = SaveMap.Request()
        request.resolution = self.map_resolution
        request.destination = '/' + relative.as_posix().strip('/') + '/'
        response = self._wait_for_future(
            self.lio_save_client.call_async(request), 300.0)
        if not response.success:
            raise RuntimeError('LIO-SAM returned success=false while saving the map')

        global_map = lio_directory / 'GlobalMap.pcd'
        if not global_map.exists():
            raise RuntimeError(f'LIO-SAM reported success, but {global_map} is missing')
        self.get_logger().info(f'3D map saved to {global_map}')
        return global_map

    def _save_posegraph(self, output_directory):
        if not self.posegraph_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError('/slam_toolbox/serialize_map is unavailable')

        request = SerializePoseGraph.Request()
        request.filename = str(output_directory / 'tunnel_posegraph')
        response = self._wait_for_future(
            self.posegraph_client.call_async(request), 180.0)
        self.get_logger().info(
            f'Pose graph serialization returned result={response.result}')

    def _write_metadata(self, output_directory, global_map):
        metadata = {
            'mission_mode': self.mission_mode,
            'baseline_id': self.baseline_id,
            'mission_id': self.mission_id,
            'map_resolution_m': self.map_resolution,
            'global_map': str(global_map),
            'saved_unix_time': time.time(),
        }
        (output_directory / 'mission_metadata.json').write_text(
            json.dumps(metadata, indent=2) + '\n', encoding='utf-8')

    def _finalize_mission(self):
        try:
            output_directory = self._output_directory()
            self._ensure_safe_output(output_directory)
            self._publish_status('SAVING_MISSION_DATA')

            self._save_2d_map(output_directory)
            global_map = self._save_lio_map(output_directory)
            self._save_posegraph(output_directory)
            self._write_metadata(output_directory, global_map)

            if self.mission_mode == 'inspection':
                trigger = String()
                trigger.data = str(global_map)
                self.detection_trigger_pub.publish(trigger)
                self._publish_status('ANOMALY_DETECTION_TRIGGERED')
            else:
                self._publish_status('BASELINE_SAVED')
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Mission finalization failed: {error}')
            self._publish_status('MISSION_FINALIZATION_FAILED')


def main(args=None):
    rclpy.init(args=args)
    node = TunnelMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
