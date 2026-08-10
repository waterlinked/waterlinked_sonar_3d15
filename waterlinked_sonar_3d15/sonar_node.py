# Copyright 2025 Julian Valdez
#
# Licensed under the MIT License.

"""ROS 2 driver node for the Water Linked Sonar 3D-15.

Built on the official ``wlsonar`` Python library (https://github.com/waterlinked/wlsonar).
Receives Range Image Protocol (RIP2) packets over UDP and publishes PointCloud2,
depth images, and intensity images.
"""

import math
import socket
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import wlsonar
import wlsonar.range_image_protocol as rip

RIP_IMU_BATCH_TYPE = getattr(rip, 'ImuBatch', None)


def _proto_timestamp_to_ros_time(proto_ts, fallback_stamp):
    """Convert protobuf Timestamp to ROS2 builtin time message.

    RIP message timestamps use protobuf (seconds, nanos). ROS2 expects
    builtin_interfaces/msg/Time (sec, nanosec).
    """
    if proto_ts is None:
        return fallback_stamp

    try:
        sec = int(proto_ts.seconds)
        nanos = int(proto_ts.nanos)
    except Exception:
        return fallback_stamp

    # Normalize nanoseconds to [0, 1e9).
    sec += nanos // 1_000_000_000
    nanos = nanos % 1_000_000_000

    stamp = fallback_stamp
    stamp.sec = sec
    stamp.nanosec = nanos
    return stamp


class SonarNode(Node):
    """Water Linked Sonar 3D-15 driver node."""

    def __init__(self):
        super().__init__('sonar_node')

        self._declare_parameters()

        self._sonar: Optional[wlsonar.Sonar3D] = None
        self._udp_sock = None
        self._recv_thread: Optional[threading.Thread] = None
        self._running = False

        # Packet statistics (written by recv thread, read by heartbeat timer)
        self._lock = threading.Lock()
        self._stats_udp_packets = 0
        self._stats_range_images = 0
        self._stats_bitmap_images = 0
        self._stats_imu_batches = 0
        self._stats_unknown_packets = 0
        self._stats_decode_errors = 0
        self._stats_timeouts = 0
        self._stats_last_seq_id = -1
        self._stats_start_time = time.monotonic()

        self._pub_point_cloud = self.create_publisher(
            PointCloud2,
            self.get_parameter('topic_point_cloud').get_parameter_value().string_value, 10)
        self._pub_range_image = self.create_publisher(
            Image,
            self.get_parameter('topic_range_image').get_parameter_value().string_value, 10)
        self._pub_intensity_image = self.create_publisher(
            Image,
            self.get_parameter('topic_intensity_image').get_parameter_value().string_value, 10)
        self._pub_camera_info = self.create_publisher(
            CameraInfo,
            self.get_parameter('topic_camera_info').get_parameter_value().string_value, 10)
        self._pub_imu = self.create_publisher(
            Imu,
            self.get_parameter('topic_imu').get_parameter_value().string_value, 10)
        self._pub_diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        diag_period = self.get_parameter('diagnostics_period').get_parameter_value().double_value
        self._diag_timer = self.create_timer(diag_period, self._diagnostics_callback)
        self._heartbeat_timer = self.create_timer(1.0, self._heartbeat_callback)

        self.add_on_set_parameters_callback(self._on_parameter_change)

        self._connect_and_configure()

    # ──────────────────────────────────────────────────────────────────────
    # Parameters
    # ──────────────────────────────────────────────────────────────────────

    def _declare_parameters(self):
        self.declare_parameter('sonar_ip', '192.168.194.96', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='IP address of the Sonar 3D-15'))
        self.declare_parameter('frame_id', 'sonar_link', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='TF frame ID for published messages'))
        self.declare_parameter('acoustics_enabled', True, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Enable acoustic imaging on startup'))
        self.declare_parameter('speed_of_sound', 1480.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Speed of sound in m/s'))
        self.declare_parameter('mode', 'low-frequency', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Imaging mode: "low-frequency" or "high-frequency" (firmware >= 1.7.0)'))
        self.declare_parameter('salinity', 'salt', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Water salinity: "salt" or "fresh" (firmware >= 1.7.0)'))
        self.declare_parameter('range_min', 0.3, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Minimum imaging range in meters'))
        self.declare_parameter('range_max', 15.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Maximum imaging range in meters'))
        self.declare_parameter('udp_mode', 'multicast', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='UDP mode: "multicast" or "unicast"'))
        self.declare_parameter('unicast_destination_ip', '', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Unicast destination IP (only used when udp_mode is "unicast")'))
        self.declare_parameter('unicast_destination_port', 0, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Unicast destination port (only used when udp_mode is "unicast")'))
        self.declare_parameter('interface_ip', '0.0.0.0', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Local IP of the network interface to use for multicast/unicast bind. '
                        'Set to the IP on the same subnet as the sonar on multi-homed machines.'))
        self.declare_parameter('diagnostics_period', 5.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Period in seconds between diagnostic queries'))
        self.declare_parameter('imu_output_enabled', True, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Enable IMU batch output from sonar (requires firmware >= 1.8.0)'))

        self.declare_parameter('topic_point_cloud', '~/point_cloud', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Topic name for PointCloud2 output'))
        self.declare_parameter('topic_range_image', '~/range_image', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Topic name for range image output'))
        self.declare_parameter('topic_intensity_image', '~/intensity_image', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Topic name for intensity image output'))
        self.declare_parameter('topic_camera_info', '~/camera_info', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Topic name for CameraInfo output'))
        self.declare_parameter('topic_imu', '~/imu', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Topic name for IMU output (sensor_msgs/Imu)'))

    def _on_parameter_change(self, params: list[Parameter]) -> SetParametersResult:
        for param in params:
            try:
                if param.name == 'acoustics_enabled' and self._sonar:
                    self._sonar.set_acoustics_enabled(param.value)
                    self.get_logger().info(f'Acoustics {"enabled" if param.value else "disabled"}')
                elif param.name == 'speed_of_sound' and self._sonar:
                    self._sonar.set_speed_of_sound(param.value)
                    self.get_logger().info(f'Speed of sound set to {param.value} m/s')
                elif param.name == 'mode' and self._sonar:
                    self._sonar.set_mode(param.value)
                    self.get_logger().info(f'Mode set to {param.value}')
                elif param.name == 'salinity' and self._sonar:
                    self._sonar.set_salinity(param.value)
                    self.get_logger().info(f'Salinity set to {param.value}')
                elif param.name in ('range_min', 'range_max') and self._sonar:
                    rmin = self.get_parameter('range_min').get_parameter_value().double_value
                    rmax = self.get_parameter('range_max').get_parameter_value().double_value
                    if param.name == 'range_min':
                        rmin = param.value
                    else:
                        rmax = param.value
                    self._sonar.set_range(rmin, rmax)
                    self.get_logger().info(f'Range set to [{rmin}, {rmax}] m')
                elif param.name == 'imu_output_enabled' and self._sonar:
                    if self._set_imu_output_enabled(param.value):
                        self.get_logger().info(
                            f'IMU output {"enabled" if param.value else "disabled"}')
            except wlsonar.VersionException as e:
                self.get_logger().warn(str(e))
            except Exception as e:
                self.get_logger().error(f'Failed to apply parameter {param.name}: {e}')
                return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    # ──────────────────────────────────────────────────────────────────────
    # Connection and configuration
    # ──────────────────────────────────────────────────────────────────────

    def _connect_and_configure(self):
        ip = self.get_parameter('sonar_ip').get_parameter_value().string_value
        self.get_logger().info(f'Connecting to Sonar 3D-15 at {ip}...')

        try:
            self._sonar = wlsonar.Sonar3D(ip)
        except Exception as e:
            self.get_logger().error(f'Failed to connect to sonar at {ip}: {e}')
            self.get_logger().error('Node will not publish data. Check IP and network.')
            return

        about = self._sonar.about()
        self.get_logger().info(
            f'Connected: {about.product_name} '
            f'(chipid={about.chipid}, fw={about.version_short})')

        self._apply_initial_configuration()
        self._open_udp_and_start_receiver()

    def _apply_initial_configuration(self):
        if self._sonar is None:
            return

        sos = self.get_parameter('speed_of_sound').get_parameter_value().double_value
        acoustics = self.get_parameter('acoustics_enabled').get_parameter_value().bool_value
        mode = self.get_parameter('mode').get_parameter_value().string_value
        salinity = self.get_parameter('salinity').get_parameter_value().string_value
        rmin = self.get_parameter('range_min').get_parameter_value().double_value
        rmax = self.get_parameter('range_max').get_parameter_value().double_value
        udp_mode = self.get_parameter('udp_mode').get_parameter_value().string_value
        imu_output_enabled = self.get_parameter('imu_output_enabled').get_parameter_value().bool_value

        try:
            self._sonar.set_speed_of_sound(sos)
            self.get_logger().info(f'Speed of sound: {sos} m/s')
        except Exception as e:
            self.get_logger().warn(f'Could not set speed of sound: {e}')

        try:
            self._sonar.set_range(rmin, rmax)
            self.get_logger().info(f'Range: [{rmin}, {rmax}] m')
        except Exception as e:
            self.get_logger().warn(f'Could not set range: {e}')

        try:
            self._sonar.set_mode(mode)
            self.get_logger().info(f'Mode: {mode}')
        except wlsonar.VersionException:
            self.get_logger().warn(
                'Firmware too old for mode setting (requires >= 1.7.0). Skipping.')
        except Exception as e:
            self.get_logger().warn(f'Could not set mode: {e}')

        try:
            self._sonar.set_salinity(salinity)
            self.get_logger().info(f'Salinity: {salinity}')
        except wlsonar.VersionException:
            self.get_logger().warn(
                'Firmware too old for salinity setting (requires >= 1.7.0). Skipping.')
        except Exception as e:
            self.get_logger().warn(f'Could not set salinity: {e}')

        try:
            self._sonar.set_acoustics_enabled(acoustics)
            self.get_logger().info(f'Acoustics: {"enabled" if acoustics else "disabled"}')
        except Exception as e:
            self.get_logger().warn(f'Could not set acoustics: {e}')

        try:
            if udp_mode == 'multicast':
                self._sonar.set_udp_multicast()
                self.get_logger().info('UDP: multicast')
            elif udp_mode == 'unicast':
                uip = self.get_parameter(
                    'unicast_destination_ip').get_parameter_value().string_value
                uport = self.get_parameter(
                    'unicast_destination_port').get_parameter_value().integer_value
                self._sonar.set_udp_unicast(uip, uport)
                self.get_logger().info(f'UDP: unicast → {uip}:{uport}')
            else:
                self.get_logger().warn(f'Unknown udp_mode "{udp_mode}", defaulting to multicast')
                self._sonar.set_udp_multicast()
        except Exception as e:
            self.get_logger().error(f'Could not configure UDP output: {e}')

        if self._set_imu_output_enabled(imu_output_enabled):
            self.get_logger().info(
                f'IMU batch output: {"enabled" if imu_output_enabled else "disabled"}')

    # ──────────────────────────────────────────────────────────────────────
    # UDP receiver
    # ──────────────────────────────────────────────────────────────────────

    def _open_udp_and_start_receiver(self):
        udp_mode = self.get_parameter('udp_mode').get_parameter_value().string_value
        iface_ip = self.get_parameter('interface_ip').get_parameter_value().string_value

        try:
            if udp_mode == 'unicast':
                uport = self.get_parameter(
                    'unicast_destination_port').get_parameter_value().integer_value
                self._udp_sock = wlsonar.open_sonar_udp_unicast_socket(
                    udp_port=uport, iface_ip=iface_ip)
                self.get_logger().info(
                    f'UDP unicast socket listening on {iface_ip}:{uport}')
            else:
                self._udp_sock = wlsonar.open_sonar_udp_multicast_socket(
                    iface_ip=iface_ip)
                self.get_logger().info(
                    f'UDP multicast socket joined {wlsonar.DEFAULT_MCAST_GRP}'
                    f':{wlsonar.DEFAULT_MCAST_PORT} on interface {iface_ip}')
        except Exception as e:
            self.get_logger().error(f'Failed to open UDP socket: {e}')
            return

        self._udp_sock.settimeout(2.0)
        self._running = True
        self._recv_thread = threading.Thread(target=self._udp_receive_loop, daemon=True)
        self._recv_thread.start()
        self.get_logger().info('UDP receiver thread started')

    def _udp_receive_loop(self):
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        local_pkt_count = 0
        local_timeout_count = 0

        while self._running and rclpy.ok():
            try:
                data, addr = self._udp_sock.recvfrom(wlsonar.UDP_MAX_DATAGRAM_SIZE)
            except (TimeoutError, socket.timeout):
                local_timeout_count += 1
                with self._lock:
                    self._stats_timeouts = local_timeout_count
                if local_timeout_count % 5 == 1:
                    self.get_logger().warn(
                        f'No UDP packets received (timeouts: {local_timeout_count}, '
                        f'packets so far: {local_pkt_count})')
                continue
            except OSError as e:
                if self._running:
                    self.get_logger().error(f'UDP socket error: {e}')
                break

            local_pkt_count += 1
            with self._lock:
                self._stats_udp_packets = local_pkt_count
            if local_pkt_count == 1:
                self.get_logger().info(
                    f'First UDP packet from {addr[0]}:{addr[1]} ({len(data)} bytes)')

            try:
                msg = rip.unpackb(data)
            except rip.UnknownProtobufTypeError:
                with self._lock:
                    self._stats_unknown_packets += 1
                    count = self._stats_unknown_packets
                if count <= 3 or count % 100 == 0:
                    self.get_logger().debug(
                        f'Unknown protobuf type (count: {count}, '
                        f'size: {len(data)} bytes)')
                continue
            except (rip.CRCMismatchError, rip.BadIDError, rip.ExtraDataError) as e:
                with self._lock:
                    self._stats_decode_errors += 1
                self.get_logger().warn(f'Packet decode error: {e}')
                continue
            except Exception as e:
                with self._lock:
                    self._stats_decode_errors += 1
                self.get_logger().warn(f'Unexpected decode error: {type(e).__name__}: {e}')
                continue

            msg_header = getattr(msg, 'header', None)
            msg_timestamp = getattr(msg_header, 'timestamp', None)
            stamp = _proto_timestamp_to_ros_time(msg_timestamp, self.get_clock().now().to_msg())
            header = Header(stamp=stamp, frame_id=frame_id)

            if isinstance(msg, rip.RangeImage):
                with self._lock:
                    self._stats_range_images += 1
                    self._stats_last_seq_id = msg.header.sequence_id
                    ri_count = self._stats_range_images
                if ri_count <= 3:
                    self.get_logger().info(
                        f'RangeImage: {msg.width}x{msg.height}, '
                        f'freq={msg.frequency}Hz, '
                        f'seq={msg.header.sequence_id}')
                self._publish_camera_info(msg, header)
                self._publish_range_image(msg, header)
                self._publish_point_cloud(msg, header)
            elif isinstance(msg, rip.BitmapImageGreyscale8):
                with self._lock:
                    self._stats_bitmap_images += 1
                    self._stats_last_seq_id = msg.header.sequence_id
                    bmp_count = self._stats_bitmap_images
                if bmp_count <= 3:
                    self.get_logger().info(
                        f'BitmapImage: {msg.width}x{msg.height}, '
                        f'freq={msg.frequency}Hz, '
                        f'seq={msg.header.sequence_id}')
                self._publish_camera_info(msg, header)
                self._publish_intensity_image(msg, header)
            elif RIP_IMU_BATCH_TYPE is not None and isinstance(msg, rip.ImuBatch):
                with self._lock:
                    self._stats_imu_batches += 1
                    imu_count = self._stats_imu_batches
                if imu_count <= 3:
                    self.get_logger().info(
                        f'ImuBatch: samples={msg.samples}, '
                        f'batch_seq={msg.batch_sequence_id}')
                self._publish_imu_batch(msg, frame_id)

    # ──────────────────────────────────────────────────────────────────────
    # Publishers
    # ──────────────────────────────────────────────────────────────────────

    def _publish_range_image(self, msg: rip.RangeImage, header: Header):
        if self._pub_range_image.get_subscription_count() == 0:
            return

        distances = wlsonar.range_image_to_distance(msg)
        arr = np.array(distances, dtype=np.float32).reshape((msg.height, msg.width))

        img = Image()
        img.header = header
        img.height = msg.height
        img.width = msg.width
        img.encoding = '32FC1'
        img.is_bigendian = False
        img.step = msg.width * 4
        img.data = arr.tobytes()

        self._pub_range_image.publish(img)

    def _publish_intensity_image(self, msg: rip.BitmapImageGreyscale8, header: Header):
        if self._pub_intensity_image.get_subscription_count() == 0:
            return

        pixels = wlsonar.bitmap_image_to_strength_log(msg)
        arr = np.array(pixels, dtype=np.uint8).reshape((msg.height, msg.width))

        img = Image()
        img.header = header
        img.height = msg.height
        img.width = msg.width
        img.encoding = '8UC1'
        img.is_bigendian = False
        img.step = msg.width
        img.data = arr.tobytes()

        self._pub_intensity_image.publish(img)

    def _publish_point_cloud(self, msg: rip.RangeImage, header: Header):
        if self._pub_point_cloud.get_subscription_count() == 0:
            return

        voxels = wlsonar.range_image_to_xyz(msg)

        points = []
        for v in voxels:
            if v is not None:
                points.append(v)

        if not points:
            return

        arr = np.array(points, dtype=np.float32)

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(points)
        cloud.data = arr.tobytes()
        cloud.is_dense = True

        self._pub_point_cloud.publish(cloud)

    def _publish_camera_info(self, msg, header: Header):
        """Publish sonar intrinsics as CameraInfo alongside each image.

        Encodes the sonar's angular FOV as a pinhole-equivalent projection so
        that standard ROS tools can relate pixel coordinates to bearing angles.
        The K matrix maps (azimuth_px, elevation_px) → bearing in the same way
        a pinhole camera maps (u, v) → ray direction.
        """
        if self._pub_camera_info.get_subscription_count() == 0:
            return

        fov_h_rad = math.radians(msg.fov_horizontal)
        fov_v_rad = math.radians(msg.fov_vertical)

        # Pinhole-equivalent focal lengths (pixels)
        fx = (msg.width / 2.0) / math.tan(fov_h_rad / 2.0) if fov_h_rad > 0 else 0.0
        fy = (msg.height / 2.0) / math.tan(fov_v_rad / 2.0) if fov_v_rad > 0 else 0.0
        cx = msg.width / 2.0
        cy = msg.height / 2.0

        ci = CameraInfo()
        ci.header = header
        ci.width = msg.width
        ci.height = msg.height
        ci.distortion_model = 'none'
        ci.d = []
        ci.k = [fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0]

        self._pub_camera_info.publish(ci)

    def _publish_imu_batch(self, msg, frame_id: str):
        if self._pub_imu.get_subscription_count() == 0:
            return

        if len(msg.timestamp) != msg.samples:
            self.get_logger().warn('Invalid ImuBatch: len(timestamp) != samples')
            return
        if len(msg.specific_force) != msg.samples * 3:
            self.get_logger().warn('Invalid ImuBatch: len(specific_force) != samples*3')
            return
        if len(msg.rate_of_turn) != msg.samples * 3:
            self.get_logger().warn('Invalid ImuBatch: len(rate_of_turn) != samples*3')
            return

        for i in range(msg.samples):
            idx = i * 3

            imu_msg = Imu()
            # Timestamps are provided in seconds/nanoseconds for each sample
            imu_msg.header.stamp.sec = msg.timestamp[i].seconds
            imu_msg.header.stamp.nanosec = msg.timestamp[i].nanos
            imu_msg.header.frame_id = frame_id

            # Orientation is not provided by ImuBatch, mark as unavailable.
            imu_msg.orientation_covariance[0] = -1.0

            imu_msg.linear_acceleration.x = msg.specific_force[idx]
            imu_msg.linear_acceleration.y = msg.specific_force[idx + 1]
            imu_msg.linear_acceleration.z = msg.specific_force[idx + 2]

            imu_msg.angular_velocity.x = msg.rate_of_turn[idx]
            imu_msg.angular_velocity.y = msg.rate_of_turn[idx + 1]
            imu_msg.angular_velocity.z = msg.rate_of_turn[idx + 2]

            self._pub_imu.publish(imu_msg)

    def _set_imu_output_enabled(self, enabled: bool) -> bool:
        if self._sonar is None:
            return False

        setter = getattr(self._sonar, 'set_output_imu_batch_enabled', None)
        if setter is None:
            self.get_logger().warn(
                'wlsonar package does not expose IMU output control; skipping configuration.')
            return False

        try:
            setter(enabled)
            return True
        except wlsonar.VersionException:
            self.get_logger().warn(
                'Firmware too old for IMU output setting (requires >= 1.8.0). Skipping.')
        except Exception as e:
            self.get_logger().warn(f'Could not set IMU output: {e}')
        return False

    # ──────────────────────────────────────────────────────────────────────
    # Heartbeat
    # ──────────────────────────────────────────────────────────────────────

    def _heartbeat_callback(self):
        """Publish a periodic DiagnosticStatus with packet receive statistics."""
        with self._lock:
            udp_pkts = self._stats_udp_packets
            range_imgs = self._stats_range_images
            bitmap_imgs = self._stats_bitmap_images
            imu_batches = self._stats_imu_batches
            unknown = self._stats_unknown_packets
            decode_err = self._stats_decode_errors
            timeouts = self._stats_timeouts
            last_seq = self._stats_last_seq_id

        elapsed = time.monotonic() - self._stats_start_time

        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'Sonar 3D-15 Receiver'
        status.hardware_id = self.get_parameter('sonar_ip').get_parameter_value().string_value

        receiving = udp_pkts > 0 and timeouts < 3
        if self._recv_thread is None or not self._recv_thread.is_alive():
            status.level = DiagnosticStatus.ERROR
            status.message = 'Receiver thread not running'
        elif not receiving and elapsed > 10.0:
            status.level = DiagnosticStatus.WARN
            status.message = f'No data (timeouts: {timeouts})'
        elif udp_pkts > 0 and range_imgs == 0 and bitmap_imgs == 0:
            status.level = DiagnosticStatus.WARN
            status.message = f'Packets received but none decoded ({unknown} unknown)'
        else:
            status.level = DiagnosticStatus.OK
            status.message = (
                f'Receiving ({range_imgs} range, {bitmap_imgs} bitmap, {imu_batches} imu batches)'
            )

        status.values = [
            KeyValue(key='udp_packets_total', value=str(udp_pkts)),
            KeyValue(key='range_images', value=str(range_imgs)),
            KeyValue(key='bitmap_images', value=str(bitmap_imgs)),
            KeyValue(key='imu_batches', value=str(imu_batches)),
            KeyValue(key='unknown_packets', value=str(unknown)),
            KeyValue(key='decode_errors', value=str(decode_err)),
            KeyValue(key='timeouts', value=str(timeouts)),
            KeyValue(key='last_sequence_id', value=str(last_seq)),
            KeyValue(key='uptime_s', value=f'{elapsed:.1f}'),
        ]

        diag_array.status.append(status)
        self._pub_diagnostics.publish(diag_array)

    # ──────────────────────────────────────────────────────────────────────
    # Diagnostics (sonar hardware)
    # ──────────────────────────────────────────────────────────────────────

    def _diagnostics_callback(self):
        if self._sonar is None:
            return

        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'Sonar 3D-15'
        status.hardware_id = self.get_parameter('sonar_ip').get_parameter_value().string_value

        try:
            temp = self._sonar.get_temperature()
            status.values.append(KeyValue(key='temperature_c', value=f'{temp:.1f}'))
        except Exception:
            pass

        try:
            sonar_status = self._sonar.get_status()
            status.values.append(
                KeyValue(key='api_status', value=sonar_status.api.status))
            status.values.append(
                KeyValue(key='temperature_status', value=sonar_status.temperature.status))
            status.values.append(
                KeyValue(key='systems_check', value=sonar_status.systems_check.status))

            all_ok = (sonar_status.api.operational
                      and sonar_status.temperature.operational
                      and sonar_status.systems_check.operational)
            if all_ok:
                status.level = DiagnosticStatus.OK
                status.message = 'All systems operational'
            else:
                status.level = DiagnosticStatus.WARN
                msgs = []
                if not sonar_status.api.operational:
                    msgs.append(f'API: {sonar_status.api.message}')
                if not sonar_status.temperature.operational:
                    msgs.append(f'Temp: {sonar_status.temperature.message}')
                if not sonar_status.systems_check.operational:
                    msgs.append(f'Systems: {sonar_status.systems_check.message}')
                status.message = '; '.join(msgs)
        except wlsonar.VersionException:
            status.level = DiagnosticStatus.OK
            status.message = 'Status API not available (firmware < 1.7.0)'
        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f'Could not query status: {e}'

        try:
            about = self._sonar.about()
            status.values.append(KeyValue(key='firmware', value=about.version_short))
            status.values.append(KeyValue(key='chipid', value=about.chipid))
            status.values.append(KeyValue(key='product', value=about.product_name))
        except Exception:
            pass

        diag_array.status.append(status)
        self._pub_diagnostics.publish(diag_array)

    # ──────────────────────────────────────────────────────────────────────
    # Shutdown
    # ──────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        self._running = False

        if self._udp_sock is not None:
            try:
                self._udp_sock.close()
            except Exception:
                pass

        if self._recv_thread is not None:
            self._recv_thread.join(timeout=3.0)

        if self._sonar is not None:
            try:
                self._sonar.set_acoustics_enabled(False)
                self.get_logger().info('Acoustics disabled on shutdown')
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
