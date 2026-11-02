# Copyright 2025 Julian Valdez
#
# Licensed under the MIT License.

"""Diagnostics CLI tool for the Water Linked Sonar 3D-15.

Measures data rates, range/intensity image timing offsets, and mode-switch
latency. Connects to the sonar independently via the ``wlsonar`` library.
"""

import collections
import socket
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import wlsonar
import wlsonar.range_image_protocol as rip


class SonarDiagNode(Node):
    """Diagnostics node for benchmarking the Sonar 3D-15."""

    def __init__(self):
        super().__init__('sonar_diag')

        self._declare_parameters()

        self._sonar: Optional[wlsonar.Sonar3D] = None
        self._udp_sock = None
        self._recv_thread: Optional[threading.Thread] = None
        self._running = False

        self._lock = threading.Lock()
        self._udp_packets = 0
        self._decode_errors = 0
        self._unknown_packets = 0
        self._other_messages = 0

        # Per-message-type arrival timestamps (monotonic) for rate calculation
        self._range_times: collections.deque = collections.deque(maxlen=30)
        self._bitmap_times: collections.deque = collections.deque(maxlen=30)

        # Arrival timestamps keyed by sequence_id for offset measurement
        self._range_by_seq: dict[int, float] = {}
        self._bitmap_by_seq: dict[int, float] = {}
        self._offsets: collections.deque = collections.deque(maxlen=30)

        # Latest message metadata
        self._last_range_meta: Optional[dict] = None
        self._last_bitmap_meta: Optional[dict] = None

        # Event used by the mode-switch test to detect new data after a switch
        self._switch_event = threading.Event()
        self._switch_first_msg: Optional[dict] = None

        self._connect()

        test_switch = self.get_parameter('test_switch').get_parameter_value().bool_value
        if test_switch:
            self._run_switch_test()
        else:
            self._report_timer = self.create_timer(1.0, self._report_callback)

    # ──────────────────────────────────────────────────────────────────────
    # Parameters
    # ──────────────────────────────────────────────────────────────────────

    def _declare_parameters(self):
        self.declare_parameter('sonar_ip', '192.168.194.96', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='IP address of the Sonar 3D-15'))
        self.declare_parameter('udp_mode', 'multicast', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='UDP mode: "multicast" or "unicast"'))
        self.declare_parameter('interface_ip', '0.0.0.0', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Local interface IP for multicast join / unicast bind'))
        self.declare_parameter('unicast_destination_ip', '', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('unicast_destination_port', 0, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('test_switch', False, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Run mode-switch latency test instead of continuous monitoring'))
        self.declare_parameter('cycles', 3, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Number of switch cycles for the mode-switch test'))

    # ──────────────────────────────────────────────────────────────────────
    # Connection
    # ──────────────────────────────────────────────────────────────────────

    def _connect(self):
        ip = self.get_parameter('sonar_ip').get_parameter_value().string_value
        self.get_logger().info(f'Connecting to Sonar 3D-15 at {ip}...')

        try:
            self._sonar = wlsonar.Sonar3D(ip)
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return

        about = self._sonar.about()
        self.get_logger().info(
            f'Connected: {about.product_name} '
            f'(chipid={about.chipid}, fw={about.version_short})')

        self._sonar.set_acoustics_enabled(True)

        udp_mode = self.get_parameter('udp_mode').get_parameter_value().string_value
        if udp_mode == 'unicast':
            uip = self.get_parameter('unicast_destination_ip').get_parameter_value().string_value
            uport = self.get_parameter(
                'unicast_destination_port').get_parameter_value().integer_value
            self._sonar.set_udp_unicast(uip, uport)
            self.get_logger().info(f'UDP: unicast -> {uip}:{uport}')
        else:
            self._sonar.set_udp_multicast()
            self.get_logger().info('UDP: multicast')

        self._open_udp_and_start_receiver()

    def _open_udp_and_start_receiver(self):
        udp_mode = self.get_parameter('udp_mode').get_parameter_value().string_value
        iface_ip = self.get_parameter('interface_ip').get_parameter_value().string_value

        try:
            if udp_mode == 'unicast':
                uport = self.get_parameter(
                    'unicast_destination_port').get_parameter_value().integer_value
                self._udp_sock = wlsonar.open_sonar_udp_unicast_socket(
                    udp_port=uport, iface_ip=iface_ip)
            else:
                self._udp_sock = wlsonar.open_sonar_udp_multicast_socket(iface_ip=iface_ip)
        except Exception as e:
            self.get_logger().error(f'Failed to open UDP socket: {e}')
            return

        self._udp_sock.settimeout(2.0)
        self._running = True
        self._recv_thread = threading.Thread(target=self._udp_receive_loop, daemon=True)
        self._recv_thread.start()
        self.get_logger().info('Receiver thread started')

    # ──────────────────────────────────────────────────────────────────────
    # UDP receiver
    # ──────────────────────────────────────────────────────────────────────

    def _udp_receive_loop(self):
        while self._running and rclpy.ok():
            try:
                data, addr = self._udp_sock.recvfrom(wlsonar.UDP_MAX_DATAGRAM_SIZE)
            except (TimeoutError, socket.timeout):
                continue
            except OSError as e:
                if self._running:
                    self.get_logger().error(f'UDP socket error: {e}')
                break

            now = time.monotonic()
            with self._lock:
                self._udp_packets += 1
                packet_count = self._udp_packets
            if packet_count == 1:
                self.get_logger().info(
                    f'First UDP packet from {addr[0]}:{addr[1]} ({len(data)} bytes)')

            try:
                msg = rip.unpackb(data)
            except rip.UnknownProtobufTypeError:
                with self._lock:
                    self._unknown_packets += 1
                continue
            except Exception as e:
                with self._lock:
                    self._decode_errors += 1
                    error_count = self._decode_errors
                if error_count <= 3:
                    self.get_logger().warn(f'UDP decode error: {type(e).__name__}: {e}')
                continue

            # IMU batches and other valid RIP messages have different fields.
            # Filter them before accessing image header, dimensions or frequency.
            if not isinstance(msg, (rip.RangeImage, rip.BitmapImageGreyscale8)):
                with self._lock:
                    self._other_messages += 1
                continue

            seq = msg.header.sequence_id
            meta = {
                'width': msg.width,
                'height': msg.height,
                'frequency': msg.frequency,
                'seq': seq,
                'time': now,
            }

            if isinstance(msg, rip.RangeImage):
                with self._lock:
                    self._range_times.append(now)
                    self._last_range_meta = meta
                    self._range_by_seq[seq] = now
                    self._try_compute_offset(seq)
                    self._notify_switch(meta)
            elif isinstance(msg, rip.BitmapImageGreyscale8):
                with self._lock:
                    self._bitmap_times.append(now)
                    self._last_bitmap_meta = meta
                    self._bitmap_by_seq[seq] = now
                    self._try_compute_offset(seq)
                    self._notify_switch(meta)

    def _try_compute_offset(self, seq: int):
        """Compute offset if both range and bitmap arrived for this seq. Caller holds lock."""
        if seq in self._range_by_seq and seq in self._bitmap_by_seq:
            offset = self._bitmap_by_seq[seq] - self._range_by_seq[seq]
            self._offsets.append(offset)
            del self._range_by_seq[seq]
            del self._bitmap_by_seq[seq]

        # Evict stale entries (keep only last 50 seq ids per type)
        for d in (self._range_by_seq, self._bitmap_by_seq):
            if len(d) > 50:
                oldest_keys = sorted(d.keys())[:len(d) - 50]
                for k in oldest_keys:
                    del d[k]

    def _notify_switch(self, meta: dict):
        """Signal the mode-switch test that new data arrived. Caller holds lock."""
        if not self._switch_event.is_set():
            self._switch_first_msg = meta
            self._switch_event.set()

    # ──────────────────────────────────────────────────────────────────────
    # Continuous monitoring
    # ──────────────────────────────────────────────────────────────────────

    def _compute_rate(self, times: collections.deque) -> float:
        if len(times) < 2:
            return 0.0
        span = times[-1] - times[0]
        if span <= 0:
            return 0.0
        return (len(times) - 1) / span

    def _report_callback(self):
        with self._lock:
            range_hz = self._compute_rate(self._range_times)
            bitmap_hz = self._compute_rate(self._bitmap_times)
            rm = self._last_range_meta
            bm = self._last_bitmap_meta
            offsets = list(self._offsets)
            packets = self._udp_packets
            decode_errors = self._decode_errors
            unknown = self._unknown_packets
            other = self._other_messages

        lines = ['--- Sonar Diagnostics ---']
        receiver_alive = self._recv_thread is not None and self._recv_thread.is_alive()
        lines.append(
            f'  UDP: {packets} packets, {decode_errors} decode errors, '
            f'{unknown} unknown types, {other} non-image messages; '
            f'receiver {"running" if receiver_alive else "stopped"}')
        if not receiver_alive:
            lines.append('  Receiver is stopped; check connection/socket errors above.')
        elif packets == 0:
            lines.append(
                '  No UDP received; check interface/destination IP, firewall, '
                'and whether another sonar node is using the same unicast port.')

        if rm:
            lines.append(
                f'  RangeImage:  {range_hz:.1f} Hz  '
                f'(seq {rm["seq"]}, {rm["width"]}x{rm["height"]}, '
                f'freq {rm["frequency"]} Hz)')
        else:
            lines.append('  RangeImage:  no data')

        if bm:
            lines.append(
                f'  BitmapImage: {bitmap_hz:.1f} Hz  '
                f'(seq {bm["seq"]}, {bm["width"]}x{bm["height"]}, '
                f'freq {bm["frequency"]} Hz)')
        else:
            lines.append('  BitmapImage: no data')

        if offsets:
            avg_ms = (sum(offsets) / len(offsets)) * 1000.0
            lines.append(
                f'  RI-BI offset: {avg_ms:.1f} ms avg (last {len(offsets)} pairs)')
        else:
            lines.append('  RI-BI offset: no paired data yet')

        for line in lines:
            self.get_logger().info(line)

    # ──────────────────────────────────────────────────────────────────────
    # Mode-switch latency test
    # ──────────────────────────────────────────────────────────────────────

    def _drain_and_clear(self):
        """Wait briefly for in-flight packets to arrive then clear all tracking state."""
        time.sleep(0.5)
        with self._lock:
            self._range_times.clear()
            self._bitmap_times.clear()
            self._range_by_seq.clear()
            self._bitmap_by_seq.clear()
            self._offsets.clear()
            self._last_range_meta = None
            self._last_bitmap_meta = None
            self._switch_event.clear()
            self._switch_first_msg = None

    def _do_switch(self, target_mode: str, timeout: float = 30.0) -> dict:
        """Switch mode and measure API latency + time to first data."""
        self._drain_and_clear()

        t0 = time.monotonic()
        self._sonar.set_mode(target_mode)
        t_api = time.monotonic()
        api_latency = t_api - t0

        got_data = self._switch_event.wait(timeout=timeout)
        t_data = time.monotonic()

        if got_data:
            with self._lock:
                first = self._switch_first_msg
            blackout = first['time'] - t0
            return {
                'api_latency': api_latency,
                'blackout': blackout,
                'width': first['width'],
                'height': first['height'],
                'frequency': first['frequency'],
                'success': True,
            }
        else:
            return {
                'api_latency': api_latency,
                'blackout': t_data - t0,
                'width': 0,
                'height': 0,
                'frequency': 0,
                'success': False,
            }

    def _run_switch_test(self):
        cycles = self.get_parameter('cycles').get_parameter_value().integer_value
        self.get_logger().info(f'Starting mode-switch latency test ({cycles} cycles)')

        # Wait for initial data to confirm the receiver is working
        self.get_logger().info('Waiting for initial data...')
        if not self._switch_event.wait(timeout=15.0):
            self.get_logger().error('No data received. Cannot run switch test.')
            return
        self._switch_event.clear()

        try:
            current_mode = self._sonar.get_mode()
        except Exception:
            current_mode = 'low-frequency'
        self.get_logger().info(f'Current mode: {current_mode}')

        results_lh = []  # low -> high
        results_hl = []  # high -> low

        for cycle in range(1, cycles + 1):
            self.get_logger().info(f'=== Mode Switch Test: cycle {cycle}/{cycles} ===')

            if current_mode == 'low-frequency':
                first_target = 'high-frequency'
                second_target = 'low-frequency'
            else:
                first_target = 'low-frequency'
                second_target = 'high-frequency'

            # First switch
            self.get_logger().info(f'  {current_mode} -> {first_target}')
            r1 = self._do_switch(first_target)
            self._log_switch_result(r1, first_target)
            if current_mode == 'low-frequency':
                results_lh.append(r1)
            else:
                results_hl.append(r1)
            current_mode = first_target

            # Second switch
            self.get_logger().info(f'  {current_mode} -> {second_target}')
            r2 = self._do_switch(second_target)
            self._log_switch_result(r2, second_target)
            if current_mode == 'high-frequency':
                results_hl.append(r2)
            else:
                results_lh.append(r2)
            current_mode = second_target

        self._log_switch_summary(results_lh, results_hl)

    def _log_switch_result(self, r: dict, target_mode: str):
        if r['success']:
            self.get_logger().info(f'  API call:      {r["api_latency"]:.3f} s')
            self.get_logger().info(f'  Data blackout: {r["blackout"]:.2f} s')
            self.get_logger().info(
                f'  First image:   {r["width"]}x{r["height"]} '
                f'@ {r["frequency"]} Hz (target: {target_mode})')
        else:
            self.get_logger().warn(
                f'  TIMEOUT: no data after {r["blackout"]:.1f} s '
                f'(API call took {r["api_latency"]:.3f} s)')

    def _log_switch_summary(self, results_lh: list, results_hl: list):
        self.get_logger().info('=== Summary ===')
        for label, results in [('low->high', results_lh), ('high->low', results_hl)]:
            ok = [r for r in results if r['success']]
            if not ok:
                self.get_logger().info(f'  {label}: no successful switches')
                continue
            api_vals = [r['api_latency'] for r in ok]
            bo_vals = [r['blackout'] for r in ok]
            self.get_logger().info(
                f'  {label}  '
                f'API: {min(api_vals):.3f}/{_avg(api_vals):.3f}/{max(api_vals):.3f} s  '
                f'blackout: {min(bo_vals):.2f}/{_avg(bo_vals):.2f}/{max(bo_vals):.2f} s')

    # ──────────────────────────────────────────────────────────────────────
    # Shutdown
    # ──────────────────────────────────────────────────────────────────────

    def destroy_node(self):
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
            except Exception:
                pass
        super().destroy_node()


def _avg(vals: list[float]) -> float:
    return sum(vals) / len(vals) if vals else 0.0


def main(args=None):
    rclpy.init(args=args)
    node = SonarDiagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
