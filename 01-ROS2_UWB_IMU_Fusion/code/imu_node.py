import math
import struct

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu


FRAME_SIZE = 28
HEADER = b'\x55\x61'
GRAVITY = 9.80665


class WT901ImuNode(Node):
    def __init__(self):
        super().__init__('wt901_imu_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('log_every_n_frames', 50)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.log_every_n_frames = max(1, self.get_parameter('log_every_n_frames').value)
        self.frame_count = 0
        self.rx_buffer = bytearray()
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)

        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0)
        except serial.SerialException as exc:
            raise RuntimeError(f'Cannot open IMU serial port {port}: {exc}') from exc

        self.timer = self.create_timer(0.01, self.read_serial)
        self.get_logger().info(f'WT901 IMU connected on {port}; publishing /imu/data')

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def read_serial(self):
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self.rx_buffer.extend(self.ser.read(waiting))

            while True:
                start = self.rx_buffer.find(HEADER)
                if start < 0:
                    keep_header_byte = self.rx_buffer[-1:] == HEADER[:1]
                    self.rx_buffer[:] = HEADER[:1] if keep_header_byte else b''
                    return
                if start:
                    del self.rx_buffer[:start]
                if len(self.rx_buffer) < FRAME_SIZE:
                    return

                frame = bytes(self.rx_buffer[:FRAME_SIZE])
                del self.rx_buffer[:FRAME_SIZE]
                self.publish_frame(frame)
        except serial.SerialException as exc:
            self.get_logger().error(f'IMU serial read failed: {exc}')

    def publish_frame(self, frame):
        # The protocol stores accel, gyro, and Euler angles as 9 little-endian int16 values.
        values = struct.unpack_from('<9h', frame, 2)
        accel = [value / 32768.0 * 16.0 * GRAVITY for value in values[0:3]]
        gyro = [math.radians(value / 32768.0 * 2000.0) for value in values[3:6]]
        roll, pitch, yaw = [math.radians(value / 32768.0 * 180.0) for value in values[6:9]]
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        # Zero covariance means the device driver does not provide covariance estimates.
        self.publisher.publish(msg)

        self.frame_count += 1
        if self.frame_count % self.log_every_n_frames == 0:
            self.get_logger().info(
                f'R/P/Y(deg)=({math.degrees(roll):.2f}, {math.degrees(pitch):.2f}, '
                f'{math.degrees(yaw):.2f}), '
                f'accel(m/s^2)=({accel[0]:.3f}, {accel[1]:.3f}, {accel[2]:.3f})'
            )

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WT901ImuNode()
        rclpy.spin(node)
    except (RuntimeError, KeyboardInterrupt) as exc:
        if not isinstance(exc, KeyboardInterrupt):
            print(exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
