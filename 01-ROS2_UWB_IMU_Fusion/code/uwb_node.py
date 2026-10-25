import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import serial
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Path

class UWBNode(Node):
    def __init__(self):
        super().__init__('uwb_node')
        # 创建发布者，发布坐标系位姿
        self.pub = self.create_publisher(PoseStamped, '/uwb_pose', 10)
        self.path_pub = self.create_publisher(Path, '/uwb_path', 10) 
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        self._rx_buffer = bytearray()
        self.ser = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.history_dist = []
        self.history_angle = []
        self.last_path_x = None
        self.last_path_y = None
        
        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB0', 
                baudrate=115200, 
                timeout=0.1
            )
            self.get_logger().info('UWB 串口已连接，开始读取数据...')
        except Exception as e:
            self.get_logger().error(f'串口打开失败: {e}')
            return

        # 每 20ms 执行一次读取回调
        self.timer = self.create_timer(0.02, self.read_uwb)

    def read_uwb(self):
        try:
            if self.ser is None or not self.ser.is_open:
                return

            waiting = self.ser.in_waiting
            if waiting:
                self._rx_buffer.extend(self.ser.read(waiting))

            # Frame: F0 06 | ID(2) | distance_cm(2) | angle_deg(2) | RSSI(1) | AA
            while True:
                start = self._rx_buffer.find(b'\xF0\x06')
                if start < 0:
                    # Keep a trailing F0 in case the header is split across reads.
                    keep = self._rx_buffer[-1:] == b'\xF0'
                    self._rx_buffer[:] = b'\xF0' if keep else b''
                    return
                if start:
                    del self._rx_buffer[:start]
                if len(self._rx_buffer) < 10:
                    return
                if self._rx_buffer[9] != 0xAA:
                    del self._rx_buffer[0]
                    continue

                frame = bytes(self._rx_buffer[:10])
                del self._rx_buffer[:10]
                self.publish_frame(frame)

        except serial.SerialException as e:
            self.get_logger().error(f'串口读取失败: {e}')
        except Exception as e:
            self.get_logger().error(f'UWB 数据解析失败: {e}')

    def publish_frame(self, frame):
        module_id = int.from_bytes(frame[2:4], byteorder='little')
        distance_cm = int.from_bytes(frame[4:6], byteorder='little')
        angle_deg = int.from_bytes(frame[6:8], byteorder='little', signed=True)
        rssi_dbm = frame[8] - 256

        # --- 核心改动：滑动窗口均值滤波 ---
        self.history_dist.append(distance_cm)
        self.history_angle.append(angle_deg)
        
        # 只保留最近 5 次的数据
        if len(self.history_dist) > 5:
            self.history_dist.pop(0)
            self.history_angle.pop(0)
            
        # 求平均值（过滤掉突然跳变的脏数据）
        smooth_dist = sum(self.history_dist) / len(self.history_dist)
        smooth_angle = sum(self.history_angle) / len(self.history_angle)

        # 用平滑后的数据计算
        distance_m = smooth_dist / 100.0
        angle_rad = math.radians(smooth_angle)
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = distance_m * math.cos(angle_rad)
        msg.pose.position.y = distance_m * math.sin(angle_rad)
        msg.pose.orientation.z = math.sin(angle_rad / 2.0)
        msg.pose.orientation.w = math.cos(angle_rad / 2.0)
        
        self.pub.publish(msg)
        
        # 发布轨迹
        x = msg.pose.position.x
        y = msg.pose.position.y

        if (
            self.last_path_x is None
            or math.hypot(x - self.last_path_x, y - self.last_path_y) > 0.05  #只有位置变化超过 5 cm 才新增轨迹点
        ):
            self.path_msg.header.stamp = msg.header.stamp
            self.path_msg.poses.append(msg)
            self.last_path_x = x
            self.last_path_y = y

            if len(self.path_msg.poses) > 500:
               self.path_msg.poses.pop(0)

        self.path_pub.publish(self.path_msg)
        
        # 发布 TF 坐标变换
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'uwb_tag'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)
        
        # 打印日志（这里打印平滑后的数据）
        self.get_logger().info(
            f'解析成功 -> ID: 0x{module_id:04X} | 平滑距离: {smooth_dist:.1f} cm | 平滑角度: {smooth_angle:.1f}°'
        )

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UWBNode()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
