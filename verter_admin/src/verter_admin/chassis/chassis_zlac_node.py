import serial

import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import math

import traceback


STATE_MODBUS_INIT = 0
STATE_READY       = 1

MODBUS_ADDR = 1

REG_OPERATION_MODE = 0x200D
OPERATION_MODE_VELOCITY = 0x03

REG_SERIAL_WATCHDOG = 0x2000
SERIAL_WATCHDOG_MS = 1000

REG_CONTROL_WORD = 0x200E
CONTROL_ENABLE = 0x08

REG_DRIVER_STATUS = 0x20A5

REG_TARGET_RPM_M1 = 0x2088


class Odom:
    x: float
    y: float
    theta: float
    last_enc_left: int
    last_enc_right: int
    updated: float


class ZlacNode(Node):

    def __init__(self):
        super().__init__('chassis_zlac_node')
        
        self.declare_parameter('modbus.port', '/dev/chassis')
        self.declare_parameter('modbus.baudrate', 115200)
        self.declare_parameter('modbus.addr', 1)
        self.declare_parameter('wheels.base', 0.3642)
        self.declare_parameter('wheels.left_diameter', 0.196)
        self.declare_parameter('wheels.right_diameter', 0.196)
        self.declare_parameter('wheels.gear_ratio', 1.0)
        self.declare_parameter('wheels.max_motor_rpm', 50)
        self.declare_parameter('wheels.encoder_ticks_per_rev', 4096)
        
        self.port: str = self.get_parameter('modbus.port').value
        self.baudrate: int = self.get_parameter('modbus.baudrate').value
        self.addr: int = self.get_parameter('modbus.addr').value
        self.wheel_base: float = self.get_parameter('wheels.base').value
        self.wheel_left_diameter: float = self.get_parameter('wheels.left_diameter').value
        self.wheel_right_diameter: float = self.get_parameter('wheels.right_diameter').value
        self.wheel_gear_ratio: float = self.get_parameter('wheels.gear_ratio').value
        self.max_motor_rpm: int = self.get_parameter('wheels.max_motor_rpm').value
        self.encoder_ticks: int = self.get_parameter('wheels.encoder_ticks_per_rev').value
        
        self.wheel_left_circumference = math.pi * self.wheel_left_diameter
        self.wheel_right_circumference = math.pi * self.wheel_right_diameter
        self.MOTOR_RPM_PER_MPS_LEFT: float = (60.0 / self.wheel_left_circumference) * self.wheel_gear_ratio
        self.MOTOR_RPM_PER_MPS_RIGHT: float = (60.0 / self.wheel_right_circumference) * self.wheel_gear_ratio
        
        self.state: int = STATE_MODBUS_INIT
        
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.modbus_init_timer = self.create_timer(2.0, self.modbus_init_handler)
        self.modbus_states_timer = self.create_timer(0.02, self.modbus_states_handler)
        
        self.get_logger().info('ZLAC driver node started')
        
        self.serial = None
        self.master = None
        
        self.odom = Odom()
        self.odom.x = 0.0
        self.odom.y = 0.0
        self.odom.theta = 0.0
        self.odom.last_enc_left = None
        self.odom.last_enc_right = None
        self.odom.updated = self.get_clock().now()
    
    def cmd_vel_to_target_rpm(self, linear: float, angular: float) -> tuple[int, int]:
        left_vel = linear - angular * self.wheel_base * 0.5
        right_vel = linear + angular * self.wheel_base * 0.5
        
        left_rpm = 1 * left_vel * self.MOTOR_RPM_PER_MPS_LEFT
        right_rpm = -1 * right_vel * self.MOTOR_RPM_PER_MPS_RIGHT

        max_requested = max(abs(left_rpm), abs(right_rpm))

        if max_requested > self.max_motor_rpm:
            scale = self.max_motor_rpm / max_requested
            left_rpm *= scale
            right_rpm *= scale

        l_rpm = round(left_rpm)
        r_rpm = round(right_rpm)

        if abs(l_rpm) > 65536 or abs(r_rpm) > 65536:
            return 0, 0
        
        if l_rpm < 0:
            l_rpm = 65536 + l_rpm
        
        if r_rpm < 0:
            r_rpm = 65536 + r_rpm
        
        return l_rpm, r_rpm

    def callback(self, msg: Twist):
        if self.state == STATE_READY:
            self.get_logger().info(f"speed={msg.linear.x} turn={msg.angular.z}")
            l_rpm, r_rpm = self.cmd_vel_to_target_rpm(msg.linear.x, msg.angular.z)
            self.get_logger().info(f"left_rpm={l_rpm} right_rpm={r_rpm}")
        
            try:
                self.master.execute(1, cst.WRITE_MULTIPLE_REGISTERS, REG_TARGET_RPM_M1, output_value=[l_rpm, r_rpm])
            except:
                self.get_logger().warning(f'Modbus write error: {traceback.format_exc()}')
                self.master.close()
                self.serial.close()
                self.state = STATE_MODBUS_INIT

    def modbus_states_handler(self):
        if self.state == STATE_READY:
            try:
                res = self.master.execute(MODBUS_ADDR, cst.READ_HOLDING_REGISTERS, REG_DRIVER_STATUS, 8)
                #self.get_logger().info(str(res))
                self.odom_handler(res[2:-2])
            except:
                self.get_logger().warning(f'Modbus read error: {traceback.format_exc()}')
                self.master.close()
                self.serial.close()
                self.state = STATE_MODBUS_INIT
    
    def odom_handler(self, regs: list[int]):
        now = self.get_clock().now()
        enc_left =  (regs[0] << 16) + regs[1]
        enc_right = (regs[2] << 16) + regs[3]
        
        if enc_left & 0x80000000:
            enc_left -= 0x100000000
        
        if enc_right & 0x80000000:
            enc_right -= 0x100000000
        
        if self.odom.last_enc_left is None:
            self.odom.last_enc_left = enc_left
            self.odom.last_enc_right = enc_right
            self.odom.updated = now
            return
        
        dt = (now - self.odom.updated).nanoseconds * 1e-9
        self.odom.updated = now
        
        delta_enc_left = enc_left - self.odom.last_enc_left
        delta_enc_right = -1 * (enc_right - self.odom.last_enc_right)
        
        self.odom.last_enc_left = enc_left
        self.odom.last_enc_right = enc_right
        
        delta_s_left = delta_enc_left * self.wheel_left_circumference / (self.encoder_ticks * self.wheel_gear_ratio)
        delta_s_right = delta_enc_right * self.wheel_right_circumference / (self.encoder_ticks * self.wheel_gear_ratio)
        delta_s = (delta_s_left + delta_s_right) / 2.0
        
        delta_theta = (delta_s_right - delta_s_left) / self.wheel_base
        theta_mid = self.odom.theta + delta_theta * 0.5
        self.odom.theta += delta_theta
        self.odom.theta = math.atan2(math.sin(self.odom.theta), math.cos(self.odom.theta))
        
        self.odom.x += delta_s * math.cos(theta_mid)
        self.odom.y += delta_s * math.sin(theta_mid)
        
        linear_velocity = delta_s / dt
        angular_velocity = delta_theta / dt
        
        half = self.odom.theta * 0.5
        
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)
        
        odom.pose.pose.position.x = self.odom.x
        odom.pose.pose.position.y = self.odom.y
        odom.pose.pose.position.z = 0.0
        
        odom.pose.covariance = [
            1e-4, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  1e-4, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  1e-4
        ]
        
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
         
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_velocity
        
        odom.twist.covariance = [
            1e-4, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  1e6, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  1e-4
        ]
        
        self.odom_pub.publish(odom)
        
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.odom.x
        t.transform.translation.y = self.odom.y
        
        t.transform.rotation.w = math.cos(half)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half)
        
        self.tf_broadcaster.sendTransform(t)
      
    def modbus_init_handler(self):
        if self.state == STATE_MODBUS_INIT:
            try:
                self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, bytesize=8, parity='N', stopbits=1, xonxoff=0)
                self.master = modbus_rtu.RtuMaster(self.serial)
                self.master.set_timeout(0.1)
                self.master.set_verbose(True)
                self.get_logger().info('Modbus master created')
                
                self.master.execute(MODBUS_ADDR, cst.WRITE_SINGLE_REGISTER, REG_OPERATION_MODE, output_value=OPERATION_MODE_VELOCITY)
                self.master.execute(MODBUS_ADDR, cst.WRITE_SINGLE_REGISTER, REG_SERIAL_WATCHDOG, output_value=SERIAL_WATCHDOG_MS)
                self.master.execute(MODBUS_ADDR, cst.WRITE_SINGLE_REGISTER, REG_CONTROL_WORD, output_value=CONTROL_ENABLE)
                
                self.state = STATE_READY
                self.get_logger().info('ZLAC driver initialized successfully')
            except:
                self.get_logger().warning(f'Could not create modbus master: {traceback.format_exc()}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = ZlacNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🔄 Завершение chassis_zlac_node ноды по Ctrl+C...")
    except Exception as e:
        print(f"\n❌ Критическая ошибка chassis_zlac_node ноды: {traceback.format_exc()}")
    finally:
        if node:
            node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
        
        print("✓ chassis_zlac_node нода завершена")


if __name__ == '__main__':
    main()