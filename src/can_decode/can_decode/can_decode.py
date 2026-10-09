#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32
from sls_msgs.msg import UGVsteer, StateUGV

import cantools
import can

class CanDecoderNode(Node):

    def __init__(self):
        super().__init__('can_decoder_node')
        try:
            self.db = cantools.database.load_file('src/ugv_depends/can_decode/can_decode/ugv.dbc')
        except:
            self.db = cantools.database.load_file('./ugv.dbc')
        self.get_logger().info("DBC database loaded successfully")

        self.custom_sub = self.create_subscription(UGVsteer,'/ugv/cmd/control/in/from_custom',self.custom_sub_callback,10)
        self.StateUGV_pub = self.create_publisher(StateUGV,'/ugv/state/msg/out/ugv_state',10)
        self.can_pub = self.create_publisher(Frame, 'to_can_bus', 10)
        self.can_sub = self.create_subscription(Frame,'from_can_bus',self.can_decode_callback,10)

        self.shift_level_reg = 0
        self.target_speed = 0
        self.steering_wheel_angle = 0
        self.drive_mode_reg = 0
        self.brk_en = 0
        self.brake_mode = 0
        self.acc_mode = 0

    def can_decode_callback(self, msg:Frame):
        msg.header.stamp = self.get_clock().now().to_msg()
        id = msg.id
        data = msg.data
        ugv_state_msg = StateUGV()
        if id == 0x51:
            decoded_data = self.db.decode_message(id, bytes(data))
            ugv_state_msg.steering_wheel_direction = decoded_data["Steering_Wheel_Direction"]
            ugv_state_msg.vehicle_speed = decoded_data["CCU_Vehicle_Speed"]
            ugv_state_msg.steering_wheel_angle = decoded_data["CCU_Steering_Wheel_Angle"]
            ugv_state_msg.shiftlevel_sts = decoded_data["CCU_ShiftLevel_Sts"]
            ugv_state_msg.p_sts = decoded_data["CCU_P_Sts"]
            ugv_state_msg.ignition_sts = decoded_data["CCU_Ignition_Sts"]
            ugv_state_msg.drive_mode = decoded_data["CCU_Drive_Mode"]
            self.StateUGV_pub.publish(ugv_state_msg)
        else:
            return
    def custom_sub_callback(self, msg:UGVsteer):
        if msg.shift_level_reg < 125:
            self.shift_level_reg = msg.shift_level_reg
        if msg.target_speed < 125:
            self.target_speed = msg.target_speed
        if msg.steering_wheel_angle < 125:
            self.steering_wheel_angle = msg.steering_wheel_angle
        if msg.drive_mode_reg < 125:
            self.drive_mode_reg = msg.drive_mode_reg
        if msg.brk_en <125:
            self.brk_en = msg.brk_en
        if msg.brake_mode <125:
            self.brake_mode = msg.brake_mode
        if msg.acc_mode <125:    
            self.acc_mode = msg.acc_mode

        signal_dict = {
            'SCU_ShiftLevel_Reg':self.shift_level_reg,
            'SCU_Target_Speed':self.target_speed,
            'SCU_Steering_Wheel_Angle':self.steering_wheel_angle,
            'SCU_Drive_Mode_Reg':self.drive_mode_reg,            
            'SCU_Brk_En':self.brk_en,
            'SCU_Brake_Mode':self.brake_mode,
            'SCU_ACC_Mode':self.acc_mode,
            'SCU_reserved':0,
        }
        self.get_logger().info(str(signal_dict))
        self.send_can_message(message_name="SCU",signal = signal_dict)

    def send_can_message(self, message_name:str="SCU", signal=dict):
        try:
            # 使用 DBC 编码消息
            message = self.db.get_message_by_name(message_name)
            encoded_data = message.encode(signal)

            # 创建 ROS2 Frame 消息
            frame_msg = Frame()
            frame_msg.id = message.frame_id
            frame_msg.data = list(encoded_data)
            frame_msg.is_extended = False
            frame_msg.dlc = message.length

            self.can_pub.publish(frame_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error sending CAN message: {e}")

def main(args=None):
    rclpy.init(args=args)
    can_decoder_node = CanDecoderNode()
    rclpy.spin(can_decoder_node)
    can_decoder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()