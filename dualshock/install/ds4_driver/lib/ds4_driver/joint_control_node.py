#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import dynamixel_sdk as dxl
from time import time
import atexit

ADDR_TORQUE_ENABLE_XM = 64
ADDR_GOAL_POSITION_XM = 116
ADDR_MOVING_SPEED_XM = 104  
ADDR_OPERATING_MODE_XM = 11 
ADDR_TORQUE_ENABLE_PH = 512
ADDR_GOAL_POSITION_PH = 564
ADDR_MOVING_SPEED_PH = 600 

PROTOCOL_VERSION = 2.0

BAUDRATE = 115200
DEVICENAME = '/dev/ttyACM0'  # OpenCR에 연결된 포트명
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
EXT_POSITION_CONTROL_MODE = 4  # 확장 위치 제어 모드(연속 회전 가능)
POSITION_CONTROL_MODE = 3  # 위치 제어 모드

POSITION_INCREMENT = 10000  # 필요에 따라 조정
POSITION_INCREMENT_XM = 100 # 필요에 따라 조정
VELOCITY = 20  # 모터의 속도 값을 낮게 설정

ADDR_HARDWARE_ERROR_STATUS_XM = 70
ADDR_HARDWARE_ERROR_STATUS_PH = 518

# Define the min and max goal positions
GOAL_POSITION_MIN_PH = -500000
GOAL_POSITION_MAX_PH = 500000

GOAL_POSITION_MIN_XM = -4000
GOAL_POSITION_MAX_XM = 4000

class DS4ControlNode(Node):
    def __init__(self):
        super().__init__('ds4_control_node')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Initialize PortHandler and PacketHandler
        self.portHandler = dxl.PortHandler(DEVICENAME)
        self.packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)

        # Open port 
        if self.portHandler.openPort():
            self.get_logger().info("Succeeded to open the port")
        else:
            self.get_logger().error("Failed to open the port")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info("Succeeded to change the baudrate")
        else:
            self.get_logger().error("Failed to change the baudrate")
            quit()

        self.read_hardware_error(2, "PH")
        self.read_hardware_error(3, "PH")
        # self.read_hardware_error(4, "XM")
        # self.read_hardware_error(5, "XM")
        # self.read_hardware_error(6, "XM")
        
        # Enable torque for all motors
        self.enable_torque(2, ADDR_TORQUE_ENABLE_PH)
        self.enable_torque(3, ADDR_TORQUE_ENABLE_PH)
        # self.enable_torque(4, ADDR_TORQUE_ENABLE_XM)
        # self.enable_torque(5, ADDR_TORQUE_ENABLE_XM)
        # self.enable_torque(6, ADDR_TORQUE_ENABLE_XM)

        # 4번과 5번 XM540 모터를 위치 제어 모드로 설정
        # self.set_operating_mode(4, POSITION_CONTROL_MODE)
        # self.set_operating_mode(5, POSITION_CONTROL_MODE)

        # 6번 XM540 모터를 확장 위치 제어 모드로 설정
        # self.set_operating_mode(6, EXT_POSITION_CONTROL_MODE)
        # self.enable_torque(6, ADDR_TORQUE_ENABLE_XM)  

        # Set moving speed (velocity) for all motors
        self.set_velocity(2, ADDR_MOVING_SPEED_PH, VELOCITY)
        self.set_velocity(3, ADDR_MOVING_SPEED_PH, VELOCITY)
        # self.set_velocity(4, ADDR_MOVING_SPEED_XM, VELOCITY)
        # self.set_velocity(5, ADDR_MOVING_SPEED_XM, VELOCITY)
        # self.set_velocity(6, ADDR_MOVING_SPEED_XM, VELOCITY)

        # Initialize goal positions
        self.goal_position = {2: 0, 3: 0}  # ID 4, 5, 6 제거

        # Initialize last press time for continuous movement tracking
        self.last_press_time = {}

        atexit.register(self.disable_all_torque)

    def enable_torque(self, dxl_id, addr_torque_enable):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, addr_torque_enable, TORQUE_ENABLE)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            self.get_logger().error("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().error("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            self.get_logger().info(f"Dynamixel {dxl_id} has been successfully connected")

    def set_velocity(self, dxl_id, addr_moving_speed, speed):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, addr_moving_speed, int(speed))
        if dxl_comm_result != dxl.COMM_SUCCESS:
            self.get_logger().error("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().error("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            self.get_logger().info(f"Velocity set for Dynamixel {dxl_id}: {speed}")

    def set_goal_position(self, dxl_id, addr_goal_position, position):
        if dxl_id in [2, 3]:  # PH 모터
            position = max(GOAL_POSITION_MIN_PH, min(position, GOAL_POSITION_MAX_PH))
        else:  # XM 모터
            position = max(GOAL_POSITION_MIN_XM, min(position, GOAL_POSITION_MAX_XM))
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, addr_goal_position, int(position))
        if dxl_comm_result != dxl.COMM_SUCCESS:
            self.get_logger().error("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().error("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            self.get_logger().info(f"Goal position set for Dynamixel {dxl_id}: {position}")

    def set_operating_mode(self, dxl_id, mode): # 작동 모드 설정
        self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE_XM, TORQUE_DISABLE)
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE_XM, mode)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            self.get_logger().error("Failed to set operating mode for Dynamixel ID %d: %s" % (dxl_id, self.packetHandler.getTxRxResult(dxl_comm_result)))
        elif dxl_error != 0:
            self.get_logger().error("Error occurred while setting operating mode for Dynamixel ID %d: %s" % (dxl_id, self.packetHandler.getRxPacketError(dxl_error)))
        else:
            self.get_logger().info(f"Operating mode set to {mode} for Dynamixel {dxl_id}")

        self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE_XM, TORQUE_ENABLE)

    def joy_callback(self, msg):
        # 십자버튼의 좌우 버튼: ID 2 제어 (왼쪽: dpad_left, 오른쪽: dpad_right)
        if msg.buttons[14]:  # 십자버튼 좌 (dpad_left)
            self.goal_position[2] -= POSITION_INCREMENT
        elif msg.buttons[16]:  # 십자버튼 우 (dpad_right)
            self.goal_position[2] += POSITION_INCREMENT

        # 십자버튼의 상하 버튼: ID 3 제어 (위: dpad_up, 아래: dpad_down)
        if msg.buttons[15]:  # 십자버튼 상 (dpad_up)
            self.goal_position[3] -= POSITION_INCREMENT
        elif msg.buttons[17]:  # 십자버튼 하 (dpad_down)
            self.goal_position[3] += POSITION_INCREMENT

         # square, circle: ID 4 제어 (위: square, 아래: circle)
        #if msg.buttons[0]:  
        #    self.goal_position[4] -= POSITION_INCREMENT_XM
        #elif msg.buttons[2]:  
        #    self.goal_position[4] += POSITION_INCREMENT_XM

        # triangle, cross: ID 5 제어 (위: triangle, 아래: cross)
        #if msg.buttons[1]:  
        #    self.goal_position[5] -= POSITION_INCREMENT_XM
        #elif msg.buttons[3]:
        #    self.goal_position[5] += POSITION_INCREMENT_XM

        # 왼쪽 트리거, 오른쪽 트리거 버튼: ID 6 제어 (l1, r1)
        #if msg.buttons[4]: 
        #    self.goal_position[6] -= POSITION_INCREMENT_XM
        #elif msg.buttons[6]: 
        #    self.goal_position[6] += POSITION_INCREMENT_XM
            
        # Set the goal positions
        self.set_goal_position(2, ADDR_GOAL_POSITION_PH, self.goal_position[2])
        self.set_goal_position(3, ADDR_GOAL_POSITION_PH, self.goal_position[3])
        # self.set_goal_position(4, ADDR_GOAL_POSITION_XM, self.goal_position[4])
        # self.set_goal_position(5, ADDR_GOAL_POSITION_XM, self.goal_position[5])
        # self.set_goal_position(6, ADDR_GOAL_POSITION_XM, self.goal_position[6])

    def read_hardware_error(self, dxl_id, motor_type): # 다이나믹셀 에러 표시
        if motor_type == "XM":
            addr_error_status = ADDR_HARDWARE_ERROR_STATUS_XM
        else:
            addr_error_status = ADDR_HARDWARE_ERROR_STATUS_PH
        
        error_status, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, dxl_id, addr_error_status)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            self.get_logger().error("Error reading from Dynamixel ID %d: %s" % (dxl_id, self.packetHandler.getTxRxResult(dxl_comm_result)))
        elif dxl_error != 0:
            self.get_logger().error("Dynamixel ID %d error: %s" % (dxl_id, self.packetHandler.getRxPacketError(dxl_error)))
        else:
            self.get_logger().info("Dynamixel ID %d hardware error status: %d" % (dxl_id, error_status))

    def disable_all_torque(self):
        # Disable torque for all motors
        for dxl_id in self.goal_position.keys():
            if dxl_id in [2, 3]: # XM있으면 6까지
                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE_PH, TORQUE_DISABLE)
            else:
                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE_XM, TORQUE_DISABLE)
            self.get_logger().info(f"Dynamixel {dxl_id} torque disabled")

def main(args=None):
    rclpy.init(args=args)
    node = DS4ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()