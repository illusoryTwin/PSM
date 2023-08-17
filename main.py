import os
from dynamixel_sdk import *

if os.name == 'nt':
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import sys
    import tty
    import termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Dynamixel settings
MY_DXL = 'MX_SERIES'
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095
PRESENT_POSITION_ADDR = 132
BAUDRATE = 3000000
PROTOCOL_VERSION = 2.0
DXL_ID = 2
DEVICENAME = '/dev/ttyUSB0'
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 20
POSITION_UNIT_ANGLE = 0.088 # One position unit is 0.088°
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]


class Motor:
    def __init__(self, dxl_id, devicename):
        self.DXL_ID = dxl_id
        self.DEVICENAME = devicename
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

    def initialize(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")

    def absolute_move_motor(self, target_position):
        index = 0
        while True:
            print("Press any key to continue! (or press ESC to quit!)")
            if getch() == chr(0x1b):
                break

            # Write goal position
            # dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            #     self.portHandler, self.DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])

            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, self.DXL_ID, ADDR_GOAL_POSITION, target_position)

            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # Read present position
            while True:
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                    self.portHandler, self.DXL_ID, ADDR_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

                print("[ID:%03d] GoalPos:%03d  PresPos:%03d" %
                      (DXL_ID, dxl_goal_position[index], dxl_present_position))

                if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                    break

            if index == 0:
                index = 1
            else:
                index = 0


    def absolute_move_motor_by_angle(self, angle):
        index = 0
        while True:
            print("Press any key to continue! (or press ESC to quit!)")
            if getch() == chr(0x1b):
                break

            # Write goal position
            # dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            #     self.portHandler, self.DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])

            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, self.DXL_ID, ADDR_GOAL_POSITION, int(angle/POSITION_UNIT_ANGLE))

            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # Read present position
            while True:
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                    self.portHandler, self.DXL_ID, ADDR_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

                print("[ID:%03d] GoalPos:%03d  PresPos:%03d" %
                      (DXL_ID, dxl_goal_position[index], dxl_present_position))

                if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                    break

            if index == 0:
                index = 1
            else:
                index = 0


    def relative_move_motor(self, pos_change):
        curr_position = self.get_current_pos()
        index = 0
        while True:
            print("Press any key to continue! (or press ESC to quit!)")
            if getch() == chr(0x1b):
                break

            # Write goal position
            # dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            #     self.portHandler, self.DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])

            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, self.DXL_ID, ADDR_GOAL_POSITION, curr_position+pos_change)

            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # Read present position
            while True:
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                    self.portHandler, self.DXL_ID, ADDR_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

                print("[ID:%03d] GoalPos:%03d  PresPos:%03d" %
                      (DXL_ID, dxl_goal_position[index], dxl_present_position))

                if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                    break

            if index == 0:
                index = 1
            else:
                index = 0

    def relative_move_motor_by_angle(self, angle_change):
        curr_position = self.get_current_pos()
        index = 0
        while True:
            print("Press any key to continue! (or press ESC to quit!)")
            if getch() == chr(0x1b):
                break

            # Write goal position
            # dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            #     self.portHandler, self.DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])

            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, self.DXL_ID, ADDR_GOAL_POSITION, curr_position+int(angle_change/POSITION_UNIT_ANGLE))

            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # Read present position
            while True:
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                    self.portHandler, self.DXL_ID, ADDR_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

                print("[ID:%03d] GoalPos:%03d  PresPos:%03d" %
                      (DXL_ID, dxl_goal_position[index], dxl_present_position))

                if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                    break

            if index == 0:
                index = 1
            else:
                index = 0

   
    def get_current_pos(self):
        dxl_present_position, dxl_comm_result, dxl_error = \
            self.packetHandler.read4ByteTxRx(
                self.portHandler, self.DXL_ID, PRESENT_POSITION_ADDR)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        return dxl_present_position

    def close(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        self.portHandler.closePort()


#motor = Motor(DXL_ID, PROTOCOL_VERSION, DEVICENAME, BAUDRATE, ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE, DXL_MOVING_STATUS_THRESHOLD)
motor = Motor(2, '/dev/ttyUSB0')
motor.initialize()

pos = motor.get_current_pos()
print("pos", pos)

motor.relative_move_motor_by_angle(100)

# motor.absolute_move_motor(90)
# motor.absolute_move_motor_by_angle(1090)

motor.close()
