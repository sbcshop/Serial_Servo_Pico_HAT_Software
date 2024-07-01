#from command import Commands
import machine
from machine import UART
import time
ID = []

Start_Byte = 0xFF
Ping_Instruction = 1
Read_Instruction = 2
Write_Instruction = 3
Reg_Write_Instruction = 4
Action_Instruction = 5
Syc_Write_Instruction = 0x83
Reset_Instruction = 6

#  EEPROM
A_Software_Version = 3  # 0x03
A_ID = 5
A_Baud = 6
A_Return_delay = 7  # 0x07
A_Answer_Status = 8  # 0x08
A_MinAngleLimit = 9  # 0x09
A_MaxAngleLimit = 11  # 0x0B
A_MaxTempLimit = 13  # 0x0D
A_MaxInVoltage = 14  # 0x0E
A_MinInVoltage = 15  # 0x0F
A_MaxTorque = 16  # 0x10
A_Proportional = 21  # 0x15
A_Derivative = 22  # 0x16
A_Integral = 23  # 0x17
A_MinPWM = 24  # 0x18

#  RAM
A_TorqueSwitch = 40  # 0x28
A_TargetPos_H = 42  # 0x2A
A_TargetPos_L = 43  # 0x2B
A_RunTime_H = 44  # 0x2C
A_RunTime_L = 45  # 0x2D
A_RunSpeed_H = 46  # 0x2E
A_RunSpeed_L = 47  # 0x2F
A_Lock = 48  # 0x30
A_CurrentPos = 56  # 0x38
A_CurrentSpeed = 58 # 0x3A
A_CurrentLoad = 60 # 0x3C
A_Voltage = 62  # 0x3E
A_Temp = 63  # 0x3F
A_RegWriteSign = 64  # 0x40

class CheckSum:
    @staticmethod
    def sum_params(params):
        """
        Add parameters
        :param params: add the data parameters
        :return: The addition of parameters
        """
        sum_params = 0
        for para in params:
            sum_params += para
        return sum_params

    @staticmethod
    def checksum_cal(_sum):
        """
        Calculate the Checksum
        :param _sum: the parameters required for Checksum, Hex
        :return: Calculated Checksum Value
        """
        #  Calculate the Checksum
        sum_hex = ~_sum
        check_sum = sum_hex & 0xff
        return check_sum

    def check_sum(self, servo_id, instruction, params=[]):
        """
        :param servo_id: ID of Servo Motor, int(hex)
        :param instruction: Instruction type  #  0x01 - 0x06, 0x83
        :param params: Parameters to pass to Servo eg. position and value, List
        :return: Check sum value in hex
        """
        length = len(params) + 2
        parameter_sum = self.sum_params(params)

        _sum = servo_id + length + instruction + parameter_sum

        check_sum = self.checksum_cal(_sum)

        return check_sum, length


class CommandSet(CheckSum):
    def __init__(self, debug=False):
        self.debug = debug
        #CheckSum.__init__(self)

    def final_data(self, servo_id, instruction, params=[]):
        #  Find Checksum and Data length
        checksum, data_length = self.check_sum(servo_id, instruction, params)
        #  Arrange data in array
        data = [Start_Byte, Start_Byte, servo_id, data_length,
                instruction] + params + [checksum]
        #  Convert Array into byte Array
        if self.debug:
            s = ' '
            print('raw data:', s.join(str(i) for i in data))
        data_bytes = bytearray(data)
        return data_bytes

    def read_set(self, servo_id, add, read_len):
        """
        Read Servo Parameters
        :return: Byte array for Read data from servo
        """
        data = [add, read_len]
        data_array = self.final_data(servo_id=servo_id,instruction=Read_Instruction, params=data)
        return data_array

    def write_set(self, servo_id, add, write_value):
        """
        Write to servo
        :return: Data set for Writing data to servo
        """
        if isinstance(write_value, int):
            data = [add, write_value]
        elif isinstance(write_value, list):
            data = [add] + write_value

        data_array = self.final_data(servo_id=servo_id,instruction=Write_Instruction,params=data)
        return data_array

    def reg_write_set(self, servo_id, add, write_value):
        """
        When the REG WRITE instruction frame is received, the received data
        is stored in the buffer reserve and the Registered Instruction
        Register is set at 1. When the ACTION instruction is received,
        the stored instruction is finally executed.
        :return: Data set for REG Writing data to servo
        """
        if isinstance(write_value, int):
            data = [add, write_value]
        elif isinstance(write_value, list):
            data = [add] + write_value
        data_array = self.final_data(servo_id=servo_id,instruction=Reg_Write_Instruction,params=data)
        return data_array

    def action_set(self):
        """
        Used For asynchronous Writing instruction
        :return: Data set for Action set to servo
        """
        servo_id = 0xFE
        data_array = self.final_data(servo_id=servo_id,instruction=Action_Instruction)
        return data_array

    def syc_write_set(self):
        """
        A SYNC WRITE instruction can modify the control table contents of
        multiple servos at one time,
        :return: Data set for Action set to servo
        """
        servo_id = 0xFE
        data_array = self.final_data(servo_id=servo_id,instruction=Syc_Write_Instruction,params=data)
        return data_array

    def reset_set(self, servo_id):
        """
        Reset Servo Configuration to default
        returns: Reset Data Set for given servo
        """
        data_array = self.final_data(servo_id=servo_id,instruction=Reset_Instruction)
        return data_array


class Commands(CommandSet):
    def __init__(self, debug=False):
        self.debug = debug
        CommandSet.__init__(self, self.debug)

    @staticmethod
    def mc_data(value):
        """
        Convert a word into Bytes
        :param value: Integer
        :return: 2 bytes from word(value)
        """
        data_l = value >> 8
        data_h = value & 255
        return data_l, data_h

    def ping(self, servo_id=254):
        data_array = self.final_data(servo_id, instruction=Ping_Instruction)
        return data_array

    #  READ
    def read_id(self, servo_id, read_len=1):
        return self.read_set(servo_id=servo_id, add=A_ID, read_len=read_len)

    def read_voltage(self, servo_id, read_len=1):
        return self.read_set(servo_id, add=A_Voltage, read_len=read_len)

    def read_temp(self, servo_id, read_len=1):
        return self.read_set(servo_id, add=A_Temp, read_len=read_len)

    def read_pos(self, servo_id, read_len=2):
        rec_data = self.read_set(servo_id, add=A_CurrentPos, read_len=read_len)
        return rec_data

    def read_speed(self, servo_id, read_len=2):
        rec_data = self.read_set(servo_id, add=A_CurrentSpeed, read_len=read_len)
        return rec_data

    def read_load(self, servo_id, read_len=2):
        rec_data = self.read_set(servo_id, add=A_CurrentLoad, read_len=read_len)
        return rec_data

    def read_lock(self, servo_id, read_len=1):
        return self.read_set(servo_id=servo_id, add=A_Lock,read_len=read_len)

    def read_baud(self, servo_id, read_len=1):
        return self.read_set(servo_id=servo_id, add=A_Baud,read_len=read_len)

    def read_angle_limit(self, servo_id, read_len=4):
        return self.read_set(servo_id=servo_id, add=A_MinAngleLimit,read_len=read_len)

    def read_voltage_limit(self, servo_id, read_len=2):
        return self.read_set(servo_id=servo_id, add=A_MaxInVoltage,read_len=read_len)

    def read_temp_limit(self, servo_id, read_len=1):
        return self.read_set(servo_id=servo_id, add=A_MaxTempLimit,read_len=read_len)

    def read_answer_status(self, servo_id, read_len=1):
        #  0: Response to read and ping instructions
        #  1: Response packet to all the instructions
        return self.read_set(servo_id=servo_id, add=A_Answer_Status,read_len=read_len)

    #  Commands Not on Config Software
    def read_torque_switch(self, servo_id, read_len=1):
        return self.read_set(servo_id, add=A_TorqueSwitch, read_len=read_len)

    def read_delay_time(self, servo_id, read_len=1):
        """
        Read Delay Time of servo, default is zero.
        Delay, when the servo receives a command that needs to be answered.
        Time range: parameter (0~254) *2US, if the parameter 250, that is,
        after 500us response, but the default is 0, which means the
        shortest response time.
        """
        return self.read_set(servo_id=servo_id, add=A_Return_delay,read_len=read_len)

    def read_max_torque(self, servo_id, read_len=2):
        """
        Set the maximum output torque of the servo. 0X03FF corresponds to
        the maximum output torque of the servo
        """
        return self.read_set(servo_id=servo_id, add=A_MaxTorque,read_len=read_len)

    def read_min_pwm(self, servo_id, read_len=2):
        return self.read_set(servo_id=servo_id, add=A_MinPWM,read_len=read_len)

    def read_pid(self, servo_id, read_len=3):
        """
        Read Pid Values
        """
        return self.read_set(servo_id=servo_id, add=A_Proportional,
                             read_len=read_len)

    #  WRITE
    def write_id(self, servo_id, new_id):
        return self.write_set(servo_id, add=A_ID, write_value=new_id)

    def write_pos(self, servo_id, r_position, r_time=1, r_speed=500):
        #  data=> 0-3
        pos_ar = self.mc_data(r_position)
        time_ar = self.mc_data(r_time)
        speed_ar = self.mc_data(r_speed)
        data = list(pos_ar + time_ar + speed_ar)
        data_array = self.write_set(servo_id, add=A_TargetPos_H,
                                    write_value=data)
        return data_array

    def write_lock(self, servo_id, value):
        return self.write_set(servo_id, add=A_Lock, write_value=value)

    def write_temp_limit(self, servo_id, value):
        return self.write_set(servo_id, add=A_MaxTempLimit, write_value=value)

    def write_angle_limit(self, servo_id, min_angle=0, max_angle=1000):
        min_angle_byte = self.mc_data(min_angle)
        max_angle_byte = self.mc_data(max_angle)
        data = list(min_angle_byte + max_angle_byte)
        return self.write_set(servo_id, add=A_MinAngleLimit, write_value=data)

    def write_voltage_limit(self, servo_id, min_voltage=50, max_voltage=250):
        data = [max_voltage, min_voltage]
        return self.write_set(servo_id, add=A_MaxInVoltage, write_value=data)

    def write_baud(self, servo_id, baud_num=0x04):
        #  0x06 is baud address
        #  4 for 115200
        if baud_num <= 7:
            data_array = self.write_set(servo_id, add=A_Baud,
                                        write_value=baud_num)
            return data_array

        else:
            if self.debug:
                print('Invalid Baud Rate selection')
            return False

    def write_torque_switch(self, servo_id, torque_status):
        #  torque_status-> 0: Turn Off | 1: Turn On
        data_array = self.write_set(servo_id, add=A_TorqueSwitch,
                                    write_value=torque_status)
        return data_array

    def write_answer_status(self, servo_id, status=0):
        #  0: Response to read and ping instructions
        #  1: Response packet to all the instructions
        data_array = self.write_set(servo_id, add=A_Answer_Status,
                                    write_value=status)
        return data_array

    def write_delay_time(self, servo_id, status=0):
        """
        Read Delay Time of servo, default is zero.
        Delay, when the servo receives a command that needs to be answered.
        Time range: parameter (0~254) *2US, if the parameter is 250, that is,
        after 500us response, but the default is 0, which means the
        shortest response time.
        """
        data_array = self.write_set(servo_id, add=A_Return_delay,
                                    write_value=status)
        return data_array

    def write_max_torque(self, servo_id, data=1023):
        """
        Set the maximum output torque of the servo. 0X03FF corresponds to
        the maximum output torque of the servo
        """
        data_array = self.write_set(servo_id, add=A_MaxTorque,
                                    write_value=data)
        return data_array

    def write_min_pwm(self, servo_id, data=0):
        data_array = self.write_set(servo_id, add=A_MinPWM,
                                    write_value=data)
        return data_array

    def write_pid(self, servo_id, p=15, d=0, i=0):
        """
        Write PID values, in order P, D, and I
        """
        data = [p, d, i]
        data_array = self.write_set(servo_id, add=A_Proportional,
                                    write_value=data)
        return data_array

    def regulation_mode(self, servo_id, speed=500, direction=0):
        """
        360 degree motor mode
        :return: data Array for motor mode
        """
        direction_flag = direction << 10
        dir_speed = direction_flag + speed
        speed_arr = self.mc_data(dir_speed)
        data_array = self.write_set(servo_id, add=A_RunTime_H,
                                    write_value=list(speed_arr))
        return data_array

class SBSServo(Commands):
    """
    This is a class for handle servo command frames
    """

    def __init__(self,baud_rate=115200):
        Commands.__init__(self)
        self.serial = UART(0, baudrate=baud_rate, bits=8, parity=None, stop=1, tx=machine.Pin(0), rx=machine.Pin(1))

   
    def connect(self,baudrate=115200):
        status = self.connect_port(baudrate)
        """
        Open the port and connect
        """
        '''
        self.log.info('Connecting to SB-S Servo on Port %s & baudrate %d..',
                      port,
                      baudrate)
        status = self.connect_port(port, baudrate)
        if status:
            self.log.info('Initialized')
        else:
            self.log.info('Initialization Failed..!!')
        '''
    def byte_to_int(self, byte, order='big', sign=False):
        return int.from_bytes(byte, byteorder=order, signed=sign)
    
    def tempRead(self, ID):
        """
        Read Servo Temperature
        """
        data_array = self.read_temp(ID)
        data = self.serial.write(data_array)
        #data = self.write(data_array)
        if data:
            return self.byte_to_int(data[-2])
        else:
            return data

    def voltageRead(self, ID):
        """
        Read Servo Voltage
        """
        data_array = self.read_voltage(ID)
        data = self.serial.write(data_array)
        #data = self.write(data_array)
        if data:
            return (self.byte_to_int(data[-2])) / 10
        else:
            return data

    def positionRead(self, ID):
        """
        Read Servo Position
        """
        data_array = self.read_pos(ID)
        data = self.serial.write(data_array)
        #data = self.write(data_array)
        if data:
            pos = self.byte_to_int(data[5] + data[6])
            return pos

    def torqueRead(self, ID):
        """
        Read Servo Torque
        """
        data_array = self.read_load(ID)
        data = self.serial.write(data_array)
        #data = self.write(data_array)
        if data:
            torque = self.byte_to_int(data[5] + data[6], sign=True)
            if torque & (1 << 10):
                torque = -(torque & ~(1 << 10))
            return torque

    def speedRead(self, ID):
        """
        Read Servo Speed
        """
        data_array = self.read_speed(ID)
        data = self.serial.write(data_array)
        #data = self.write(data_array)
        if data:
            speed = self.byte_to_int(data[5] + data[6])
            if speed & (1 << 15):
                speed = -(speed & ~(1 << 15))
            return speed

    def adjustAngleOffset(self, ID=1, offset=0):
        """
        Adjust angle offset to zero
        """
        pass

    def readAngleOffset(self, ID):
        """
        Read Servo Angle Offset
        """
        return None

    def readAngleLimit(self, ID):
        """
        Read Servo Angle Limit
        """
        data_array = self.read_angle_limit(ID)
        response = self.serial.write(data_array)
        #response = self.write(data_array)
        if response:
            min_angle = int.from_bytes(response[5] + response[6],
                                       byteorder='big')
            max_angle = int.from_bytes(response[7] + response[8],
                                       byteorder='big')
            return min_angle, max_angle

    def writeAngleLimit(self, ID, angleMin=0, angleMax=1000):
        """
        Write Servo Angle Limit
        """
        data_array = self.write_angle_limit(ID, angleMin, angleMax)
        self.serial.write(data_array)
        #self.write(data_array, waitForResponse=True)

    def readVolLimit(self, ID):
        """
        Read Servo Voltage Limit
        """
        data_array = self.read_voltage_limit(ID)
        response = self.serial.write(data_array)
        if response:
            max_voltage = int.from_bytes(response[5], byteorder='big') / 10
            min_voltage = int.from_bytes(response[6], byteorder='big') / 10
            return min_voltage, max_voltage

    def writeVolLimit(self, ID, voltMin=5.0, voltMax=12.0):
        """
        Read Servo Voltage Limit
        """
        data_array = self.write_voltage_limit(ID, int(voltMin * 10),
                                              int(voltMax * 10))
        self.serial.write(data_array, waitForResponse=True)

    def readTempLimit(self, ID):
        """
        Read Servo Temperature Limit
        """
        data_array = self.read_temp_limit(ID)
        response = self.serial.write(data_array)
        if response:
            max_temp = int.from_bytes(response[5], byteorder='big')
            return max_temp

    def writeTempLimit(self, ID, temp=85):
        """
        Write Servo Temperature Limit
        """
        data_array = self.write_temp_limit(ID, temp)
        self.write(data_array, waitForResponse=True)

    def torqueServo(self, ID, status):
        """
        Enable/Disable Servo Torque
        """
        data_array = self.write_torque_switch(ID, status)
        self.write(data_array, waitForResponse=False)

    def write_lock_status(self, ID, lock_status):
        """
        Change the lock of servo for writing into EEPROM
        :param ID: ID of servo
        :param lock_status: Status of lock. 0-> locked; 1-> unlocked
        :return: None
        """
        data_array = self.write_lock(ID, lock_status)
        self.serial.write(data_array)

    def writeID(self, ID, new_id):
        """
        CHANGES NEEDED
        Write Servo ID
        """
        #  Write New ID into servo
        if ID != new_id:
            data_array = self.write_id(ID, new_id)
            self.serial.write(data_array)

    def readID(self, total_servos=253):
        """
        Scan connected Servo
        :total_servos: Number of servos to scan, int
        :return: Connected servo IDs
        """
        total_servos = 7
        id_array = []
        for i in range(1, total_servos):
            data_array = self.read_id(servo_id=i)
            response = self.serial.write(data_array)
            time.sleep(0.02)
            res = self.serial.read()
            if res is not None and len(res)==15:
                dat = ['{:02x}'.format(x) for x in res]
                ids = dat[2]
                ID.append(ids)               
        return ID

    def write_baudrate(self, ID, new_baud=4):
        """
        Change/Write BAUD Rate of the servo motor
        """
        #  Write New ID into servo
        data_array = self.write_baud(ID, new_baud)
        self.serial.write(data_array)

    def servoWrite(self, ID=1, position=0, r_time=0, r_speed=0):
        """
        Rotate: Change/Write servo time and position values
        """
        data_array = self.write_pos(ID, position, r_time=r_time,r_speed=r_speed)
        time.sleep(0.02)
        self.serial.write(data_array)

    def read_answer(self, ID):
        #  0: Response to read and ping instructions
        #  1: Response packet to all the instructions
        data_array = self.read_answer_status(servo_id=ID)
        response = self.serial.write(data_array)
        if response:
            status = int.from_bytes(response[5], byteorder='big')
            return status

    def write_answer(self, ID=1, status=0):
        """
        0: Response to read and ping instructions
        1: Response packet to all the instructions
        """
        data_array = self.write_answer_status(ID, status)
        self.serial.write(data_array)

    #  Not On Configuration Software
    def read_delay(self, ID):
        """
        Read Delay Time of servo, default is zero.
        Delay, when the servo receives a command that needs to be answered.
        Time range: parameter (0~254) *2US, if the parameter 250, that is,
        after 500us response, but the default is 0, which means the
        shortest response time.
        """
        data_array = self.read_delay_time(servo_id=ID)
        response = self.serial.write(data_array)
        if response:
            status = int.from_bytes(response[5], byteorder='big')
            return status

    def write_delay(self, ID=1, delay=0):
        """
        Read Delay Time of servo, default is zero.
        Delay, when the servo receives a command that needs to be answered.
        Time range: parameter (0~254) *2US, if the parameter 250, that is,
        after 500us response, but the default is 0, which means the
        shortest response time.
        """
        data_array = self.write_delay_time(ID, delay)
        self.serial.write(data_array)

    def read_torque_limit(self, ID):
        """
        Set the maximum output torque of the servo. 0X03FF corresponds to
        the maximum output torque of the servo
        """
        data_array = self.read_max_torque(servo_id=ID)
        response = self.serial.write(data_array)
        if response:
            limit = int.from_bytes(response[5] + response[6],byteorder='big')
            return limit

    def write_torque_limit(self, ID=1, delay=0):
        """
        Set the maximum output torque of the servo. 0X03FF corresponds to
        the maximum output torque of the servo
        """
        data_array = self.write_max_torque(ID, delay)
        self.serial.write(data_array)

    def read_pwm(self, ID):
        data_array = self.read_min_pwm(servo_id=ID)
        response = self.serial.write(data_array)
        if response:
            limit = int.from_bytes(response[5] + response[6], byteorder='big')
            return limit

    def write_pwm(self, ID=1, data=0):
        data_array = self.write_min_pwm(ID, data)
        self.serial.write(data_array)

    def read_pid_value(self, ID):
        data_array = self.read_pid(servo_id=ID)
        response = self.serial.write(data_array)
        if response:
            p = int.from_bytes(response[5], byteorder='big')
            d = int.from_bytes(response[6], byteorder='big')
            i = int.from_bytes(response[7], byteorder='big')
            return p, d, i

    def write_pid_value(self, ID=1, p=15, d=0, i=0):
        data_array = self.write_pid(ID, p=p, d=d, i=i)
        self.serial.write(data_array)

    def motor_mode(self, ID, speed=500, direction=0):
        """
        :param ID: ID of motor
        :param speed: speed of motor
        :param direction: 0 for anticlockwise, 1 for clockwise
        """
        data_array = self.regulation_mode(servo_id=ID, speed=speed,direction=direction)
        self.serial.write(data_array)
        