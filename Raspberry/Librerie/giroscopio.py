import smbus
import time
import struct
import RPi.GPIO as gpio
import board

class BNO055:
    BNO055_ID 				= 0xA0
    
    POWER_MODE_NORMAL 		= 0X00
    OPERATION_MODE_CONFIG 	= 0X00
    OPERATION_MODE_IMUPLUS 	= 0X08
    
    VECTOR_EULER 			= 0x1A
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28
    
    BNO055_PAGE_ID_ADDR 	= 0X07
    BNO055_CHIP_ID_ADDR 	= 0x00
    
    BNO055_OPR_MODE_ADDR 	= 0X3D
    BNO055_PWR_MODE_ADDR 	= 0X3E
    
    BNO055_SYS_TRIGGER_ADDR = 0X3F
    
    def __init__(self, sensorId=-1, address=0x28):
        gpio.setup(9,gpio.OUT)
        gpio.output(9,gpio.LOW)
        gpio.setup(10,gpio.OUT)
        gpio.output(10,gpio.LOW)
        gpio.output(9,gpio.HIGH)
        time.sleep(0.5)
        self._sensorId = sensorId
        self._address = address
        self._mode = BNO055.OPERATION_MODE_IMUPLUS
        self._offset = 0.0
        self._bus = smbus.SMBus(1)
    
    def begin(self, mode=None):
        if mode is None: mode = self._mode
        self._bus = smbus.SMBus(1)

        if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
            time.sleep(0.8)# Wait for the device to boot up
            if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
                return False

        self.setMode(BNO055.OPERATION_MODE_CONFIG)

        # Trigger a reset and wait for the device to boot up again
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x20])
        time.sleep(0.8)
        while self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
            time.sleep(0.01)
        time.sleep(0.05)

        # Set to normal power mode
        self.writeBytes(BNO055.BNO055_PWR_MODE_ADDR, [BNO055.POWER_MODE_NORMAL])
        time.sleep(0.01)

        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0])
        time.sleep(0.01)

        self.setMode(mode)
        time.sleep(0.02)

        self._offset = 0.0
        return True
    
    def readAngleRotRaw(self):
        buf = self.readBytes(BNO055.VECTOR_EULER, 6)
        xyz = struct.unpack('h', struct.pack('BB', buf[0], buf[1]))
        out = xyz[0]/16.0
        if out >= 360:
            out = 360
        elif out <= 0:
            out = 0
        return out
    
    def zero(self):
        # heading attuale come nuovo zero, al posto del reset con begin()
        self._offset = self.readAngleRotRaw()
    
    def readAngleRot(self):
        return (self.readAngleRotRaw() - self._offset) % 360
    
    def readAngleInc(self):
        buf = self.readBytes(BNO055.VECTOR_EULER, 6)
        xyz = struct.unpack('h', struct.pack('BB', buf[2], buf[3]))
        out = xyz[0]/16.0
        if out >= 180 or out <= -180:
            out = 0
        return out
    
    def sbatto(self):
        buf = self.readBytes(BNO055.BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, 6)
        xyz = struct.unpack('h', struct.pack('BB', buf[0], buf[1]))
        out = xyz[0]/16.0
        return out
    
    def setMode(self, mode):
        self._mode = mode
        self.writeBytes(BNO055.BNO055_OPR_MODE_ADDR, [self._mode])
        time.sleep(0.03)
    
    def readBytes(self, register, numBytes=1):
        return self._bus.read_i2c_block_data(self._address, register, numBytes)
    
    def writeBytes(self, register, byteVals):
        return self._bus.write_i2c_block_data(self._address, register, byteVals)


if __name__ == '__main__':
    bno = BNO055()
    if bno.begin() is not True:
        print("Error initializing device")
        exit()
    time.sleep(1)
    while True:
        ang = bno.readAngleInc()
#         if ang < 0 or ang > 360:
        print(ang)
        time.sleep(0.01)

