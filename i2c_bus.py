from smbus2 import SMBus

I2C_ADDR = 0x08
bus = SMBus(1)

servoPositionMap = {
    "closed" : 0,
    "straight" : 90,
    "open" : 180
}

def write_to_motor(cmd, leftMotorSpeed, rightMotorSpeed):
    bus.write_i2c_block_data(I2C_ADDR, cmd, [leftMotorSpeed, rightMotorSpeed])

def write_to_servo(angle):
    bus.write_byte(I2C_ADDR, servoPositionMap[angle])