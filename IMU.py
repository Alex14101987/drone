import smbus2
import time
from math import atan2, sqrt, pi
import math

ALPHA = 0.99
DT = 1/400_000
# костыль
from mpu6050 import mpu6050
sensor = mpu6050(0x68, 5)
while True:
    accel = sensor.get_accel_data(g=True)
    gyro = sensor.get_gyro_data()
    # print(gyro, math.sqrt(accel['x']**2+accel['y']**2+accel['z']**2))
    # time.sleep(0.01)
        #  # Compute roll and pitch angles using accelerometer data (degrees)
    roll_acc = math.degrees(math.atan2(accel['y'], math.sqrt(accel['x'] ** 2 + accel['z'] ** 2)))
    pitch_acc = math.degrees(math.atan2(-accel['x'], math.sqrt(accel['y'] ** 2 + accel['z'] ** 2)))

    # Complementary filter
    roll = ALPHA * (roll_acc + math.degrees(gyro['x']) * DT) + (1 - ALPHA) * roll_acc
    pitch = ALPHA * (pitch_acc + math.degrees(gyro['y']) * DT) + (1 - ALPHA) * pitch_acc
    
    print(pitch, roll)

# DEVICE_ADDRESS = 0x68
# BUS_NUMBER = 5
# DT = 0.01
# ALPHA = 0.99
# yaw = 0.0

# # для MPU-6050
# ACCEL_XOUT = 0x3B
# ACCEL_YOUT = 0x3D
# ACCEL_ZOUT = 0x3F
# GYRO_XOUT = 0x43
# GYRO_YOUT = 0x45
# GYRO_ZOUT = 0x47
# TEMPERATURE = 0x41

# # для BMI160
# # ACCEL_XOUT = 0x11
# # ACCEL_YOUT = 0x13
# # ACCEL_ZOUT = 0x17 # угадал
# # GYRO_XOUT = 0x0C
# # GYRO_YOUT = 0x0E
# # GYRO_ZOUT = 0x10


# bus = smbus2.SMBus(BUS_NUMBER)

# def read_word(reg):
#     high_byte = bus.read_byte_data(DEVICE_ADDRESS, reg)
#     low_byte = bus.read_byte_data(DEVICE_ADDRESS, reg + 1)
#     value = (high_byte << 8) + low_byte
#     return value

# def read_sensor_data():
#     accel_x = round(read_word(ACCEL_XOUT)/32768 * 2, 2)
#     accel_y = round(read_word(ACCEL_YOUT)/32768 * 2, 2)
#     accel_z = round(read_word(ACCEL_ZOUT)/32768 * 2, 2)
#     gyro_x = round(read_word(GYRO_XOUT)/32768 * 250, 2)
#     gyro_y = round(read_word(GYRO_YOUT)/32768 * 250, 2)
#     gyro_z = round(read_word(GYRO_ZOUT)/32768 * 250, 2)
#     return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

# try:
#     while True:
#         # print(bus)
#         accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = read_sensor_data()
#         roll = atan2(accel_y,accel_z) * 180.0/pi
#         pitch = atan2(-accel_x, sqrt(accel_y*accel_y+accel_z*accel_z)) * 180.0/pi

#         #  # Compute roll and pitch angles using accelerometer data (degrees)
#         roll_acc = math.degrees(math.atan2(accel_y, math.sqrt(accel_x ** 2 + accel_z ** 2)))
#         pitch_acc = math.degrees(math.atan2(-accel_x, math.sqrt(accel_y ** 2 + accel_z ** 2)))

#         # Complementary filter
#         roll = ALPHA * (roll_acc + math.degrees(gyro_x) * DT) + (1 - ALPHA) * roll_acc
#         pitch = ALPHA * (pitch_acc + math.degrees(gyro_y) * DT) + (1 - ALPHA) * pitch_acc
#         yaw += math.degrees(gyro_z) * DT


#         print(
#             f"Accelerometer: X = {accel_x}, Y = {accel_y}, Z = {accel_z}",
#             f"Gyroscope: X = {gyro_x}, Y = {gyro_y}, Z = {gyro_z}",
#             f'Angles: roll = {round(roll, 2)}, pitch = {round(pitch, 2)}',
#             f'SUM = {round(sqrt(accel_x**2 + accel_y**2+accel_z**2), 2)}'
#             )
#         time.sleep(1)  

# except KeyboardInterrupt:
#     pass
# finally:
#     bus.close()