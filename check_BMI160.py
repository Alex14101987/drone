# sudo chmod 666 /dev/i2c-5

import time
from DFRobot_BMI160.python.raspberrypi.DFRobot_BMI160 import *

bmi = DFRobot_BMI160_IIC(addr = BMI160_IIC_ADDR_SDO_H)

# bmi.begin()
if __name__ == "__main__":
  while bmi.begin() != BMI160_OK:
    print("Initialization 6-axis sensor failed.")
    time.sleep(1)
  print("Initialization 6-axis sensor sucess.")
  while True:
    data = bmi.get_sensor_data()
    bmi.begin()
    print("gyro  :  x: %.3f rad/s,  y: %.3f rad/s,  z: %.3f rad/s"%(data['gyro']['x']*3.14/180.0, data['gyro']['y']*3.14/180.0, data['gyro']['z']*3.14/180.0),"accel :  x: %.3f g    ,  y: %.3f g    ,  z: %.3f g    "%(data['accel']['x']/16384.0, data['accel']['y']/16384.0, data['accel']['z']/16384.0))
    # print(data)
    # time.sleep(1)