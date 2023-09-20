# mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --daemon 2>/dev/null 1>&2 &
# v4l2-ctl -d /dev/video6 --list-formats-ext

import cv2
import os, shutil
import json
import time
from datetime import datetime
import math
from threading import Thread
from pymavlink import mavutil
from PIL import Image, PngImagePlugin
from io import BytesIO
import smbus2
from mpu6050 import mpu6050
from math import atan2, sqrt, pi
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250


save_dir = 'videos'
WIDTH = 640
HEIGHT = 480
# WIDTH = 1920
# HEIGHT = 1080
FPS = 30
# FOURCC = cv2.VideoWriter_fourcc(*'MJPG')
ALPHA = 0.98
DT = 1 / 400_000

# AUTO_EXPOSURE = 1
# EXPOSURE = 50
FOURCC = cv2.VideoWriter_fourcc(*'YUYV')

class Video():
    def __init__(self, save_dir):
        self.cap = cv2.VideoCapture(4)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FPS)
        self.cap.set(cv2.CAP_PROP_FOURCC, FOURCC)
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, AUTO_EXPOSURE)
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, EXPOSURE)
        self.buffer = ()
        self.frame = None
        self.save_dir = save_dir
        self.count = 0
        self.metadata = {}
        # self.roll = None
        # self.pitch = None
        # self.yaw = 0
        # self.roll_P = None
        # self.pitch_P = None
        # self.yaw = None
        # self.gyro_x = None
        # self.gyro_y = None
        # self.gyro_z = None
        # self.heading = None
        # self.groundspeed = None
        # self.latitude = None
        # self.longitude = None
        # self.altitude = None
        # self.acc_x = None
        # self.acc_y = None
        # self.acc_z = None
        # self.gyro_x_P = None
        # self.gyro_y_P = None
        # self.gyro_z_P = None
        # self.mag_x_P = None
        # self.mag_y_P = None
        # self.mag_z_P = None
        # # self.acc_x_IMU = None
        # # self.acc_y_IMU = None
        # # self.acc_z_IMU = None
        # self.roll_P_G = None
        # self.pitch_P_G = None
        # self.yaw_P_G = None
        # self.gyro_x_IMU = None
        # self.gyro_y_IMU = None
        # self.gyro_z_IMU = None
        self.IMU_data = []
        self.PixHawk_data = []

    def start(self):
        try:
            Thread(target=self.run_cam, daemon=True).start()
        except Exception as e:
            print(f'{type(e).__name__}: CAMERA_DATA NOT FOUND')
        try:
            Thread(target=self.run_IMU, daemon=True).start()
        except Exception as e:
            print(f'{type(e).__name__}: IMU_DATA NOT FOUND')
        Thread(target=self.run_GPS, daemon=True).start()

    def run_cam(self):
        while True:
            _, frame = self.cap.read()
            self.frame = frame
            # cv2.putText(frame, str(self.count), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            self.metadata = {
                'frame_id': self.count,
                'timestamp': datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S.%f')}
            self.metadata.update({
                                'PixHawk_data': self.PixHawk_data,
                                # 'roll_P_G': self.roll_P_G,
                                # 'pitch_P_G': self.pitch_P_G,
                                # 'yaw_P_G': self.yaw_P_G,
                                # 'heading': self.heading,
                                # 'groundspeed': self.groundspeed,
                                # 'latitude': self.latitude,
                                # 'longitude': self.longitude,
                                # 'altitude': self.altitude,
                                # 'roll_P': self.roll_P,
                                # 'pitch_P': self.pitch_P,
                                # 'acc_x_P': self.acc_x,
                                # 'acc_y_P': self.acc_y,
                                # 'acc_z_P': self.acc_z,
                                # 'gyro_x_P': self.gyro_x_P,
                                # 'gyro_y_P': self.gyro_y_P,
                                # 'gyro_z_P': self.gyro_z_P,
                                # 'mag_x_P': self.mag_x_P,
                                # 'mag_y_P': self.mag_y_P,
                                # 'mag_z_P': self.mag_z_P,
                                })
            self.metadata.update({
                # roll и pitch поменяны местами так датчик закреплен боком
                                    'IMU_data': self.IMU_data
                #                   'roll_IMU': self.pitch,
                #                   'pitch_IMU': self.roll,
                #                   # 'yaw_IMU': self.yaw,
                #                   'acc_x_IMU': self.acc_x_IMU,
                #                   'acc_y_IMU': self.acc_y_IMU,
                #                   'acc_z_IMU': self.acc_z_IMU,
                #                   'gyro_x_IMU': self.gyro_x_IMU,
                #                   'gyro_y_IMU': self.gyro_y_IMU,
                #                   'gyro_z_IMU': self.gyro_z_IMU
                                })
            self.buffer = (self.frame, self.metadata)
            self.count += 1
            # print(self.metadata_gps)
            # print(self.count)

    def run_IMU(self):
        mpu = MPU9250(
            address_ak=AK8963_ADDRESS, 
            address_mpu_master=MPU9050_ADDRESS_68, # Master has 0x68 Address
            address_mpu_slave=MPU9050_ADDRESS_68, # Slave has 0x68 Address
            bus=5, 
            gfs=GFS_1000, 
            afs=AFS_8G, 
            mfs=AK8963_BIT_16, 
            mode=AK8963_MODE_C100HZ)

        mpu.configure() # Apply the settings to the registers.
        
        while True:
            # print(mpu.getAllData())
            imu_data = mpu.getAllData()
            self.IMU_data = {
                'timestamp_IMU': imu_data[0],
                'acc_x_IMU': imu_data[1],
                'acc_y_IMU': imu_data[2],
                'acc_z_IMU': imu_data[3],
                'gyro_x_IMU': imu_data[4],
                'gyro_y_IMU': imu_data[5],
                'gyro_z_IMU': imu_data[6],
                'temp_IMU': imu_data[-2],
                        }

    def run_GPS(self):
        # Подключение к устройству
        # master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
        mavpackets = []
        master = mavutil.mavlink_connection('udp:127.0.0.1:14551', baud=115200)
        while True:
            match = master.recv_match()
            if match is not None:
                match = match.to_dict()
                if match['mavpackettype'] == 'ATTITUDE':
                    mavpackets = []
                    mavpackets.append(match)
                elif match['mavpackettype'] == 'BATTERY_STATUS':
                    mavpackets.append(match)
                    self.PixHawk_data = mavpackets
                else:
                    mavpackets.append(match)


def video_write(save_dir):
    video = Video(save_dir)
    video.start()
    # folder_count = math.ceil(len(os.listdir(save_dir)))
    folders = [f for f in os.listdir(save_dir) if os.path.isdir(os.path.join(save_dir, f))]
    if not folders:
        folder_count = 0
    else:
        for i in folders:
            if not i.isdigit():
                folders.remove(i)
        max_folder = max(folders, key=lambda x: int(x))
        folder_count = int(max_folder) + 1
    os.makedirs(f'{save_dir}/{folder_count}')
    os.makedirs(f'{save_dir}/{folder_count}/frames/')
    count = 0
    # video_writer = cv2.VideoWriter(
    #     f'videos/{folder_count}/{count}.avi',
    #     cv2.VideoWriter.fourcc(*"FFV1"),
    #     FPS,
    #     (WIDTH, HEIGHT)
    # )
    # metadata = []
    frame_count = 0
    cur_count = 0
    while True:
        # если счетчик изменился, то дописывем фрейм и добавляем метаданные
        if cur_count < video.count:
            buffer = video.buffer[:]
            # metadata.append(buffer[1])
            # video_writer.write(buffer[0])

            # запись фрейма с метаданными
            im = Image.fromarray(buffer[0])
            png_info = PngImagePlugin.PngInfo()
            png_info.add_text('metadata', str(buffer[1]))
            with BytesIO() as output:
                im.save(output, "PNG", pnginfo=png_info)
                binary_data = output.getvalue()
            with open(f'{save_dir}/{folder_count}/frames/{buffer[1]["frame_id"]}.png', "wb") as file:
                file.write(binary_data)

            cur_count = video.count
            frame_count += 1

        # каждые 100 фреймов релиз видео, запись метаданных в json
        if frame_count >= 100:
            # # print('RELEASE', count, 'FPS', video.cap.get(cv2.CAP_PROP_FPS), 'SIZE', (WIDTH, HEIGHT))
            # with open(f'videos/{folder_count}/{count}.json', 'w') as f:
            #     json.dump(metadata, f)
            # video_writer.release()
            os.sync()
        if frame_count >= 1000:
            # # затем обновление переменных
            frame_count = 0
            # metadata = []
            # count += 1
            # video_writer = None
            # video_writer = cv2.VideoWriter(
            #     f'videos/{folder_count}/{count}.avi',
            #     cv2.VideoWriter.fourcc(*"FFV1"),
            #     FPS,
            #     (WIDTH, HEIGHT)
            # )

            # проверка на максимальный размер папки, если больше 20 Гб, то начинают удаляться старые файлы
            total_size = 0
            for dirpath, dirnames, filenames in os.walk(save_dir):
                for f in filenames:
                    fp = os.path.join(dirpath, f)
                    total_size += os.path.getsize(fp)
            if total_size > (20 * 1024 * 1024 * 1024):
                oldest_file = None
                oldest_file_ctime = None
                for dirpath, dirnames, filenames in os.walk(save_dir):
                    for f in filenames:
                        fp = os.path.join(dirpath, f)
                        current_file_ctime = os.path.getctime(fp)
                        if oldest_file is None or current_file_ctime < oldest_file_ctime:
                            oldest_file = fp
                            oldest_file_ctime = current_file_ctime
                os.remove(os.path.join(save_dir, oldest_file))
            for root, dirs, files in os.walk(save_dir, topdown=False):
                for empty_dir in dirs:
                    dir_path = os.path.join(root, empty_dir)
                    try:
                        os.rmdir(dir_path)
                        print(f"Директория {dir_path} удалена.")
                    except OSError as e:
                        # print(f"Не удалось удалить директорию {dir_path}: {e}")
                        continue


if __name__ == '__main__':
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    video_write(save_dir)
