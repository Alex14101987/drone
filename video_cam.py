# mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --daemon 2>/dev/null 1>&2 &
# mavproxy.py --baudrate=115200 --master=/dev/ttyS3 --streamrate=10 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --daemon 2>/dev/null 1>&2 &
# mavproxy.py --baudrate=115200 --master=/dev/ttyS3 --out=udp:127.0.0.1:14550 --streamrate=10 --out=udp:127.0.0.1:14551 --daemon 2>~/logMavproxy.txt 1>&2 &
# v4l2-ctl -d /dev/video6 --list-formats-ext
# chmod 666 /dev/i2c-5

import cv2
import os
import time
from datetime import datetime
from threading import Thread
from pymavlink import mavutil
from PIL import Image, PngImagePlugin
from io import BytesIO
from DFRobot_BMI160.python.raspberrypi.DFRobot_BMI160 import *

save_dir = '/home/orangepi/videos'
WIDTH = 640
HEIGHT = 480
# WIDTH = 1920
# HEIGHT = 1080
FPS = 30
# FOURCC = cv2.VideoWriter_fourcc(*'MJPG')
# AUTO_EXPOSURE = 1
# EXPOSURE = 50
FOURCC = cv2.VideoWriter_fourcc(*'YUYV')

class Video():
    def __init__(self, save_dir):
        self.cap = cv2.VideoCapture(0)
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
        # try:
        #     Thread(target=self.check_folder_size, daemon=True).start()
        # except:
        #     print(f'can not empty folders')
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
                                })
            self.metadata.update({
                                    'IMU_data': self.IMU_data
                                 })
            self.buffer = (self.frame, self.metadata)
            self.count += 1

    def run_IMU(self):

        bmi = DFRobot_BMI160_IIC(addr = BMI160_IIC_ADDR_SDO_H)
        while bmi.begin() != BMI160_OK:
            print("Initialization 6-axis sensor failed.")
            time.sleep(1)
        print("Initialization 6-axis sensor sucess.")
        while True:
            # print(mpu.getAllData())
            data = bmi.get_sensor_data()
            self.IMU_data = {
                'acc_x_IMU': data['accel']['x']/16384.0,
                'acc_y_IMU': data['accel']['y']/16384.0,
                'acc_z_IMU': data['accel']['z']/16384.0,
                'gyro_x_IMU': data['gyro']['x']*3.14/180.0,
                'gyro_y_IMU': data['gyro']['y']*3.14/180.0,
                'gyro_z_IMU': data['gyro']['z']*3.14/180.0,
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

    def check_folder_size(self):
        # проверка папки на размер и удаление всех пустых папок и файлов если превышает 15 Гб
        def get_dir_size(save_dir):
            total_size = 0
            for dirpath, dirnames, filenames in os.walk(save_dir):
                for f in filenames:
                    fp = os.path.join(dirpath, f)
                    total_size += os.path.getsize(fp)
            return total_size

        while get_dir_size(save_dir) > (15 * 1024 * 1024 * 1024):  # 15 GB
            oldest_file = None
            oldest_file_ctime = None
            for dirpath, dirnames, filenames in os.walk(save_dir):
                for f in filenames:
                    fp = os.path.join(dirpath, f)
                    current_file_ctime = os.path.getctime(fp)
                    if oldest_file is None or current_file_ctime < oldest_file_ctime:
                        oldest_file = fp
                        oldest_file_ctime = current_file_ctime
            if oldest_file is not None:
                # print(f'Удален файл: {oldest_file}')
                os.remove(oldest_file)
        for root, dirs, files in os.walk(save_dir, topdown=False):
            for empty_dir in dirs:
                dir_path = os.path.join(root, empty_dir)
                try:
                    os.rmdir(dir_path)
                    # print(f"Директория {dir_path} удалена.")
                except OSError as e:
                    # print(f"Не удалось удалить директорию {dir_path}: {e}")
                    continue


def video_write(save_dir):
    video = Video(save_dir)
    video.start()

    # создаем папку с именем на 1 больше чем максимальное имя уже имееющееся в директории
    folders = [f for f in os.listdir(save_dir) if os.path.isdir(os.path.join(save_dir, f))]
    if not folders:
        folder_count = 0
    else:
        for i in folders:
            if not i.isdigit():
                folders.remove(i)
        max_folder = max(folders, key=lambda x: int(x))
        folder_count = int(max_folder) + 1
    # os.makedirs(f'{save_dir}/{folder_count}')
    os.makedirs(f'{save_dir}/{folder_count}/frames/')

    # frame_count = 0
    cur_count = 0

    while True:
        # если счетчик изменился, то дописывем фрейм и добавляем метаданные
        if cur_count < video.count:

            buffer = video.buffer[:]
            
            # запись фрейма с метаданными
            im = Image.fromarray(buffer[0])
            png_info = PngImagePlugin.PngInfo()
            png_info.add_text('metadata', str(buffer[1]))
            with BytesIO() as output:
                im.save(output, "PNG", pnginfo=png_info)
                binary_data = output.getvalue()
            with open(f'{save_dir}/{folder_count}/frames/{buffer[1]["frame_id"]}.png', "wb") as file:
                file.write(binary_data)
            print(folder_count, buffer[1]["frame_id"])
            cur_count = video.count
            # frame_count += 1
            os.sync()
        # каждые 100 frame sync
        # if frame_count >= 100:
            
        #     frame_count = 0

if __name__ == '__main__':
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    video_write(save_dir)
