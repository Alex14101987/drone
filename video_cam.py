# mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --daemon 2>/dev/null 1>&2 &
# mavproxy.py --baudrate=115200 --master=/dev/ttyS3 --streamrate=10 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --daemon 2>/dev/null 1>&2 &
# mavproxy.py --baudrate=57600 --master=/dev/ttyS3 --streamrate=10 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --daemon 2>/dev/null 1>&2 &
# mavproxy.py --baudrate=115200 --master=/dev/ttyS3 --out=udp:127.0.0.1:14550 --streamrate=10 --out=udp:127.0.0.1:14551 --daemon 2>~/logMavproxy.txt 1>&2 &
# mavproxy.py --baudrate=115200 --master=/dev/ttyS3 --streamrate=10 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --daemon 2>/home/orangepi/logMavproxy.txt 1>/home/orangepi/logMavproxy1.txt &
# v4l2-ctl -d /dev/video6 --list-formats-ext
# chmod 666 /dev/i2c-5

import cv2
import shutil
import os, os.path
import time
from datetime import datetime
from threading import Thread
from pymavlink import mavutil
from PIL import Image, PngImagePlugin
from io import BytesIO
# from DFRobot_BMI160.python.raspberrypi.DFRobot_BMI160 import *
import copy
import subprocess
import socket, cv2, pickle, struct, time
from check_cam1 import *



save_dir = '/home/orangepi/videos'
MAX_SIZE_GB = 50
WIDTH = 640
HEIGHT = 480
WIDTH = 1920
HEIGHT = 1080
FPS = 60
# FOURCC = cv2.VideoWriter_fourcc(*'MJPG')
# FOURCC = cv2.VideoWriter_fourcc(*'YUYV')

# AUTO_EXPOSURE = 1
# EXPOSURE = 50
FOURCC = cv2.VideoWriter_fourcc(*'MJPG')

class Video():
    def __init__(self, save_dir):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            # cap.release()
            self.cap = cv2.VideoCapture(1)
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
        self.IMU_data = {}
        self.PixHawk_data = {}
        self.timestamp = None

    def start(self):
        try:
            Thread(target=self.run_cam, daemon=True).start()
        except Exception as e:
            print(f'{type(e).__name__}: CAMERA_DATA NOT FOUND')
        # try:
        #     Thread(target=self.run_IMU, daemon=True).start()
        # except Exception as e:
        #     print(f'{type(e).__name__}: IMU_DATA NOT FOUND')
        try:
            Thread(target=self.run_GPS, daemon=True).start()
        except Exception as e:
            print(f'{type(e).__name__}: GPS_DATA NOT FOUND')
        try:
            Thread(target=self.delete_subfolders).start()
        except Exception as e:
            print(f'{type(e).__name__}: CHECK_FOLDER_SIZE CAN NOT WORK')
        time.sleep(1)
        try:
            Thread(target=self.server_run, daemon=True).start()
        except Exception as e:
            print(f'{type(e).__name__}: CAN NOT RUN CAMERA SERVER')

    def run_cam(self):
        while True:
            _, frame = self.cap.read()
            self.frame = frame
            self.timestamp = time.time_ns()
            # cv2.putText(frame, str(self.count), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            # self.metadata = {
            #     'frame_id': self.count,
            #     'timestamp': datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S.%f')}
            self.metadata.update({
                                'PixHawk_data': self.PixHawk_data,
                                })
            self.metadata.update({
                                    'IMU_data': self.IMU_data
                                 })
            self.buffer = (self.frame, self.metadata, self.timestamp)
            self.count += 1

    def server_run(self):
        # config
        host_ip = '192.168.144.100'
        port = 10100
        RESIZE_FACTOR = 8
        TIME_SLEEP = 0.001
        socket_address = (host_ip, port)
        # Socket Create
        server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        # Socket Bind
        server_socket.bind(socket_address)
        # Socket Listen
        server_socket.listen(5)
        print("LISTENING AT:",socket_address)
        while True:
            client_socket,addr = server_socket.accept()
            print('GOT CONNECTION FROM:',addr)
            if client_socket:
                try:
                    while True:
                        frame = self.frame
                        frame = cv2.resize(frame, (frame.shape[1]//RESIZE_FACTOR,frame.shape[0]//RESIZE_FACTOR), interpolation = cv2.INTER_AREA)
                        a = pickle.dumps(frame)
                        time.sleep(TIME_SLEEP)
                        message = struct.pack("Q",len(a))+a
                        client_socket.sendall(message)
                except Exception as e:
                    print(f"Connection error: {e}")
            client_socket.close()

    def run_IMU(self):
        bmi = DFRobot_BMI160_IIC(addr = BMI160_IIC_ADDR_SDO_H)
        while bmi.begin() != BMI160_OK:
            print("Initialization 6-axis sensor failed.")
            time.sleep(1)
        print("Initialization 6-axis sensor sucess.")
        while True:
            data = bmi.get_sensor_data()
            print('IMU_DATA============>', data)
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
        mavpackets = {}
        master = mavutil.mavlink_connection('udp:127.0.0.1:14551', baud=115200)
        while True:
            match = master.recv_match()
            if match is not None:
                match = match.to_dict()
                if match['mavpackettype'] == 'ATTITUDE':
                    self.PixHawk_data = mavpackets
                    mavpackets = {}
                    mavpackets.update({str(match['mavpackettype']): match})
                # elif match['mavpackettype'] == 'BATTERY_STATUS':
                #     mavpackets.append(match)
                #     self.PixHawk_data = mavpackets
                else:
                    mavpackets.update({str(match['mavpackettype']): match})

    def delete_subfolders(self):
        def get_size():
            size = subprocess.check_output(['du','-sh', "/home/orangepi/videos"]).split()[0].decode('utf-8')
            # print('==========', size, type(size))
            if 'G' in size:
                size = round(float(size.replace('G', '')) * 1024**3)
            elif 'M' in size:
                size = round(float(size.replace('M', '')) * 1024**2)
            elif 'K' in size:
                size = round(float(size.replace('K', '')) * 1024)
            print(size/1024**3, 'GB')
            return size
        while get_size() > MAX_SIZE_GB * 1024**3:
            subfolders = [f for f in os.listdir("/home/orangepi/videos") if os.path.isdir(os.path.join("/home/orangepi/videos", f))]
            sorted_subfolders = sorted(subfolders, key=int)
            # print(sorted_subfolders)
            subfolder_path = os.path.join("/home/orangepi/videos", sorted_subfolders[0])
            print(subfolder_path)
            os.chmod(subfolder_path, 0o1777)
            shutil.rmtree(subfolder_path)
            print(f"Удалена папка: {subfolder_path}")

    def save_img(self, buffer: tuple, folder_count: int, cur_count: int):
        # запись фрейма с метаданными
        frame = buffer[0]
        # frame = cv2.resize(frame, (640, 360))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(frame)
        png_info = PngImagePlugin.PngInfo()
        png_info.add_text('metadata', str(buffer[1]))
        with BytesIO() as output:
            im.save(output, "PNG", pnginfo=png_info)
            binary_data = output.getvalue()
        with open(f'/home/orangepi/videos/{folder_count}/frames/{buffer[2]}.png', "wb") as file:
            file.write(binary_data)
        # print(self.cap.get(cv2.CAP_PROP_FPS))
        # print('SAVE_IMG ==========>', folder_count, cur_count)
        os.sync()

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
    fpath = f'/home/orangepi/videos/{folder_count}/frames/'
    os.makedirs(fpath)
    # print('FOLDER=============>', fpath)
    cur_count = 0
    while True:
        if cur_count < video.count:
            buffer = video.buffer[:]
            # start_time = time.time()
            if cur_count % 4 == 0:
                x = copy.copy(buffer)
                thread = Thread(target=video.save_img, args=(x, folder_count, copy.copy(cur_count)))
                thread.start()
            cur_count = video.count

            # end_time = time.time()
            # elapsed_time = end_time - start_time
            # print(f"Time elapsed for this loop iteration: {'{:.8f}'.format(elapsed_time)} seconds")

if __name__ == '__main__':
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    video_write(save_dir)
