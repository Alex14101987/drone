from PIL import Image, PngImagePlugin
from matplotlib import pyplot as plt
import os
from datetime import datetime
import json
import numpy as np
import ast
import cv2
import math
import tqdm
import time
import matplotlib.image as mpimg

def str_to_dict(string):
    return ast.literal_eval(string)

def check_roll_pitch(folder_path):
    focal_resolution = 0.046875  # фокусное разрешение в градусах
    focal_resolution = math.pi * 2 / 1920  # фокусное разрешение в радианах
    file_list = os.listdir(folder_path)
    sorted_files = sorted(file_list, key=lambda x: int(x[:-4]))
    video_writer = cv2.VideoWriter('test1.avi', cv2.VideoWriter_fourcc(*'xvid'), 3.5, (640, 480))
    for file_name in sorted_files:
        file_path = os.path.join(folder_path, file_name)
        print(file_path)
        try:
            img = cv2.imread(file_path)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            im = Image.open(file_path)
            metadata = str_to_dict(im.info['metadata'])

            # metadata = metadata['PixHawk_data'][16] # ATTITUDE = 0 AHRS2 = 16
            # print(metadata)
            roll = metadata['PixHawk_data'][0]['roll']
            pitch = metadata['PixHawk_data'][0]['pitch']

            cv2.putText(img, f'roll:   {round((roll*(180/math.pi)), 5)}*', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
            cv2.putText(img, f'pitch: {round((pitch*(180/math.pi)), 5)}*', (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)

            center_x = img.shape[1] // 2
            center_y = img.shape[0] // 2

            x = roll / focal_resolution
            y = pitch / focal_resolution

            point_x = round(center_x - x)
            point_y = round(center_y - y)
            # print('focal_resolution', focal_resolution)
            # print('pitch', pitch*(math.pi/180), 'roll', roll*(math.pi/180))
            # print(x, y)
            cv2.circle(img, (point_x, point_y), 5, (255, 0, 0), -1)

            # plt.imshow(img)
            # plt.show()

            video_writer.write(img)
        except (IndexError, KeyError, ValueError):
            continue
    video_writer.release()
    print('FINISH')


folder_path = "/home/user/Downloads/videos/test"

check_roll_pitch(folder_path)
