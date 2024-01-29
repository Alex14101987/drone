from PIL import Image
import os
import time
import cv2
import numpy as np


def get_newest_file(directory):
    files = [f for f in os.listdir(directory) if f.endswith('.png')]
    paths = [os.path.join(directory, basename) for basename in files]
    if paths:
        # print('-------------', max(paths, key=os.path.getctime))
        return max(paths, key=os.path.getctime)
    else:
        print(f'{directory} is empty')  # Получаем самый новый файл по времени создания


def display_latest_image(folders_directory):
    while True:
        subfolders = [f for f in os.listdir(folders_directory) if os.path.isdir(os.path.join(folders_directory, f))]
        if subfolders:  # Проверяем, есть ли подпапки
            latest_subfolder = sorted(subfolders, key=int)[-1]
            frames_directory = os.path.join(folders_directory, latest_subfolder, 'frames')
            newest_image_path = get_newest_file(frames_directory)

            if os.path.exists(newest_image_path):  # Проверяем существование файла
                im = np.array(Image.open(newest_image_path))
                im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
                # cv2.imshow('im', np.array(im))
        # print(im.size)
        return im
        time.sleep(1)  # Период обновления, проверки новых фреймов


# Укажите путь к папке с видео
videos_directory = "/home/orangepi/videos"

# Запуск отображения изображений в бесконечном цикле
# display_latest_image(videos_directory)
if __name__ == '__main__':
    display_latest_image(videos_directory)
