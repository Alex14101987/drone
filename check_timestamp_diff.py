from PIL import Image, PngImagePlugin
from matplotlib import pyplot as plt
import os
from datetime import datetime
import json
import numpy as np


def str_to_dict(string):
    string = string.replace("'", '"')  # заменяем одинарные кавычки на двойные, чтобы они были совместимы с json
    return json.loads(string)



def check_timestamp_diff(folder_path):
    file_list = os.listdir(folder_path)
    file_list = sorted(file_list, key=lambda x: os.path.getmtime(os.path.join(folder_path, x)), reverse=False)
    x = []
    y = []
    size = []
    count = 0
    for i in range(len(file_list)-1):
        file_path = os.path.join(folder_path, file_list[i])
        im = Image.open(file_path)
        metadata = im.info['metadata']
        timestamp = str_to_dict(metadata)['timestamp']
        timestamp = timestamp.strip("'")
        timestamp = datetime.strptime(timestamp, '%Y-%m-%d %H:%M:%S.%f')
        file_path1 = os.path.join(folder_path, file_list[i+1])
        im1 = Image.open(file_path1)
        metadata1 = im1.info['metadata']
        timestamp1 = str_to_dict(metadata1)['timestamp']
        timestamp1 = timestamp1.strip("'")
        timestamp1 = datetime.strptime(timestamp1, '%Y-%m-%d %H:%M:%S.%f')
        # print(timestamp1 - timestamp)
        x.append(count)
        y.append(timestamp1 - timestamp)
        size.append(os.path.getsize(file_path))
        count += 1
    y = [td.total_seconds() for td in y]
    return x, y, size

plt.rcParams['figure.figsize'] = [20, 20]
fig, ax = plt.subplots()

folder_path = "/home/user/Downloads/videos/frames"
x, y, size = check_timestamp_diff(folder_path)
z = []
a = []
for i in range(len(y)):
    z.append(size[i]/1_000_000)
# for i in range(len(z)):
#     a.append(y[i]/z[i])
ax.plot(x, z, c='b', label='размер: Мб', linewidth=0.5)
ax.plot(x, y, c='r', label='время стало: сек.', linewidth=0.5)
# ax.plot(x, a, c='y', label='coef', linewidth=5)
# folder_path = "/home/user/Downloads/videos/10/frames"
# x1, y1, _ = check_timestamp_diff(folder_path)
# ax.plot(x1, y1, c='r', label='время было: сек.', linewidth=0.5)
# ax.plot(x, y, c='b', label='время стало: сек.', linewidth=0.5)
correlation_matrix = np.corrcoef(y, z)
print(len(a))
correlation = correlation_matrix[0, 1]

plt.title(f'Корреляция между значениями: {correlation}')
leg = plt.legend()
plt.show()


