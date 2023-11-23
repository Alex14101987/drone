from PIL import Image, PngImagePlugin
from matplotlib import pyplot as plt
import os
from datetime import datetime
import json
import numpy as np
import ast


def str_to_dict(string):
    # print(string, end='\r')
    # string = string.replace("'", '"')  # заменяем одинарные кавычки на двойные, чтобы они были совместимы с json
    return ast.literal_eval(string)
    # return json.loads(string)



def check_timestamp_diff(folder_path):
    file_list = os.listdir(folder_path)
    file_list = sorted(file_list, key=lambda x: os.path.getmtime(os.path.join(folder_path, x)), reverse=False)
    x = []
    y = []
    count = 0
    for i in range(len(file_list)-1):
        timestamp = file_list[i][:19]
        timestamp = datetime.fromtimestamp(int(timestamp)/ 1e9)

        timestamp1 = file_list[i+1][:19]
        timestamp1 = datetime.fromtimestamp(int(timestamp1)/ 1e9)

        x.append(count)
        y.append(timestamp1 - timestamp)
        count += 1
    y = [td.total_seconds() for td in y]
    return x, y

plt.rcParams['figure.figsize'] = [20, 20]
fig, ax = plt.subplots()

folder_path = "/home/orangepi/videos/28/frames"
x, y = check_timestamp_diff(folder_path)
print(x, y)

ax.plot(x, y, c='r', label='время стало: сек.', linewidth=0.5)

leg = plt.legend()
plt.show()


