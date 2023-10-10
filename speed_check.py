from PIL import Image, PngImagePlugin
from io import BytesIO
import os
from matplotlib import pyplot as plt
import numpy as np
from time import time



img = Image.open('пример файла с метаданными.png')
img_array = np.array(img)
metadata = img.info['metadata']
count = 0
timing = [0] * 10
frames = 100
while count < frames:

    img_array[0, 0] = np.random.randint(0, 256, 3)
    metadata = str(np.random.randint(0, 1000)) + metadata
    start = time()

    buffer = (img_array, metadata)
    timing[0] += time() - start
    start = time()

    im = Image.fromarray(buffer[0])
    timing[1] += time() - start
    start = time()
    # print(dir(im))
    # print(dir(PngImagePlugin))
    # print(dir(BmpImagePlugin))
    png_info = PngImagePlugin.PngInfo()
    timing[2] += time() - start
    start = time()

    png_info.add_text('metadata', str(buffer[1]))
    timing[3] += time() - start
    start = time()

 #   with BytesIO() as output:
 #       timing[9] += time() - start
 #       start = time()
#
 #       im.save(output, "PNG", pnginfo=png_info)
  #      timing[4] += time() - start
   #     start = time()
#
 #       binary_data = output.getvalue()
  #      timing[5] += time() - start
   #     start = time()
#
 #   with open(f'Videos/{count}.png', "wb") as file:
  #      file.write(binary_data)
   #     timing[6] += time() - start
    #    start = time()

    count += 1
    timing[7] += time() - start
    start = time()

if count >= 10:
    os.sync()
    timing[8] += time() - start
    start = time()

    # Normalize the timing data
    total = sum(timing)
    timing_percentage = [round(t / total * 100, 3) for t in timing]

    # Plot the pie chart
    labels = [
        f'write buffer: {round(timing[0]/frames, 3)}',
        f'read image from buffer: {round(timing[1]/frames, 3)}',
        f'create png_info: {round(timing[2]/frames, 3)}',
        f'add_text for metadata: {round(timing[3]/frames, 3)}',
        f'save png_info: {round(timing[4]/frames, 3)}',
        f'create binary_data: {round(timing[5]/frames, 3)}',
        f'write binary_data: {round(timing[6]/frames, 3)}',
        f'count += 1: {round(timing[7]/frames, 3)}',
        f'os.sync(): {round(timing[8]/frames, 3)}',
        f'create BytesIO: {round(timing[9]/frames, 3)}'
    ]
    print(timing)
    print(total)
    print(timing_percentage)
    plt.pie(timing_percentage, labels=labels, autopct='%1.1f%%', labeldistance=1, pctdistance=0.5)
    plt.axis('equal')
    plt.title('Loop Time')
    plt.show()
