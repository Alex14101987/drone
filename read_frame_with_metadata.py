from PIL import Image
import os

def get_newest_file(directory):
    files = [f[:-4] for f in os.listdir(os.path.join("/home/orangepi/videos", directory, 'frames'))]
    max_files = sorted(files, key=int)[-1]
    print('FILE',max_files)
    return max_files

subfolders = [f for f in os.listdir("/home/orangepi/videos") if os.path.isdir(os.path.join("/home/orangepi/videos", f))]
max_subfolder = sorted(subfolders, key=int)[-1]
print(max_subfolder)
newest_file = os.path.join("/home/orangepi/videos", max_subfolder, 'frames', get_newest_file(os.path.join("/home/orangepi/videos", max_subfolder)))
print(newest_file)
im = Image.open(newest_file+'.png')
metadata = im.info['metadata']
print(metadata)
