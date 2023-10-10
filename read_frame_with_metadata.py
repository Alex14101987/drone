from PIL import Image, PngImagePlugin
from matplotlib import pyplot as plt
import os

file_path = 'videos/1/frames/846.png'
im = Image.open(file_path)
metadata = im.info['metadata']
print(metadata)