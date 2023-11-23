from PIL import Image


file_path = 'videos/43/frames/1698793477596341818.png'
im = Image.open(file_path)
metadata = im.info['metadata']
print(metadata)
