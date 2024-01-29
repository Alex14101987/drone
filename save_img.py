from PIL import Image, PngImagePlugin
from io import BytesIO


def save_img(self, buffer: tuple, folder_count: int, cur_count: int):
    # запись фрейма с метаданными
    im = Image.fromarray(buffer[0])
    png_info = PngImagePlugin.PngInfo()
    png_info.add_text('metadata', str(buffer[1]))
    with BytesIO() as output:
        im.save(output, "PNG", pnginfo=png_info)
        binary_data = output.getvalue()
    with open(f'/home/orangepi/videos/{folder_count}/frames/{buffer[2]}.png', "wb") as file:
        file.write(binary_data)
