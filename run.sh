# !/bin/bash
sudo chmod 666 /dev/i2c-5
export HOME=/home/orangepi/
mavproxy.py --baudrate=115200 --master=/dev/ttyS3 --streamrate=10 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --daemon 2>/dev/null 1>&2 &
/home/orangepi/miniconda3/bin/python3 /home/orangepi/video_cam.py
