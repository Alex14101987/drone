# !/bin/bash
sudo chmod 666 /dev/i2c-5
mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --daemon 2>/dev/null 1>&2 &
sudo python3 /home/orangepi/video_cam.py
