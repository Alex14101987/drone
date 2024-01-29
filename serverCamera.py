import socket, cv2, pickle, struct, time
from check_cam1 import *
frame = 0
def server_run(frame):
    # config
    host_ip = '192.168.144.100'
    port = 10100
    RESIZE_FACTOR = 4
    TIME_SLEEP = 0.001
    videos_directory = "/home/orangepi/videos"
    socket_address = (host_ip, port)

    # Socket Create
    server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    # Socket Bind
    server_socket.bind(socket_address)

    # Socket Listen
    server_socket.listen(5)
    print("LISTENING AT:",socket_address)

    # Camera on
    # camera = cv2.VideoCapture(0)
    # if not camera.isOpened():
    #     camera = cv2.VideoCapture(1)
    #     if not camera.isOpened():
    #         print("Cannot open camera")
    #         exit()

    # Socket Accept
    camera = cv2.VideoCapture(0)
    while True:
        client_socket,addr = server_socket.accept()
        print('GOT CONNECTION FROM:',addr)
        if client_socket:
            try:
                while True:
                    ret, frame = camera.read()
                    if not ret:
                        print("Can't receive frame (stream end?). Exiting ...")
                        break
                    # frame = display_latest_image(videos_directory)
                    # print(frame.shape)
                    frame = cv2.resize(frame, (frame.shape[1]//RESIZE_FACTOR,frame.shape[0]//RESIZE_FACTOR), interpolation = cv2.INTER_AREA)
                    a = pickle.dumps(frame)
                    time.sleep(TIME_SLEEP)
                    message = struct.pack("Q",len(a))+a
                    client_socket.sendall(message)
            except Exception as e:
                print(f"Connection error: {e}")
        client_socket.close()

if __name__ == '__main__':
    server_run(frame)