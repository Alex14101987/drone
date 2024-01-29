import cv2
import time
from multiprocessing import Process
import socket, pickle, struct

RESIZE_FACTOR = 2


# Получения картинки с удаленной камеры
class ImageSocket():
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self, ip, port):
        self._stop_event = False
        self._ip_device = ip
        self._port = port
        self._video_client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    def stop(self):
        self._stop_event = True

    def _stopped(self):
        return self._stop_event

    def run(self):
        if self._ip_device != None and self._port != None:
            self._cameraRead()
        else:
            print("IP or port not set!")

    def _cameraRead(self):
        self._video_client.connect((self._ip_device, self._port))  
        data = b""
        payload_size = struct.calcsize("Q")

        while not self._stopped():
            try:
                while len(data) < payload_size:
                    packet = self._video_client.recv(4*1024) # 4K
                    if not packet: 
                        break
                    data += packet

                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("Q",packed_msg_size)[0]

                while len(data) < msg_size:
                    data += self._video_client.recv(128*1024)

                frame_data = data[:msg_size]
                data  = data[msg_size:]
                frame = pickle.loads(frame_data)

                frame = cv2.resize(frame, (frame.shape[1]*RESIZE_FACTOR,frame.shape[0]*RESIZE_FACTOR), interpolation = cv2.INTER_AREA)
                cv2.imshow("RECEIVING VIDEO",frame)
                key = cv2.waitKey(1)
                if key == ord('q'):
                    self.stop()
            except Exception as e:
                print(f'Caught error {e}')
                self.stop()
        self._video_client.close()
        print("Stop read Camera.")
        cv2.destroyWindow("RECEIVING VIDEO")

if __name__ == "__main__":

    # Config
    host_ip = '192.168.144.100'
    video_port = 10100

    #Удаленная камера
    image_view = ImageSocket(host_ip, video_port)
    process_image = Process(target=image_view.run)
    process_image.start()
    process_image.join()
