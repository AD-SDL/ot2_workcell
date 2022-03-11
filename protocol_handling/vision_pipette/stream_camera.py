import pickle
import socket
import struct
import sys
import time
import cv2
import numpy as np


def camera_client(is_tip):
    # Address, port 8080
    addr = ("IP", 8080)
    sock = socket.create_connection(addr)

    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, img = cap.read()
        print(img)
        if not ret:
            break

        img = cv2.flip(img, 1)

        # Quality 0 to 100, default=95
        quality = 50
        ret, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        a = jpeg.tobytes()

        msg = struct.pack('Q', len(a)) + a
        
        sock.sendall(msg)

        cv2.imshow('', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

    sock.close()

if __name__ == '__main__':
    camera_client("tip")
