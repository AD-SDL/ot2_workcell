import pickle
import socket
import struct
import sys
import time
import cv2
import numpy as np

def camera_client(error_job):
    # Address, port 8080
    addr = ("192.168.1.81", 8080)
    sock = socket.create_connection(addr)
 
    cap = cv2.VideoCapture(0)
    i = 0
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

        #cv2.imshow('', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(1)
    
        error_message = sock.recv(1024).decode()

        if i > 5:
            print(error_message, i)
            return error_message
        i+=1
    cv2.destroyAllWindows()

    sock.close()

def main_null():
    print("This function is not meant to have a main function")

if __name__ == '__main__':
    camera_client("tip")
