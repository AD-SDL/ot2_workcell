import pickle
import socket
import struct
import sys

import cv2
import numpy as np

def find_draw_fiducial(img): 
    # Made by hand. Should be calculated by calibration for better results
    cameraMatrix = np.array([[ 1000,    0, img.shape[0]/2],
                             [    0, 1000, img.shape[1]/2],
                             [    0,    0,              1]])
    distCoeffs = np.zeros((5, 1))

    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img

    # Find markers
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    img_markers = img.copy()

    if len(corners):
        img_markers = cv2.aruco.drawDetectedMarkers(img_markers, corners, ids)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, cameraMatrix, distCoeffs)

        # Draw each marker
        for rvec, tvec in zip(rvecs, tvecs):
            cv2.aruco.drawAxis(img_markers, cameraMatrix, distCoeffs, rvec, tvec, 0.05)

        for cornerset in corners:
            cornerset = cornerset[0].astype(np.int)

            down = cornerset[3] - cornerset[0]
            right = cornerset[2] - cornerset[3]

            tl = cornerset[0] + 3*down - right//2
            tr = cornerset[1] + 3*down + right//2
            bl = cornerset[0] + 5*down - right//2
            br = cornerset[1] + 5*down + right//2

            # cv2.line(img_markers, cornerset[0], cornerset[0] + 3*(cornerset[3]-cornerset[0]), 255, 5)
            # cv2.line(img_markers, cornerset[1], cornerset[1] + 3*(cornerset[2]-cornerset[1]), 255, 5)

            # cv2.line(img_markers, tl, tr, 255, 5)
            # cv2.line(img_markers, tr, br, 255, 5)
            # cv2.line(img_markers, br, bl, 255, 5)
            # cv2.line(img_markers, bl, tl, 255, 5)
    else:
        tl = tr = bl = [0, 0]
        br = gray.shape[::-1]

    return (img_markers,
            np.max([np.min([tl, tr, bl, br], axis=0), [0, 0]], axis=0),
            np.max([tl, tr, bl, br], axis=0))

def locate_pipette(img):
    if len(img.shape) == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        img = img.copy()

    #### Increase the significance of the pipette features in the image

    # Vertical blur to increase the significance of vertical pipette features
    # Balance brightnesses
    # Local thresholding to remove shadow-like effects
    img_0 = cv2.erode(img, np.ones((7, 7), np.uint8), iterations=1)
    img_1 = cv2.dilate(img_0, np.ones((7, 7), np.uint8), iterations=1)
    # img_2 = cv2.blur(img_1, (5, 31))
    # img_2 = cv2.dilate(img_2, np.ones((91, 3), np.uint8), iterations=1)
    # img_2 = cv2.equalizeHist(img_1)
    img_2 = cv2.adaptiveThreshold(img_1, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 51, -13)

    # Dilate white vertically so that background features vanish, leaving vertical features
    img_3 = cv2.dilate(img_2, np.ones((31, 5), np.uint8), iterations=1)
    img_4 = cv2.erode(img_3, np.ones((61, 3), np.uint8), iterations=1)
    # kernel = np.ones((9, 3), np.uint8)
    # img_q = cv2.erode(img_q, kernel, iterations=1)

    cv2.imshow('0 erode', img_0)
    cv2.imshow('1 dilate', img_1)
    cv2.imshow('2 threshold', img_2)
    cv2.imshow('3 dilate', img_3)
    cv2.imshow('4 erode', img_4)



    # blobimg = np.zeros((img_4.shape[0]+100, img_4.shape[1]+100), dtype=np.uint8)
    # blobimg[50:-50, 50:-50] = img_4
    CUT_SIZE = 20
    blobimg = img_4[CUT_SIZE:-CUT_SIZE, CUT_SIZE:-CUT_SIZE]

    numLabels, labels, stats, centroids = cv2.connectedComponentsWithStats(blobimg, 4, cv2.CV_32S)

    if len(blobimg.shape) != 3:
        blobimg = cv2.cvtColor(blobimg, cv2.COLOR_GRAY2BGR)

    # 0 is background
    for i in range(1, numLabels):
        # Extract the connected component statistics and centroid for the current label
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        area = stats[i, cv2.CC_STAT_AREA]
        cX, cY = centroids[i]

        cv2.rectangle(blobimg, (x, y), (x+w, y+h), (0, 255, 0), 3)
        cv2.circle(blobimg, (int(cX), int(cY)), 4, (0, 0, 255), -1)

        # componentMask = (labels==i).astype('uint8') * 255
    cv2.imshow('5 blob', blobimg)
    # cv2.imshow('5 component', componentMask)

    if numLabels == 2:
        return (x+20, y+20), (x+w+20, y+h+20)
    else:
        return (0, 0), (0, 0)

    # params = cv2.SimpleBlobDetector_Params()
    # params.filterByArea = False
    # params.filterByCircularity = False
    # params.filterByConvexity = False
    # params.filterByInertia = False
    # blobDetector = cv2.SimpleBlobDetector_create(params)
    # blobimg = np.zeros((img_4.shape[0]+100, img_4.shape[1]+100), dtype=np.uint8)
    # blobimg[50:-50, 50:-50] = img_4
    # if len(blobimg.shape) != 3:
    #     blobimg = cv2.cvtColor(blobimg, cv2.COLOR_GRAY2BGR)
    # keypoints = blobDetector.detect(blobimg)

    # img_5 = cv2.drawKeypoints(blobimg // 2, keypoints, None, 255, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # cv2.imshow('5 keypoints', img_5)

def draw_pipette(img, tl, ptl, pbr):
    # The given image may be very large
    # Crop the image to at most 200x100 around the pipette
    centroid = (
        tl[1] + (ptl[1]+pbr[1])//2,
        tl[0] + (ptl[0]+pbr[0])//2,
    )
    img_crop = img[
        max(0, centroid[0]-100):centroid[0]+100,
        max(0, centroid[1]-50) :centroid[1]+50
    ]

    cv2.imshow('Pipette Draw 1', img_crop)

    # Subtract the average value of nearby pixels
    # This enhances detail
    blur = cv2.GaussianBlur(img_crop, (1, 15), 20)
    sub = cv2.subtract(img_crop, blur) + 150

    # Use the full set of available brightness levels
    norm = cv2.normalize(sub, None, 0, 255, cv2.NORM_MINMAX)

    cv2.imshow('Pipette Draw 2', norm)

    if len(img.shape) == 3:
        cv2.rectangle(
            img,
            (ptl[0]+tl[0], ptl[1]+tl[1]),
            (pbr[0]+tl[0], pbr[1]+tl[1]),
            (0, 0, 255),
            1)
        cv2.rectangle(
            img,
            (max(0, centroid[1]-50), max(0, centroid[0]-100)),
            (centroid[1]+50, centroid[0]+100),
            (255, 0, 0),
            1)
    else:
        cv2.rectangle(
            img,
            (ptl[0]+tl[0], ptl[1]+tl[1]),
            (pbr[0]+tl[0], pbr[1]+tl[1]),
            255,
            1)
        cv2.rectangle(
            img,
            (max(0, centroid[1]-50), max(0, centroid[0]-100)),
            (centroid[1]+50, centroid[0]+100),
            255,
            1)

#def launch_camera():

def listener():
    # All interfaces, port 8080
    addr = ('', 8080 if len(sys.argv) < 2 else int(sys.argv[1]))
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(addr)
    sock.listen(5)

    print('Listening at:', addr)

def main():
    
    addr = ('', 8080 if len(sys.argv) < 2 else int(sys.argv[1]))
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(addr)
    sock.listen(5)

    print('Listening at:', addr)

    while True:
        sockClient, addrClient = sock.accept()
        print('Got connection from:', addrClient)

        if not sockClient:
            continue

        data = b''
        # headerSize tells how large the header is that specifies the true message size
        headerSize = struct.calcsize('Q')
        try:
            while True:
                # Read at least the header
                while len(data) < headerSize:
                    packet = sockClient.recv(4096)
                    if not packet:
                        break
                    data += packet

                # Split the header off the front of the data, preserve the rest of the data
                packed_msg_size = data[:headerSize]
                data = data[headerSize:]

                # Extract the message size from the header
                messageSize = struct.unpack('Q', packed_msg_size)[0]

                # Read the rest of the message
                while len(data) < messageSize:
                    data += sockClient.recv(4096)

                # Split the message off the front of the data, preserve the rest of the data
                imgData = data[:messageSize]
                data = data[messageSize:]

                # Decode the message into an image
                img = cv2.imdecode(np.frombuffer(imgData, np.byte), cv2.IMREAD_ANYCOLOR)

                cv2.imshow('Receiving video', img)

                # Find the marker and crop out where the pipette should be found
                img, tl, br = find_draw_fiducial(img)
                img_pipette = img[tl[1]:br[1], tl[0]:br[0]]

                if img_pipette.size > 10:
                    # Narrow down the pipette top left and bottom right points
                    ptl, pbr = locate_pipette(img_pipette)

                    # Draw a marker around it if it exists
                    if ptl != (0, 0) and pbr != (0, 0):
                        draw_pipette(img, tl, ptl, pbr)

                    cv2.imshow('Annotated video', img)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                if ptl == (0, 0) and pbr == (0, 0):
                    ms = "False"
                    sockClient.send(ms.encode())
                else:
                    ms = "True" 

                    sockClient.send(ms.encode())

        except struct.error as e:
            print('Lost connection from:', addrClient)
            # sockClient.shutdown(socket.SHUT_RDWR)
            sockClient.close()
        except KeyboardInterrupt:
            print('Shutting down socket')
            # sockClient.shutdown(socket.SHUT_RDWR)
            sockClient.close()
            exit()

if __name__ == '__main__':
    main()