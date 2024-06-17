import cv2
import numpy as np
from matplotlib import pyplot as plt

class Track:

    def __init__(self, draw=True):
        self.LINE_LR_DEPTH = 30
        self.count = 0
        self.draw = draw
        self.found = False
        if self.draw:
            self.window_names=['image', 'gray', 'edge']
            cv2.namedWindow(self.window_names[0])
            cv2.moveWindow(self.window_names[0], 0, 0)
            cv2.namedWindow(self.window_names[1])
            cv2.moveWindow(self.window_names[1], 0, 150)
            cv2.namedWindow(self.window_names[2])
            cv2.moveWindow(self.window_names[2], 0, 300)

        self.color_dict_HSV = {
              'black': [[180, 255, 30], [0, 0, 0]],
              'white': [[180, 18, 255], [0, 0, 231]],
              'red1': [[180, 255, 255], [159, 50, 70]],
              'red2': [[9, 255, 255], [0, 50, 70]],
              'green': [[89, 255, 255], [36, 50, 70]],
              'blue': [[128, 255, 255], [90, 50, 70]],
              'yellow': [[35, 255, 255], [25, 50, 70]],
              'purple': [[158, 255, 255], [129, 50, 70]],
              'orange': [[24, 255, 255], [10, 50, 70]],
              'gray': [[180, 18, 230], [0, 0, 40]]}

    def track(self, image):

        h, w, c = image.shape
        # resize = cv2.resize(image, dsize=(320, 240), interpolation = cv2.INTER_AREA)
        work = image[int(h*3/4):h-30, 0:w]
        hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)
        # lowerValues = np.array([25, 50, 70])
        # upperValues = np.array([35, 255, 255])
        # binary = cv2.inRange(hsv, lowerValues, upperValues)
        # ret,binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
        # sobel = cv2.Sobel(binary, -1, 1, 0)
        edge = cv2.Canny(gray, 200, 400)        # P1: Image, P2: connected points, P3: Threshold
        cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(edge)
        line_pos = 0
        line_pos1 = 0
        line_pos2 = 0
        line_tall1 = 0
        line_tall2 = 0
        for i in range(1,cnt):
            (x, y, w, h, area) = stats[i]
            if area < 80:
                continue
            if line_tall1 < h:
                line_tall1 = h
                line_pos1 = x + int(w/2)
            elif line_tall2 < h:
                line_tall2 = h
                line_pos2 = x + int(w/2)
            if self.draw:
                cv2.rectangle(work, (x,y,w,h), (0,0,255))

        if abs(line_pos1 - line_pos2) < self.LINE_LR_DEPTH:
            self.found = True
        else:
            self.found = False

        if self.found:
            line_pos = (line_pos1+line_pos2)/2
        else:
            line_pos = -1
        print('line_pos=', line_pos, '(', line_pos1, ',', line_pos2, ')')

        # Display image
        if self.draw:
            h, w, c = work.shape
            cv2.circle(work, (int(line_pos), int(h-60)), 5, (255, 0, 0), 3)

            cv2.imshow(self.window_names[0], work)
            cv2.imshow(self.window_names[1], gray)
            cv2.imshow(self.window_names[2], edge)

        return line_pos

        # self.count = self.count+1
        # print('track', self.count)
