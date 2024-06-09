import cv2
import numpy as np
from matplotlib import pyplot as plt

class Track:

    def __init__(self, draw=True):
        self.count = 0
        self.draw = draw
        if self.draw:
            self.window_names=['image', 'gray', 'binary', 'sobel', 'edge']
            cv2.namedWindow(self.window_names[0])
            cv2.moveWindow(self.window_names[0], 0, 0)
            cv2.namedWindow(self.window_names[1])
            cv2.moveWindow(self.window_names[1], 0, 150)
            cv2.namedWindow(self.window_names[2])
            cv2.moveWindow(self.window_names[2], 0, 300)
            cv2.namedWindow(self.window_names[3])
            cv2.moveWindow(self.window_names[3], 0, 450)
            cv2.namedWindow(self.window_names[4])
            cv2.moveWindow(self.window_names[4], 0, 600)

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
        ret,binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
        sobel = cv2.Sobel(binary, -1, 1, 0)
        edge = cv2.Canny(sobel, 100, 200)
        # edge = cv2.bilateralFilter(edge, -1, 50, 50)
        cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(edge)
        line_pos = 0
        line_tall = 0
        for i in range(1,cnt):
            (x, y, w, h, area) = stats[i]
            if area < 80:
                continue
            if line_tall < h:
                line_tall = h
                line_pos = x + int(w/2)
            if self.draw:
                cv2.rectangle(work, (x,y,w,h), (0,0,255))
        print('line_pos ', line_pos)

        # cnt, labels = cv2.connectedComponents(edge)
        # # 레이블 개수만큼 순회
        # for i in range(cnt):
        #     # 레이블이 같은 영여겡 랜덤 색상 적용
        #     img2[labels==i] = [int(j) for j in np.random.randint(0, 255, 3)]


        # lines = cv2.HoughLinesP(edge,1,np.pi/180, threshold=30,minLineLength=20,maxLineGap=3)
        # if lines is not None:
        #     print('found ', len(lines), ' lines')
        #     for points in lines:
        #         x1,y1,x2,y2 = points[0]
        #         cv2.line(resize, (x1, y1),(x2,y2), (255,0,0), 3)

        # titles = ['gray', 'BINARY']
        # images = [gray, binary]
        # for i in range(2):
        #    plt.subplot(2, 1, i+1), plt.imshow(images[i], 'gray', vmin=0,vmax=255)
        #    plt.title(titles[i])
        #   #  plt.xticks([]).yticks([])
        # plt.show()

        # print(blur[100])
        # h, w = gray.shape
        # val = np.argmax(edge[int(h-10)])
        # print('max: ', val)

        # Display image
        if self.draw:
            h, w, c = work.shape
            cv2.circle(work, (line_pos, int(h-50)), 5, (255, 0, 0), 3)
            # cv2.imshow("camera", work)

            cv2.imshow(self.window_names[0], work)
            cv2.imshow(self.window_names[1], gray)
            cv2.imshow(self.window_names[2], binary)
            # cv2.imshow("hist", hist)
            cv2.imshow(self.window_names[3], sobel)
            cv2.imshow(self.window_names[4], edge)

        return line_pos

        # self.count = self.count+1
        # print('track', self.count)
