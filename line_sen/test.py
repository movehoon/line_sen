import cv2
from track import Track

cap = cv2.VideoCapture('C://Users/moveh/Downloads/field1.avi') 
track = Track(False)

# skip_count = 1200
skip_count = 0

if cap.isOpened():
    while True:
        ret, frame = cap.read()
        if ret == True :
            if skip_count > 0:
                skip_count = skip_count-1
            else:
                ret = track.track(frame)
                h,w,c = frame.shape
                if ret > 0:
                    cv2.circle(frame, (int(ret), int(h-60)), 5, (255, 0, 0), 3)
                cv2.imshow('[video_pub]frame', frame)
                cv2.waitKey(30)
        else:
            break

