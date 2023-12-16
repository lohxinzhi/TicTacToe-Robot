import cv2
import numpy as np



video_source = "http://10.42.0.31:81/stream"
# video_source = 2

cap = cv2.VideoCapture(video_source)

while True:
    ret, frame = cap.read()
    if ret == True:
        # cv2.imshow("video", frame)
        # cv2.waitKey(1)
        # print(frame.shape)
        height, width, _ = frame.shape
        print (frame.shape)
        # cropped = frame[40:200, 70:230]
        cropped = frame[150:850, 240:910]

        cv2.imshow("video", cropped)
        # cv2.imshow("video", frame)

        cv2.waitKey(1)
        

    else: 
        break

cap.release()
cv2.destroyAllWindows()
