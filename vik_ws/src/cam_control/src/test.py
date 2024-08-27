import cv2
import numpy as np

capture = cv2.VideoCapture(0)
codec = cv2.VideoWriter_fourcc(*'XVID')

recording_flag = False

while True:
    ret, frame_temp = capture.read()
    cv2.imshow('FRAME', frame_temp)
    key=cv2.waitKey(1)
    if key%256 == 27:
        break
    elif key%256 == 32:
        if recording_flag == False:
            # we are transitioning from not recording to recording
            output = cv2.VideoWriter('CAPTURE.avi', codec, 30, (640, 480))
            recording_flag = True
        else:
            # transitioning from recording to not recording
            recording_flag = False

    if recording_flag:
        output.write(frame_temp)

capture.release()
output.release()
cv2.destroyAllWindows()
