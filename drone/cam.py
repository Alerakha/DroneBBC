import cv2
import platform

def main():

    if platform.system() == 'Linux':
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(0)
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    while True:
        _, frame = cap.read()
        frame = cv2.flip(frame,1)
        
        cv2.imshow("Frame", frame)
        
        key = cv2.waitKey(1)
        if key == 27:  # ESC key to exit
            break

    cap.release()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()
