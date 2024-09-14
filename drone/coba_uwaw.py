import cv2
import numpy as np
import math
import platform
import mavarm3_2
import time
# from pymavlink import mavutil
# Buat bagaimana caranya agar drone dapat memposisikan diri ke tengah objek baru turun, dan jika pada saat turun dia bergeser dia akan mengembalikan posisi ke center sebelum menurunkan ketinggian lagi, kemudian setelah turun pada ketinggian tertentu dia hover diam beberapa detik dahulu sbeelum lanjut. Buat flag condition
# Outdoor : naik +- 5~10 meter kemudian maju sampai detect kotak, kemudian hover sambil turun ketinggian ~1 meter dan drop menggerakkan servo, dia belok kiri atau kanan berdasarkan situasi indoornya. Jika di misi indoor dia belok ke kanan maka misi outdoor setelah dia selesai pada persegi pertama maka dia yaw 90 derajat ke kiri, yaw 90 derajat kemudian lurus sampai menemukan persegi lagi. Setelah itu yaw lagi 120 derajat, lurus sampai nemu persegi terakhir +> landing.
# How to change the camera drone using channel 11 on rcover
def main():

    if platform.system() == 'Linux':
        cap0 = cv2.VideoCapture(0, cv2.CAP_V4L2)
        cap1 = cv2.VideoCapture(1, cv2.CAP_V4L2)
    else:
        cap0 = cv2.VideoCapture(0) # Kamera bawah
        cap1 = cv2.VideoCapture(1) # Kamera depan, lek kuwalik kari diwalik
    
    cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
    cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    active_cam = 0
    
    while True:
        if active_cam == 0 :
            ret, frame = cap0.read()
        else :
            ret, frame = cap1.read()
            
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        height, width, _ = frame.shape
        
        # Center point
        cx = int(width / 2)
        cy = int(height / 2)
        
        cv2.line(frame, (int(width/3), 0), (int(width/3), height), (200, 120, 0), 1)
        cv2.line(frame, (int(width/3)*2, 0), (int(width/3)*2, height), (200, 120, 0), 1)
        
        cv2.line(frame, (0, int(height / 3)), (width, int(height / 3)), (200, 120, 0), 1)
        cv2.line(frame, (0, int(height / 3)*2), (width, int(height / 3)*2), (200, 120, 0), 1)
        cv2.circle(frame, (cx, cy), 5, (225, 25, 25), 3)

        # Range warna mask
        lower_red1 = np.array([166, 80, 90])
        upper_red1 = np.array([179, 255, 255])
        mask_red1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        
        lower_red2 = np.array([0, 80, 90])
        upper_red2 = np.array([6, 255, 255])
        mask_red2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        lower_orange = np.array([7, 121, 120])
        upper_orange = np.array([23, 255, 255])
        mask_orange = cv2.inRange(hsv_frame, lower_orange, upper_orange)

        lower_blue = np.array([90, 121, 120])
        upper_blue = np.array([125, 255, 255])
        mask_blue = cv2.inRange(hsv_frame, lower_blue, upper_blue)
        
        # mask = cv2.bitwise_or(mask_red, mask_orange)
        # mask = cv2.bitwise_or(mask, mask_blue)
        
        #detect warna dan shape 
        def detect_and_label(mask, color_label, frame):
            centered = False
            orange_proceed = False
            center_threshold = 20
            normal_altitude = 2
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area > 2500:  # Luas minimal area yg di detect dalam pixel, cek area lewat mask
                    peri = cv2.arcLength(c, True)
                    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # Center point objek 
                    object_cx = x + w // 2
                    object_cy = y + h // 2
                    cv2.circle(frame, (object_cx, object_cy), 5, (0, 0, 255), 2)

                    # Draw a line from the main center to the object center
                    cv2.line(frame, (cx, cy), (object_cx, object_cy), (255, 255, 0), 2)

                    # Calculate the distance between the main center and the object center
                    distance = int((math.sqrt((object_cx - cx) ** 2 + (object_cy - cy) ** 2)) / 10)
                    cv2.putText(frame, f"Distance: {distance}", (object_cx + 10, object_cy - 10), cv2.FONT_HERSHEY_COMPLEX, 0.7, (113, 204, 46), 2)

                    if abs(object_cx - cx) < center_threshold and abs(object_cy - cy) < center_threshold:
                        centered = True
                    
                    # Label the detected shape
                    shape_label = "Undefined shape"
                    if len(approx) >= 6:
                        shape_label = "Circle"
                    elif len(approx) == 4:
                        shape_label = "Rectangle"
                    else:
                        shape_label = "Undefined shape"
 
                    # Draw the shape label
                    cv2.putText(frame, shape_label, (x + w + 20, y + h + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
                    # Draw the color label below the shape label
                    cv2.putText(frame, f"Color: {color_label}", (x + w + 20, y + h + 70), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"Points : {len(approx)}", (x + w + 20, y + h + 100), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)

                    # Determine navigation direction
                    nilaikiri = distance
                    nilaikanan = distance
                    nilaidepan = distance
                    nilaibelakang = distance
                    
                    if shape_label == "Circle" and (color_label == "ORANGE" or color_label=="RED"):
                        while not centered:
                            direction = ""
                            if object_cx < width / 3:
                                direction = "Go Left"
                                mavarm3_2.rcover((1500-nilaikiri),1500,1557,1500,0,0,0,0)
                            elif object_cx > (width / 3) * 2:
                                direction = "Go Right"
                                mavarm3_2.rcover((1500+nilaikanan),1500,1557,1500,0,0,0,0)
                            elif object_cy < height / 3:
                                direction = "Forward"
                                mavarm3_2.rcover((1500+nilaidepan),1500,1557,1500,0,0,0,0)
                            elif object_cy > (height / 3) * 2:
                                direction = "Backward"
                                mavarm3_2.rcover((1500-nilaibelakang),1500,1557,1500,0,0,0,0)
                            else :
                                centered = True
                                mavarm3_2.rcover(1500,1500,1550,1500,0,0,0,0)

                            if direction: #Ngasih tulisan direction
                                cv2.putText(frame, direction, (x + w + 20, y + h + 120), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 255), 2)
                        
                        while centered :
                            current_altitude = mavarm3_2.bottom_distance
                            if current_altitude > 0.6:
                                mavarm3_2.rcover(1500,1500,1480,1500,0,0,0,0)
                                cv2.putText(frame, "Lowering altitude", (x + w + 20, y + h + 120), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,0,255), 2)
                                current_altitude = mavarm3_2.bottom_distance
                            
                            if current_altitude <= 0.6:
                                mavarm3_2.rcover(1500,1500,1557,1500,0,0,0,0) #hovering sebentar
                                time.sleep(4)
                                mavarm3_2.rcover(1500,1500,1590,1500,0,0,0,0) #naik
                                centered = False
                                break
                                    
                    elif shape_label=="Rectangle" and (color_label =="RED" or color_label=="BLUE"):
                        while not centered:
                            direction = ""
                            if object_cx < width / 3:
                                direction = "Go Left"
                                mavarm3_2.rcover((1500-nilaikiri),1500,1557,1500,0,0,0,0)
                            elif object_cx > (width / 3) * 2:
                                direction = "Go Right"
                                mavarm3_2.rcover((1500+nilaikanan),1500,1557,1500,0,0,0,0)
                            elif object_cy < height / 3:
                                direction = "Forward"
                                mavarm3_2.rcover((1500+nilaidepan),1500,1557,1500,0,0,0,0)
                            elif object_cy > (height / 3) * 2:
                                direction = "Backward"
                                mavarm3_2.rcover((1500-nilaibelakang),1500,1557,1500,0,0,0,0)
                            else :
                                centered = True
                                mavarm3_2.rcover(1500,1500,1550,1500,0,0,0,0)

                            if direction: #Ngasih tulisan direction
                                cv2.putText(frame, direction, (x + w + 20, y + h + 120), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 255), 2)
                        
                        while centered :
                            current_altitude = mavarm3_2.bottom_distance
                            if current_altitude > 1:
                                mavarm3_2.rcover(1500,1500,1480,1500,0,0,0,0)
                                cv2.putText(frame, "Lowering altitude", (x + w + 20, y + h + 120), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,0,255), 2)
                            
                            if current_altitude <= 1:
                                mavarm3_2.rcover(1500,1500,1557,1500,0,0,0,0) #hovering sebentar
                                time.sleep(4)
                                mavarm3_2.rcover(1500,1500,1590,1500,0,0,0,0) #naik
                                time.sleep(3)
                                break
                            
                    
        # Panggil fungsi untuk setiap warna
        detect_and_label(mask_red, "RED", frame)
        detect_and_label(mask_orange, "ORANGE", frame)
        detect_and_label(mask_blue, "BLUE", frame)
        
        cv2.imshow("Frame", frame)
        # cv2.imshow("Mask", mask)
        
        key = cv2.waitKey(1)
        if key == 27:  # ESC key to exit
            break

    cap0.release()
    cap1.release()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()
