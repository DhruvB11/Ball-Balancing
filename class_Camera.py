import cv2 as cv
import numpy as np
from picamera2 import Picamera2
import time

class Camera:
    def _init_(self):
        self.cam = Picamera2()
        config = self.cam.create_video_configuration(main={"format": 'RGB888', "size": (640, 480)})
        self.cam.configure(config)
        self.cam.start()
        time.sleep(1)  # Allow the camera to warm up
    
    def get_ball_position(self):
        img = self.cam.capture_array()
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        _, thresh = cv.threshold(gray, 190, 255, cv.THRESH_BINARY)
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        clean = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
        contours, _ = cv.findContours(clean, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # Looping to find the best circular blob
        best_contour = None
        best_score = 0

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area < 500:  # Ignoring tiny contours
                continue
            perimeter = cv.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)

            if circularity > 0.75 and area > best_score:
                best_score = area
                best_contour = cnt

        # THis is to get the best match
        # Remove the comments, if you want to visualize the circle and show the output for a frame
        # output = img.copy()
        if best_contour is not None:
            # cv.drawContours(output, [best_contour], -1, (0, 255, 0), 3)
            x, y, w, h = cv.boundingRect(best_contour)
            center = [x + w // 2, y + h // 2]
            # cv.circle(output, center, 5, (0, 0, 255), -1)
            # cv.putText(output, "Circle", (center[0] - 30, center[1] - 10), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            # cv.imshow('img',output)
            # cv.waitKey(0)
            # cv.destroyAllWindows()
            return center
        else:
            return [0, 0] #prev. output ?? , This case should never happen, but in case

    def close(self):
        self.cam.stop()
