import cv2
import numpy as np
from picamera2 import Picamera2
import time

class Camera:
    def __init__(self, min_radius=10, max_radius=100, detection_params=None):
        self.cam = Picamera2()
        config = self.cam.create_video_configuration(main={"format": 'RGB888', "size": (640, 480)})
        self.cam.configure(config)
        self.cam.start()
        time.sleep(1)  # Allow the camera to warm up
        
        # Circle detection parameters
        self.min_radius = min_radius
        self.max_radius = max_radius
        
        # Default HoughCircles parameters (can be tuned)
        if detection_params is None:
            self.detection_params = {
                'dp': 1,                    # Inverse ratio of accumulator resolution
                'min_dist': 50,             # Minimum distance between circle centers
                'param1': 100,              # Upper threshold for edge detection
                'param2': 30,               # Accumulator threshold for center detection
                'min_radius': min_radius,
                'max_radius': max_radius
            }
        else:
            self.detection_params = detection_params
            
        # Frame dimensions for normalization
        self.frame_width = 640
        self.frame_height = 480

    def preprocess_frame(self, frame):
        """
        Preprocess the frame for better circle detection
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        # Optional: Apply histogram equalization for better contrast
        # equalized = cv2.equalizeHist(blurred)
        
        return blurred

    def detect_circles_hough(self, processed_frame):
        """
        Detect circles using HoughCircles
        """
        circles = cv2.HoughCircles(
            processed_frame,
            cv2.HOUGH_GRADIENT,
            dp=self.detection_params['dp'],
            minDist=self.detection_params['min_dist'],
            param1=self.detection_params['param1'],
            param2=self.detection_params['param2'],
            minRadius=self.detection_params['min_radius'],
            maxRadius=self.detection_params['max_radius']
        )
        
        return circles

    def detect_circles_contour_based(self, frame):
        """
        Alternative method: Detect circles using contour analysis
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        # Apply adaptive threshold
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
        )
        
        # Morphological operations to clean up the image
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        circles = []
        for contour in contours:
            # Filter by area
            area = cv2.contourArea(contour)
            if area < np.pi * self.min_radius**2 or area > np.pi * self.max_radius**2:
                continue
            
            # Calculate circularity
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
                
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            # Filter by circularity (closer to 1.0 = more circular)
            if circularity > 0.7:  # Adjust threshold as needed
                # Get the enclosing circle
                (x, y), radius = cv2.minEnclosingCircle(contour)
                
                # Additional check: radius should be reasonable
                if self.min_radius <= radius <= self.max_radius:
                    circles.append((x, y, radius))
        
        return circles

    def get_ball_position(self, use_contour_method=False):
        """
        Detect ball position using shape-based detection
        """
        try:
            # Capture a frame from the camera
            frame = self.cam.capture_array()
            
            detected_circles = []
            
            if use_contour_method:
                # Use contour-based detection
                circles = self.detect_circles_contour_based(frame)
                detected_circles = circles
            else:
                # Use HoughCircles detection
                processed_frame = self.preprocess_frame(frame)
                circles = self.detect_circles_hough(processed_frame)
                
                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")
                    detected_circles = [(x, y, r) for x, y, r in circles]

            if detected_circles:
                # If multiple circles detected, choose the largest one
                best_circle = max(detected_circles, key=lambda c: c[2])  # Sort by radius
                x, y, radius = best_circle
                
                # Additional validation: check if the circle is reasonable
                if radius >= self.min_radius and radius <= self.max_radius:
                    # Normalize coordinates from -1 to 1
                    x_norm = 2 * (x / self.frame_width) - 1
                    y_norm = 2 * (y / self.frame_height) - 1
                    
                    # Clamp to valid range
                    x_norm = max(-1, min(1, x_norm))
                    y_norm = max(-1, min(1, y_norm))
                    
                    return [x_norm, y_norm]
            
            return [0, 0]  # Default if no ball found
        
        except Exception as e:
            print(f"Camera error: {e}")
            return [0, 0]

    def get_ball_position_with_debug(self, use_contour_method=False, show_detection=False):
        """
        Enhanced version that can show detection results for debugging
        """
        try:
            frame = self.cam.capture_array()
            debug_frame = frame.copy()
            
            detected_circles = []
            
            if use_contour_method:
                circles = self.detect_circles_contour_based(frame)
                detected_circles = circles
            else:
                processed_frame = self.preprocess_frame(frame)
                circles = self.detect_circles_hough(processed_frame)
                
                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")
                    detected_circles = [(x, y, r) for x, y, r in circles]

            # Draw all detected circles for debugging
            if show_detection:
                for (x, y, r) in detected_circles:
                    cv2.circle(debug_frame, (int(x), int(y)), int(r), (0, 255, 0), 2)
                    cv2.circle(debug_frame, (int(x), int(y)), 2, (0, 0, 255), 3)

            if detected_circles:
                best_circle = max(detected_circles, key=lambda c: c[2])
                x, y, radius = best_circle
                
                if radius >= self.min_radius and radius <= self.max_radius:
                    if show_detection:
                        # Highlight the selected circle
                        cv2.circle(debug_frame, (int(x), int(y)), int(radius), (255, 0, 0), 3)
                        print(f"Ball detected at ({x:.1f}, {y:.1f}) with radius {radius:.1f}")
                    
                    x_norm = 2 * (x / self.frame_width) - 1
                    y_norm = 2 * (y / self.frame_height) - 1
                    x_norm = max(-1, min(1, x_norm))
                    y_norm = max(-1, min(1, y_norm))
                    
                    if show_detection:
                        cv2.imshow("Ball Detection", debug_frame)
                        cv2.waitKey(1)
                    
                    return [x_norm, y_norm]
            
            if show_detection:
                cv2.imshow("Ball Detection", debug_frame)
                cv2.waitKey(1)
                print("No ball detected")
            
            return [0, 0]
        
        except Exception as e:
            print(f"Camera error: {e}")
            return [0, 0]

    def update_detection_params(self, **kwargs):
        """
        Update detection parameters during runtime
        """
        for key, value in kwargs.items():
            if key in self.detection_params:
                self.detection_params[key] = value
                print(f"Updated {key} to {value}")

    def set_radius_range(self, min_radius, max_radius):
        """
        Update the radius range for detection
        """
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.detection_params['min_radius'] = min_radius
        self.detection_params['max_radius'] = max_radius
        print(f"Updated radius range to [{min_radius}, {max_radius}]")

    # Compatibility methods
    def set_up(self):
        """Compatibility method - camera is already set up in __init__"""
        pass
    
    def clean_up(self):
        """Alternative name for close() method"""
        self.close()

    def close(self):
        """Stop the camera and cleanup"""
        try:
            self.cam.stop()
            cv2.destroyAllWindows()  # Close any debug windows
        except Exception as e:
            print(f"Error closing camera: {e}")