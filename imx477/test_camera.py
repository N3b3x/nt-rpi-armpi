#!/usr/bin/env python3
# encoding:utf-8
import sys
import cv2
import time
from Camera import Camera

def main():
    print("Testing IMX477 camera...")
    print("Initializing camera with resolution 640x480...")
    camera = Camera(resolution=(640, 480))
    
    frame_count = 0
    start_time = time.time()
    
    try:
        while True:
            frame = camera.get_frame()
            if frame is not None:
                frame_count += 1
                elapsed_time = time.time() - start_time
                if elapsed_time >= 1.0:  # Print FPS every second
                    fps = frame_count / elapsed_time
                    print(f"FPS: {fps:.2f}")
                    frame_count = 0
                    start_time = time.time()
                
                cv2.imshow('IMX477 Test', frame)
                key = cv2.waitKey(1)
                if key == 27:  # ESC key
                    break
            else:
                print("No frame received")
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        print("Closing camera...")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
