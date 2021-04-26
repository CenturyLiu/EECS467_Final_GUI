# importing libraries
import cv2
import numpy as np
import time
import winsound

width=640

def playVideo(filename):
    # Create a VideoCapture object and read from input file
    cap = cv2.VideoCapture('../video/free_style_demo_3.mp4')
    
    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video  file")
    
    # Read until video is completed
    while(cap.isOpened()):
        
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret == True:
        
            h, w, _ = frame.shape
            if w >= width:
                height = int((width/w)*h)
            else:
                height = h
            #calculate the 50 percent of original dimensions
            dsize = (width, height)
            # resize image
            output = cv2.resize(frame, dsize)
    
            # Display the resulting frame
            cv2.imshow('Sample Motion', output)
        
            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
        
        # Break the loop
        else: 
            break
    
    # When everything done, release 
    # the video capture object
    cap.release()
    
    # Closes all the frames
    cv2.destroyAllWindows()

if __name__ == "__main__":
    time.sleep(10)
    filename = '../sounds/free_style_demo_3.wav'
    winsound.PlaySound(filename, winsound.SND_FILENAME | winsound.SND_ASYNC)
    print("play one sound")
    playVideo("freestyle_demo.mp4")