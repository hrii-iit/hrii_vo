from PIL import Image, ImageDraw
import cv2
from picamera2 import Picamera2
import time
import socket
import numpy

UDP_IP = '192.168.0.54'  # replace with your PC's IP address
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (1280, 960)})
#config = picam2.create_preview_configuration()
picam2.configure(config)
picam2.start()
MAX_DGRAM_SIZE = 65507

time.sleep(1)
i = 0

#for i in range(43000):

while True:

    if i == 0:
        T_1 = time.time()
    elif i%30==0:
        T_2 = time.time()
        print(f"30 frames in {(T_2 - T_1)} second and the both camera image size is {im_rgb.shape}")
        T_1 = time.time()
    image = picam2.capture_image("main")
    
    open_cv_image = numpy.array(image) 
    im_rgb = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2RGB)
    data = cv2.imencode('.jpg', im_rgb)[1].tobytes()
    num_chunks = len(data) // MAX_DGRAM_SIZE + 1
    print("HERE")
    for i in range(num_chunks):
        chunk = data[i*MAX_DGRAM_SIZE : (i+1)*MAX_DGRAM_SIZE]

            # send chunk via UDP
        sock.sendto(chunk, (UDP_IP, UDP_PORT))
        print(f"Sent {len(data)} bytes to {UDP_IP}:{UDP_PORT}")
    #sock.sendto(data, (UDP_IP, UDP_PORT))
    # Convert RGB to BGR 
    #open_cv_image = open_cv_image[:, :, ::-1].copy()
    #cv2.imshow("BGR", im_rgb)
    #cv2.waitKey(1)
    i+=1
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break





 


