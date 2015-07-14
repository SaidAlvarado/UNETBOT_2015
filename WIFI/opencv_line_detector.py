import numpy as np
import cv2

cap = cv2.VideoCapture("udpsrc port=5000 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false")

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Our operations on the frame come here
    # Pasamos la imagen a blanco y negro
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Calculamos la derivada en x de la imagen.
    # Operador de Sobel
    sobelx = cv2.Sobel(gray,cv2.CV_64F,1,0,ksize=3)
    sobelx = np.uint8(np.absolute(sobelx))


    #Unimos las imagenes para hacer un display simultaneo y rescalamos
    #resultado = np.hstack((sobelx,scharr))
    #resultado = cv2.resize(resultado,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_AREA)
    resultado = sobelx

    # Display the resulting frame
    cv2.imshow('frame',resultado)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

