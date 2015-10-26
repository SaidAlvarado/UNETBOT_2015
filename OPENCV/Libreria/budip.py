__author__ = 'Said'

import cv2
import numpy as np
import warnings
import sys
import time

def nothing(x):
    pass


imagen = cv2.imread('foto_de_la_torre.png')


# print "{}x{}x{}".format(frame.shape[0], frame.shape[1], frame.shape[2])

base = np.zeros((155, 700), dtype = np.uint8)


cv2.rectangle(base, (0,30), (700,95), (255,255,255), -1)
cv2.rectangle(base, (30,0), (70,30), (255,255,255), -1)
cv2.rectangle(base, (630,0), (670,30),(255,255,255), -1)
cv2.rectangle(base, (300,95), (400,155),(255,255,255), -1)
print base.shape

last_time = time.time()

res = cv2.matchTemplate(imagen[:,:,0],base,cv2.TM_CCOEFF_NORMED)
min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

print("tiempo = {}ms".format(1000*(time.time() - last_time)))

cv2.rectangle(imagen, max_loc, ( max_loc[0] + base.shape[1],max_loc[1] + base.shape[0]) ,(0,0,255), 3)


while(1):

    cv2.imshow('template',base)
    cv2.imshow('image',imagen)
    cv2.imshow('resultado',res)
    #time.sleep(0.2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()


