#!/usr/bin/env python
import cv2
import numpy as np

def processGeom(img):
    top_cutoff = 40
    img = cv2.resize(img, (img.shape[0]/3, img.shape[0]/3),interpolation=cv2.INTER_NEAREST)
    img = img[top_cutoff:,:,:]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    colors = ['white'.'yellow','red']
    for color in colors:
    	bw = colorFilter(hsv,color)
    	mask = detectColor(bw,color)


#taken straight out of line_detector1.py
def colorFilter(img, color):
    # threshold colors in HSV space
    hsv_white1 = [0,0,150]
    hsv_white2 = [180,60,255]
    hsv_yellow1 = [25,140,100]
    hsv_yellow2 = [45,255,255]
    hsv_red1 = [0,140,100]
    hsv_red2 = [15,255,255]
    hsv_red3 = [165,140,100]
    hsv_red4 = [180,255,255]

    if color == 'white':
        bw = cv2.inRange(img, hsv_white1, hsv_white2)
    elif color == 'yellow':
        bw = cv2.inRange(img, hsv_yellow1, hsv_yellow2)
    elif color == 'red':
        bw1 = cv2.inRange(img, hsv_red1, hsv_red2)
        bw2 = cv2.inRange(img, hsv_red3, hsv_red4)
        bw = cv2.bitwise_or(bw1, bw2)
    else:
        raise Exception('Error: Undefined color strings...')

    # binary dilation
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3, 3))
    bw = cv2.dilate(bw, kernel)

    return bw

def detectColor(img, color):
    n = 10
    if color=='white':
	#white lines should be perpendicular to horizon
	w,h = 20,50
    elif color=='yellow':
	#yellow should come in patches, separated by gray space
	w,h = 30,30
    elif color=='red':
	#red lines are near parallel to the horizon
	w,h = 50,20
    else:
	raise Exception('Error: Undefined color strings...')
    w,h = 100,100
    r = contigRegion(bw,w,h,n)
    mask = np.zeros(bw.shape)
    for i in r.shape[0]:
	y,x = r[i,0],r[i,1]
        mask[y-h:y+h,x-w:x+w] = 1
    mask = cv2.bitwise_and(bw,mask)
    return mask

def contigRegion(bw,kx,ky,n):
    #kernel shape is based on some geometric primitive of lane
    kernel = np.ones((ky,kx),np.float32)
    conv = cv2.filter2D(bw,-1,kernel)
    row, col = conv.shape[0], conv.shape[1]
    a = conv.reshape(row*col)
    #top n maximum indices
    a = a.argsort()[-n:][::-1]
    r = np.zeros((a.shape[0],2))
    r[:,0] = a//col + ky
    r[;,1] = a%col + kx
    return r

