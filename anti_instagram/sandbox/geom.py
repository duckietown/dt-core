#!/usr/bin/env python
import cv2
import numpy as np
import sys

def processGeom(img):
    #img = cv2.resize(img, (img.shape[0]/3, img.shape[0]/3),interpolation=cv2.INTER_NEAREST)
    top_cutoff = img.shape[0]/3
    img = img[top_cutoff:,:,:]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    colors = ['white','yellow','red']
    disp = np.zeros((hsv.shape[0],hsv.shape[1],3),dtype=np.uint8)
    masks = {}
    cmasks = {}
    disp2 = np.zeros_like(img)
    for color in colors:
    	bw = colorFilter(hsv,color)
    	#masks[color] = detectColor(bw,color)
	contours = detectColor2(bw)
	#masks[color] = cv2.cvtColor(masks[color], cv2.COLOR_GRAY2BGR)
	if color=='white':
	    #masks[color] *= np.array([255,255,255], dtype=np.uint8)
	    col = (255,255,255)
	if color=='yellow':
	    #masks[color] *= np.array([0,255,255], dtype=np.uint8)
	    col = (0,255,255)
	if color=='red':
	    #masks[color] *= np.array([0,0,255], dtype=np.uint8)
	    col = (0,0,255)
	#disp += masks[color]
	cmasks[color] = np.zeros(img.shape, dtype=np.uint8)
	cv2.drawContours(cmasks[color],contours,-1,255,-1)
	cmasks[color] = np.where(cmasks == 255)
	cv2.drawContours(disp2,contours,-1,col,-1)

    return disp, disp2

#taken straight out of line_detector1.py
def colorFilter(img, color):
    # threshold colors in HSV space
    '''TODO: use the previous kmeans estimate +- some range to be the threshold colors'''
    '''TODO: instead of thresholding for 3 colors (poor performance on pic1, pic3) threshold for the entire white to red to yellow range'''
    hsv_white1 = np.array([0,0,150])
    hsv_white2 = np.array([180,60,255])
    hsv_yellow1 = np.array([25,140,100])
    hsv_yellow2 = np.array([45,255,255])
    hsv_red1 = np.array([0,140,100])
    hsv_red2 = np.array([15,255,255])
    hsv_red3 = np.array([165,140,100])
    hsv_red4 = np.array([180,255,255])

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
    #cv2.imshow('img',bw)
    #cv2.waitKey(0)

    return bw

#the assumption here is that our regions of interest are polygonal
def detectColor2(bw):
    im2, contours, hierarchy = cv2.findContours(bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = [contour for contour in contours if contour.shape[0] >= 30]
    return contours

#first idea, obsolete
def detectColor(bw, color):
    n = 20
    if color=='white':
	#white lines should be perpendicular to horizon
	w,h = 50,200
    elif color=='yellow':
	#yellow should come in patches, separated by gray space
	w,h = 80,80
    elif color=='red':
	#red lines are near parallel to the horizon
	w,h = 200,50
    else:
	raise Exception('Error: Undefined color strings...')
    w,h = 100,100
    r = contigRegion(bw,w,h,n)
    mask = np.zeros(bw.shape,dtype=np.uint8)
    for i in range(r.shape[0]):
	y,x = r[i,0],r[i,1]
        mask[y-h:y+h,x-w:x+w] = 1
    mask = cv2.bitwise_and(bw,mask)
    return mask

def contigRegion(bw,kx,ky,n):
    #kernel shape is based on some geometric primitive of lane (should be trapezoid instead of rectangle?)
    kernel = np.ones((ky,kx),np.float32)
    conv = cv2.filter2D(bw,-1,kernel)
    row, col = conv.shape[0], conv.shape[1]
    a = conv.reshape(row*col)
    #top n maximum indices
    a = a.argsort()[-n:][::-1]
    r = np.zeros((a.shape[0],2))
    r[:,0] = a//col + ky
    r[:,1] = a%col + kx
    return r

if __name__=='__main__':
    fname = sys.argv[1]
    img = cv2.imread(fname)
    disp, disp2 = processGeom(img)
    cv2.imshow('img',img)
    cv2.waitKey(0)
    cv2.imshow('img',disp)
    cv2.waitKey(0)
    cv2.imshow('img',disp2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
