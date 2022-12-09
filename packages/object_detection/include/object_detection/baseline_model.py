from .model import Model
import numpy as np
import cv2
from matplotlib import pyplot as plt


class BaseModel(Model):
    def __init__(self, 
        sigma=6
    ):
        self.sigma = sigma
    
    def infer(self, imagebrg, debug=False):    

        imghsv = cv2.cvtColor(imagebrg, cv2.COLOR_BGR2HSV)
        img_gaussian_filter = cv2.GaussianBlur(imghsv,(0,0), self.sigma)

        #* Simple yellow color filter
        yellow_lower_hsv = np.array([15, 100, 100])
        yellow_upper_hsv = np.array([40, 255, 255]) 
        mask_yellow = cv2.inRange(img_gaussian_filter, yellow_lower_hsv, yellow_upper_hsv)

        ret, thresh = cv2.threshold(mask_yellow, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, 1, 2)
        
        if debug: 
            return contours, mask_yellow

        return contours, [0 for _ in range(len(contours))], 0.50    


if __name__=="__main__":
    model = BaseModel()

    imgbgr = cv2.imread('solution/images/visual_control/pic1_rect.png')

    contours, mask = model.infer(imgbgr, debug=True)

    fig = plt.figure(figsize = (20,10))
    ax1 = fig.add_subplot(1,3,1)
    ax1.imshow(imgbgr)
    ax1.set_title('Original'), ax1.set_xticks([]), ax1.set_yticks([])

    ax2 = fig.add_subplot(1,3,2)
    ax2.imshow(imgbgr)
    ax2.imshow(mask, cmap='jet', alpha=0.5)
    ax2.set_title('Filtered by Color (Yellow)'), ax2.set_xticks([]), ax2.set_yticks([]);

    result = imgbgr

    for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            result = cv2.rectangle(result, (x,y), (x+w,y+h), (255,10,10), 2)

    ax3 = fig.add_subplot(1,3,3)
    ax3.imshow(result)
    ax3.set_title('Bonding bx applied on mask'), ax3.set_xticks([]), ax3.set_yticks([]);

    fig.savefig("simple-model-result.png")

