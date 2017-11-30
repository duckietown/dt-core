#!/usr/bin/env python
import sys
from sklearn.cluster import KMeans

input_img = cv2.imread("test_images/pic1.jpg", cv2.IMREAD_UNCHANGED)
kmc = KMeans(n_clusters=10)
kmc.fit_predict(input_img)
