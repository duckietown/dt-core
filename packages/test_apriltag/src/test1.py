#!/usr/bin/env python

import os
import yaml
import cv2
import time

from dt_apriltags import Detector


# load image and calibration file
data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')
image_file = os.path.join(data_dir, 'rectification_center', 'rect.jpg')
calib_file = os.path.join(data_dir, 'calibration', 'autobot02.yaml')
calibration = yaml.load(open(calib_file, 'rt'), yaml.SafeLoader)
image = cv2.imread(image_file, 0)

# create apriltag detector
detector = Detector(searchpath=['/code/catkin_ws/devel/lib/'])

stime = time.time()
detection = detector.detect(image)
ftime = time.time() - stime
print('Processed in %.5f secs' % ftime)

print(detection)

