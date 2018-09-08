#!/usr/bin/env python
import argparse
import os
import os.path
import shutil

import cv2
import matplotlib.pyplot as plt
import numpy as np

from geom import identifyLaneSurface

if __name__ == '__main__':
    ap = argparse.ArgumentParser(
        description='Given the directory of duckietown images, identify the horizon for each image.')
    ap.add_argument(
        'img_dir', help='path to the directory of duckietown images')
    ap.add_argument(
        '--output_dir', default='output_dir', help='output directory for the processed images')
    args = ap.parse_args()
    if os.path.exists(args.output_dir):
        shutil.rmtree(args.output_dir, ignore_errors=True)
    os.makedirs(args.output_dir)
    for file_name in os.listdir(args.img_dir):
        base_name, extension = os.path.splitext(file_name)
        if not extension in {'.png', '.jpg'}:
            continue
        img = cv2.imread(os.path.join(args.img_dir, file_name))
        processed_img, _ = identifyLaneSurface(img, visualize=False)
        cv2.imwrite(os.path.join(args.output_dir, '{}_{}'.format(
            base_name, extension)), processed_img)
