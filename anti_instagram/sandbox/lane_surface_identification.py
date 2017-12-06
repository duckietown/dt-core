#!/usr/bin/env python
import argparse
import os
import os.path
import shutil

import cv2
import matplotlib.pyplot as plt
import numpy as np


def identify_lane_surface(img, use_hsv=False, visualize=False):
    h, w = img.shape[:2]
    img_for_grad = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) if use_hsv else img
    dx = cv2.Sobel(img_for_grad, cv2.CV_32F, 1, 0)
    dy = cv2.Sobel(img_for_grad, cv2.CV_32F, 0, 1)
    grad = (np.sqrt(np.mean(dx**2 + dy**2, 2)) > 30).astype(np.uint8)
    mask = np.zeros((h + 2, w + 2), np.uint8)
    y = int(h / 5.0 * 3)
    for x in range(w):
        if not (grad[y, x] or mask[y + 1, x + 1]):
            cv2.floodFill(grad, mask, (x, y), 0,
                          flags=cv2.FLOODFILL_MASK_ONLY)
    mask[y + 1:, :] = 1
    mask[0, :] = 0
    mask = mask[:, 1:-1]
    mask_ = np.zeros((h + 4, w + 2), np.uint8)
    cv2.floodFill(mask, mask_, (0, 0), 0, flags=cv2.FLOODFILL_MASK_ONLY)
    mask = 1 - mask_[2:-2, 1:-1]
    processed_img = img * np.tile(mask[..., np.newaxis], (1, 1, 3))
    if visualize:
        plt.subplot(2, 2, 1)
        plt.imshow(img[..., ::-1])
        plt.subplot(2, 2, 2)
        plt.imshow(grad, cmap='gray')
        plt.subplot(2, 2, 3)
        plt.imshow(mask, cmap='gray')
        plt.subplot(2, 2, 4)
        plt.imshow(processed_img[..., ::-1])
        plt.get_current_fig_manager().full_screen_toggle()
        plt.show()
    return processed_img, mask


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
        processed_img, _ = identify_lane_surface(img, visualize=False)
        cv2.imwrite(os.path.join(args.output_dir, '{}_{}'.format(
            base_name, extension)), processed_img)
