#!/usr/bin/env python

import argparse
import os
import os.path
import shutil

import cv2
import duckietown_utils
import numpy as np
import numpy.random
import rosbag

# Example usage: ./sample_images.py path_to_bag /scbb/camera_rectifier/image/compressed 500

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Sample images from ROS Bags.')
    parser.add_argument('bag_path', help='path to the ROS Bag')
    parser.add_argument('topic', help='the topic for image sampling')
    parser.add_argument(
        'n_imgs', help='the approximate number of images to extract')
    parser.add_argument('--output_dir', default='./sampled_images',
                        help='output directory for the sampled images')
    args = parser.parse_args()
    if os.path.exists(args.output_dir):
        shutil.rmtree(args.output_dir, ignore_errors=True)
    os.makedirs(args.output_dir)
    rb = rosbag.Bag(args.bag_path)
    n_imgs = float(args.n_imgs)
    p = n_imgs / rb.get_message_count(args.topic)
    img_no = 0
    for msg in rb.read_messages(args.topic):
        if numpy.random.rand() < p:
            cv2.imwrite(os.path.join(args.output_dir, 'sample_{{:0{}d}}.png'.format(int(np.floor(
                np.log10(n_imgs))) + 1).format(img_no)), duckietown_utils.rgb_from_ros(msg.message)[..., ::-1])
            img_no += 1
