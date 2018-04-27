#!/usr/bin/env python
import cv2
import numpy as np
import math

class simpleColorBalanceClass:
    # initialize
    def __init__(self):
        self.ThLow = np.zeros(3, np.int16)
        self.ThLow.fill(-1)
        self.ThHi = np.zeros(3, np.int16)
        self.ThHi.fill(-1)
        self.halfPercent = -1
        print('Instance of simpleColorBalanceClass created.')

    def apply_mask(self, matrix, mask, fill_value):
        masked = np.ma.array(matrix, mask=mask, fill_value=fill_value)
        return masked.filled()

    def apply_threshold(self, matrix, low_value, high_value):
        low_mask = matrix < low_value
        matrix = self.apply_mask(matrix, low_mask, low_value)

        high_mask = matrix > high_value
        matrix = self.apply_mask(matrix, high_mask, high_value)

        return matrix

    def thresholdAnalysis(self, img, percent):
        #assert img.shape[2] == 3
        #assert percent > 0 and percent < 100

        self.halfPercent = percent / 200.0
        channels = cv2.split(img)

        for idx, channel in enumerate(channels):
            #assert len(channel.shape) == 2
            # find the low and high precentile values (based on the input percentile)
            height, width = channel.shape
            vec_size = width * height
            flat = channel.reshape(vec_size)

            #assert len(flat.shape) == 1

            flat = np.sort(flat)

            n_cols = flat.shape[0]

            self.ThLow[idx] = flat[int(math.floor(n_cols * self.halfPercent))]
            self.ThHi[idx] = flat[int(math.ceil(n_cols * (1.0 - self.halfPercent)))]

        return self.ThLow, self.ThHi


    def applyTrafo(self, img, ThLow = [], ThHi = []):
        #for i in range(3):
        #    assert(self.ThHi[i] >= 0)
        #    assert(self.ThLow[i] >= 0)

        if ThLow == [] and ThHi == []:
            ThLow = self.ThLow
            ThHi = self.ThHi

        channels = cv2.split(img)
        out_channels = []

        for idx, channel in enumerate(channels):
            #assert len(channel.shape) == 2
            # saturate below the low percentile and above the high percentile
            thresholded = self.apply_threshold(channel, ThLow[idx], ThHi[idx])
            # scale the channel
            normalized = cv2.normalize(thresholded, thresholded.copy(), 0, 255, cv2.NORM_MINMAX)
            out_channels.append(normalized)

        return cv2.merge(out_channels)