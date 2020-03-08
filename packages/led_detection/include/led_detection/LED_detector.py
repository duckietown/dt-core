import numpy as np
from scipy import fftpack
import cv2

class LEDDetector:
    def __init__(self, parameters, logger):
        self.log = logger
        self.parameters = parameters
        # create the detector object
        self.detector = {}

        self.update_parameters(self.parameters)

    def update_parameters(self, new_parameters):
        """Updates parameters."""
        self.parameters = new_parameters
        bd_param_db = cv2.SimpleBlobDetector_Params()
        bd_param_tl = cv2.SimpleBlobDetector_Params()

        # Assign values to object variables
        for key, val in self.parameters['~blob_detector_db'].items():
            setattr(bd_param_db, key, val)
        for key, val in self.parameters['~blob_detector_tl'].items():
            setattr(bd_param_tl, key, val)

        # Create a detector with the parameters
        self.detector['car'] = cv2.SimpleBlobDetector_create(bd_param_db)
        self.detector['tl'] = cv2.SimpleBlobDetector_create(bd_param_tl)

    def find_blobs(self, images, target):

        blobs = []
        frames = []

        # Iterate over the sequence of images
        for t, img in enumerate(images):
            frame = []
            keypoints = self.detector[target].detect(img)

            for kp in keypoints:
                kp_coords = kp.pt
                frame.append(kp_coords)

                if len(blobs) == 0:
                    # If no blobs saved, then save the first LED detected
                    blobs.append({'p': kp_coords, 'N': 1, 'Signal': np.zeros(images.shape[2])})
                    blobs[-1]['Signal'][t] = 1

                else:
                    # Thereafter, check whether the detected LED belongs to a blob by pixel distance
                    dist_kp_to_blobs = [np.linalg.norm(blob['p'] - kp_coords) for blob in blobs]

                    if np.min(dist_kp_to_blobs) < self.parameters['~DTOL']:
                        idx_closest = int(np.argmin(dist_kp_to_blobs))
                        if blobs[idx_closest]['Signal'][t] == 0:
                            blobs[idx_closest]['N'] += 1
                            blobs[idx_closest]['Signal'][t] = 1
                    else:
                        # Its a new one
                        blobs.append({'p': kp_coords, 'N': 1, 'Signal': np.zeros(images.shape[2])})
                        blobs[-1]['Signal'][t] = 1

            frames.append(frame)

        return blobs, frames

    def interpret_signal(self, blobs, t_s, num_img):
        # Semantically decide if blobs represent a known signal
        for blob in blobs:
            # Detection
            detected, freq_identified, fft_peak_freq = self.examine_blob(blob, t_s, num_img)

            # Take decision
            detected_signal = None
            if detected:
                if self.parameters['~verbose'] == 2:
                    msg = '\n-------------------\n' + \
                          'num_img = %d \n' % num_img + \
                          't_samp = %f \n' % t_s + \
                          'fft_peak_freq = %f \n' % fft_peak_freq + \
                          'freq_identified = %f \n' % freq_identified + \
                          '-------------------'
                    self.log(msg)

                for signal_name, signal_value in self.parameters['~LED_protocol']['signals'].items():
                    if signal_value['frequency'] == freq_identified:
                        detected_signal = signal_name
                    break

            return detected_signal

    def examine_blob(self, blob, t_s, num_img):
        """Detects if a blob is blinking at specific frequencies."""
        # Percentage of appearance
        appearance_percentage = (1.0 * blob['N']) / (1.0 * num_img)

        # Frequency estimation based on FFT
        signal_f = fftpack.fft(blob['Signal'] - np.mean(blob['Signal']))
        y_f = 2.0 / num_img * np.abs(signal_f[:num_img / 2 + 1])
        fft_peak_freq = 1.0 * np.argmax(y_f) / (num_img * t_s)

        if self.parameters['~verbose'] == 2:
            self.log('Appearance perceived. = %s, frequency = %s' %
                     (appearance_percentage, fft_peak_freq))
        freq_identified = None
        # Take decision
        detected = False
        freq_to_identify = self.parameters['~LED_protocol']['frequencies'].values()
        for freq in freq_to_identify:
            if abs(fft_peak_freq - freq) < 0.35:
                # Decision
                detected = True
                freq_identified = freq
                break

        return detected, freq_identified, fft_peak_freq

    @staticmethod
    def get_keypoints(blobs, radius):
        # Extract blobs
        keypoint_blob = []
        for blob in blobs:
            assert np.sum(blob['Signal']) == blobs['N']
            keypoint_blob.append(cv2.KeyPoint(blob['p'][0], blob['p'][1], radius))
        return keypoint_blob
