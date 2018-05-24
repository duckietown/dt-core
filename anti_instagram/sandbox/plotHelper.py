import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from collections import Counter


def plotKMeansCenters(trained_centers, labels):
    # loop over all centers
    n_centers, n_channels = trained_centers.shape
    color_array = []
    color_image_array = np.zeros((n_centers, 200, 200, 3), np.uint8)
    labelcount = Counter()

    for center in np.arange(n_centers):
        # get color
        color_i = tuple(
            [trained_centers[center, 2], trained_centers[center, 1], trained_centers[center, 0]])
        color_array.append(color_i)
        color_image_array[center, :] = color_i
        labelcount[center] = np.sum(labels == center)

    plotRows = int(math.ceil(n_centers / 2.0))
    f, axarr = plt.subplots(plotRows, 2)
    for row in range(plotRows):
        if n_centers % 2 == 0:
            axarr[row, 0].imshow(color_image_array[2 * row])
            axarr[row, 0].axis('off')
            axarr[row, 0].set_title(str(labelcount[2 * row]))

            axarr[row, 1].imshow(color_image_array[2 * row + 1])
            axarr[row, 1].axis('off')
            axarr[row, 1].set_title(str(labelcount[2 * row + 1]))
        else:
            if row != plotRows - 1:
                axarr[row, 0].imshow(color_image_array[2 * row])
                axarr[row, 0].axis('off')
                axarr[row, 0].set_title(str(labelcount[2 * row]))

                axarr[row, 1].imshow(color_image_array[2 * row + 1])
                axarr[row, 1].axis('off')
                axarr[row, 1].set_title(str(labelcount[2 * row + 1]))
            else:
                axarr[row, 0].imshow(color_image_array[2 * row])
                axarr[row, 0].axis('off')
                axarr[row, 0].set_title(str(labelcount[2 * row]))

                axarr[row, 1].axis('off')
    print(color_array)
    plt.show()
    cv2.waitKey(0)
    # cv2.destroyAllWindows()


def plotDeterminedCenters(centerBlack, centerYellow, centerWhite, centerRed):
    tupleBlack = tuple([centerBlack[2], centerBlack[1], centerBlack[0]])
    tupleWhite = tuple([centerWhite[2], centerWhite[1], centerWhite[0]])
    tupleYellow = tuple([centerYellow[2], centerYellow[1], centerYellow[0]])
    tupleRed = tuple([centerRed[2], centerRed[1], centerRed[0]])

    imageBlack = np.zeros((200, 200, 3), np.uint8)
    imageBlack[:] = tupleBlack
    imageWhite = np.zeros((200, 200, 3), np.uint8)
    imageWhite[:] = tupleWhite
    imageYellow = np.zeros((200, 200, 3), np.uint8)
    imageYellow[:] = tupleYellow
    imageRed = np.zeros((200, 200, 3), np.uint8)
    imageRed[:] = tupleRed

    f, axarr = plt.subplots(2, 2)

    axarr[0, 0].imshow(imageBlack)
    axarr[0, 0].axis('off')
    axarr[0, 0].set_title("Black")

    axarr[0, 1].imshow(imageWhite)
    axarr[0, 1].axis('off')
    axarr[0, 1].set_title("White")

    axarr[1, 0].imshow(imageYellow)
    axarr[1, 0].axis('off')
    axarr[1, 0].set_title("Yellow")

    axarr[1, 1].imshow(imageRed)
    axarr[1, 1].axis('off')
    axarr[1, 1].set_title("Red")

    plt.show()
    cv2.waitKey(0)
    # cv2.destroyAllWindows()
