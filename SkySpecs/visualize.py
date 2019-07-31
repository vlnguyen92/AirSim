import numpy as np
import re
import cv2
import argparse
import matplotlib.pyplot as plt


def load_pfm(file):
    color = None
    width = None
    height = None
    scale = None
    data_type = None
    header = str(file.readline()).rstrip()

    if header == 'PF':
        color = True
    elif header == 'Pf':
        color = False
    else:
        raise Exception('Not a PFM file.')
    dim_match = re.match(r'^(\d+)\s(\d+)\s$', file.readline())
    if dim_match:
        width, height = map(int, dim_match.groups())
    else:
        raise Exception('Malformed PFM header.')
    # scale = float(file.readline().rstrip())
    scale = float((file.readline()).rstrip())
    if scale < 0: # little-endian
        data_type = '<f'
    else:
        data_type = '>f' # big-endian
    data_string = file.read()
    data = np.fromstring(data_string, data_type)
    # data = np.fromfile(file, data_type)
    shape = (height, width, 3) if color else (height, width)
    data = np.reshape(data, shape)
    data = cv2.flip(data, 0)
    return data

if __name__ == '__main__':
    """
        Based on code from: https://github.com/YoYo000/MVSNet/blob/master/mvsnet/visualize.py
    """

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--depth_map', help="Path to depth map")

    args = parser.parse_args()

    depth_image = load_pfm(open(args.depth_map, 'rb'))

    ma = np.ma.masked_equal(depth_image, 0.0, copy=False)

    print('value range: {}, {}'.format(ma.min(), ma.max()))

    plt.imshow(depth_image, 'rainbow')
    plt.show()