import numpy as np
import matplotlib.pyplot as plt

def print_info(var):
    print(var.shape, var.dtype, 'min: {}'.format(np.min(var)), 'max: {}'.format(np.max(var)))
    print()

def main():
    data_path = '/home/nicolas/vrep_v4.0/'

    id = 1
    # data = np.loadtxt(data_path+'KinLog{}.txt'.format(id))
    data = np.loadtxt(data_path+'KinDepthLog{}_meters.txt'.format(id))

    # ----- Parsing Data ----- #
    # DepthCam(0=closest to sensor (i.e. close clipping plane), 1=farthest from sensor (i.e. far clipping plane))
    depthValue = data[:, 2]/100.0
    depthValue = np.reshape(depthValue, (640,480))
    depthValue = np.rot90(depthValue)

    grayLevel = data[:, 3].astype(np.uint8)
    grayLevel = np.reshape(grayLevel, (640,480))
    grayLevel = np.rot90(grayLevel)

    grayLevel_rgb = data[:, 3:].astype(np.uint8)
    grayLevel_rgb = np.reshape(grayLevel_rgb, (640,480,3))
    grayLevel_rgb = np.rot90(grayLevel_rgb)

    print_info(data)
    print_info(depthValue)
    print_info(grayLevel)
    print_info(grayLevel_rgb)

    # Plots
    plt.figure(1)
    plt.imshow(depthValue)
    plt.title('depthValue [meters]')
    plt.colorbar()

    plt.figure(2)
    # plt.imshow(grayLevel)
    plt.imshow(grayLevel, cmap='Greys')
    plt.title('grayLevel')
    plt.colorbar()

    plt.figure(3)
    plt.imshow(grayLevel_rgb)
    plt.title('grayLevel_rgb')

    # plt.show()
    plt.draw()
    plt.pause(1)

    input("<Hit Enter To Close>")
    plt.close('all')

    print('Done.')

if __name__ == '__main__':
    main()