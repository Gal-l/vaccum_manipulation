#!/usr/bin/env python  
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    logfile = '/home/picko/pickommerce_ws/src/calibration_abb_2_camera/logs/tf_camera_2_base_log_01-03-2022_18-53-15.csv'
    data = np.genfromtxt(logfile, delimiter=',')
    data = data[10:, :]

    print("mean: {}".format(np.mean(data, axis=0)))
    print("std: {}".format(np.std(data, axis=0)))
    
    x = np.arange(data.shape[0])
    fig, axs = plt.subplots(2, 4)
    
    axs[0, 0].plot(x, np.round(data[:, 0] * 1000), 'tab:orange')
    axs[0, 0].ticklabel_format(useOffset=False)
    axs[0, 0].set_title('translation x')
    
    axs[0, 1].plot(x, np.round(data[:, 1] * 1000), 'tab:orange')
    axs[0, 1].ticklabel_format(useOffset=False)
    axs[0, 1].set_title('translation y')

    axs[0, 2].plot(x, np.round(data[:, 2] * 1000), 'tab:orange')
    axs[0, 2].ticklabel_format(useOffset=False)
    axs[0, 2].set_title('translation z')

    axs[1, 0].plot(x, data[:, 3], 'tab:blue')
    axs[1, 0].set_title('rotation x')

    axs[1, 1].plot(x, data[:, 4], 'tab:blue')
    axs[1, 1].set_title('rotation y')

    axs[1, 2].plot(x, data[:, 5], 'tab:blue')
    axs[1, 2].set_title('rotation z')

    axs[1, 3].plot(x, data[:, 6], 'tab:blue')
    axs[1, 3].set_title('rotation w')

    plt.show()

