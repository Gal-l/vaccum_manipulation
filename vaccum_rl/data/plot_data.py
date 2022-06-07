from matplotlib import pyplot as plt
import pickle
import numpy as np
if __name__ == '__main__':
    file = open("Mon Jun  6 17:07:15 2022.pkl", 'rb')
    data = pickle.load(file)
    for ii in range(6):
        plt.plot(range(len(data)), np.rad2deg(data[:, ii]))
    plt.plot(range(len(data)), (data[:, 6])/100)
    plt.grid()
    plt.show()