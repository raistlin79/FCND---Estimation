import numpy as np
import matplotlib.pyplot as plt

gps_x = np.loadtxt('./config/log/Graph1.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]
acc_x = np.loadtxt('./config/log/Graph2.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]


gps_x_std = np.std(gps_x)
print('GPS X Std: {}'.format(gps_x_std))
acc_x_std = np.std(acc_x)
print('IMU X Std: {}'.format(acc_x_std))
